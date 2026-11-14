// 前端主控：Three.js 视窗 + 内核 schema 驱动控件 + 修订/失败联动。
// 所有几何数据来自后端内核；前端只做可视化，不自行"补面"。
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
let state = {
  cloud: null,
  run: null,
  selectedNoise: new Set(),
  scale: 1,
};

// ---------------- Three.js 场景 ----------------
const container = $('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
container.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0e14);
const camera = new THREE.PerspectiveCamera(55, 1, 0.01, 500);
camera.position.set(4.5, 3.5, 5.5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(5, 8, 6);
scene.add(dir);
scene.add(new THREE.AxesHelper(0.6));

let layers = { points: null, mesh: null, boundary: null, ambiguous: null, unsupported: null };

function clearLayer(obj) {
  if (!obj) return;
  scene.remove(obj);
  obj.geometry?.dispose();
  if (obj.material) (Array.isArray(obj.material) ? obj.material : [obj.material]).forEach((m) => m.dispose());
}
function clearAllLayers() {
  for (const k of Object.keys(layers)) { clearLayer(layers[k]); layers[k] = null; }
}

function resize() {
  const w = container.clientWidth, h = container.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(container);
resize();
(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

function frameBox(box) {
  const c = new THREE.Vector3();
  box.getCenter(c);
  const size = new THREE.Vector3();
  box.getSize(size);
  controls.target.copy(c);
  const r = Math.max(size.length(), 0.5);
  camera.position.set(c.x + r * 0.9, c.y + r * 0.7, c.z + r * 1.1);
  controls.update();
}

// ---------------- 点云渲染 ----------------
function renderCloud(cloud) {
  clearLayer(layers.points);
  clearLayer(layers.ambiguous);
  clearLayer(layers.unsupported);
  const n = cloud.points.length;
  const geo = new THREE.BufferGeometry();
  const pos = new Float32Array(n * 3);
  cloud.points.forEach((p, i) => { pos[i * 3] = p[0]; pos[i * 3 + 1] = p[1]; pos[i * 3 + 2] = p[2]; });
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  const mat = new THREE.PointsMaterial({ color: 0x9fd0ff, size: 0.035, sizeAttenuation: true });
  layers.points = new THREE.Points(geo, mat);
  layers.points.scale.setScalar(state.scale);
  scene.add(layers.points);
  applyVisibility();
  if (!geo.boundingBox) geo.computeBoundingBox();
  frameBox(geo.boundingBox);
}

// ---------------- 网格渲染 ----------------
function renderRun(run) {
  clearLayer(layers.mesh);
  clearLayer(layers.boundary);
  const { vertices, faces, boundaryEdges } = run.mesh;
  const geo = new THREE.BufferGeometry();
  const pos = new Float32Array(vertices.length * 3);
  vertices.forEach((p, i) => { pos[i * 3] = p[0]; pos[i * 3 + 1] = p[1]; pos[i * 3 + 2] = p[2]; });
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setIndex(faces.flat());
  geo.computeVertexNormals();
  const mat = new THREE.MeshStandardMaterial({
    color: 0x4c9aff, roughness: 0.65, metalness: 0.05,
    transparent: true, opacity: 0.55, side: THREE.DoubleSide,
  });
  layers.mesh = new THREE.Mesh(geo, mat);
  layers.mesh.scale.setScalar(state.scale);
  scene.add(layers.mesh);

  // 边界边：显式呈现开放边（缺墙/玻璃洞的诚实证据）
  if (boundaryEdges && boundaryEdges.length) {
    const epos = [];
    for (const [a, b] of boundaryEdges) {
      epos.push(...vertices[a], ...vertices[b]);
    }
    const eg = new THREE.BufferGeometry();
    eg.setAttribute('position', new THREE.Float32BufferAttribute(epos, 3));
    const em = new THREE.LineBasicMaterial({ color: 0xffb454 });
    layers.boundary = new THREE.LineSegments(eg, em);
    layers.boundary.scale.setScalar(state.scale);
    scene.add(layers.boundary);
  }
  renderFlagOverlays(run);
  applyVisibility();
}

// 法线歧义点（红）与未支撑点（橙）
function renderFlagOverlays(run) {
  clearLayer(layers.ambiguous);
  clearLayer(layers.unsupported);
  const cloud = state.cloud;
  if (!cloud) return;
  const flags = run.normals.flags;
  const ambIdx = [], unsIdx = new Set(run.mesh.unsupported.map((u) => u.index));
  flags.forEach((f, i) => { if (f & 4 || f & 8) ambIdx.push(i); });

  const makePoints = (indices, color, size) => {
    if (!indices.length) return null;
    const g = new THREE.BufferGeometry();
    const p = new Float32Array(indices.length * 3);
    indices.forEach((ci, k) => {
      const q = cloud.points[ci];
      p[k * 3] = q[0]; p[k * 3 + 1] = q[1]; p[k * 3 + 2] = q[2];
    });
    g.setAttribute('position', new THREE.BufferAttribute(p, 3));
    const m = new THREE.PointsMaterial({ color, size, sizeAttenuation: true });
    const pts = new THREE.Points(g, m);
    pts.scale.setScalar(state.scale);
    return pts;
  };
  layers.ambiguous = makePoints(ambIdx, 0xff4d4d, 0.06);
  if (layers.ambiguous) scene.add(layers.ambiguous);
  layers.unsupported = makePoints([...unsIdx], 0xffb454, 0.07);
  if (layers.unsupported) scene.add(layers.unsupported);
}

function applyVisibility() {
  if (layers.points) layers.points.visible = $('showPoints').checked;
  if (layers.mesh) layers.mesh.visible = $('showMesh').checked;
  if (layers.boundary) layers.boundary.visible = $('showBoundary').checked;
  if (layers.ambiguous) layers.ambiguous.visible = $('showAmbiguous').checked;
  if (layers.unsupported) layers.unsupported.visible = $('showUnsupported').checked;
}
['showPoints', 'showMesh', 'showBoundary', 'showAmbiguous', 'showUnsupported'].forEach((id) =>
  $(id).addEventListener('change', applyVisibility));

// 观测距离着色：用点到其观测传感器的距离映射颜色
$('colorDistance').addEventListener('change', (e) => {
  if (!layers.points || !state.cloud) return;
  if (e.target.checked) {
    const cloud = state.cloud;
    const dists = cloud.points.map((p, i) => {
      const v = cloud.views?.[i];
      return v ? Math.hypot(v[0] - p[0], v[1] - p[1], v[2] - p[2]) : 0;
    });
    const lo = Math.min(...dists), hi = Math.max(...dists, lo + 1e-6);
    const colors = new Float32Array(cloud.points.length * 3);
    const c = new THREE.Color();
    dists.forEach((d, i) => {
      c.setHSL(0.66 * (1 - (d - lo) / (hi - lo)), 0.9, 0.55);
      colors[i * 3] = c.r; colors[i * 3 + 1] = c.g; colors[i * 3 + 2] = c.b;
    });
    layers.points.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    layers.points.material.vertexColors = true;
  } else {
    layers.points.geometry.deleteAttribute('color');
    layers.points.material.vertexColors = false;
    layers.points.material.color.set(0x9fd0ff);
  }
  layers.points.material.needsUpdate = true;
});

// 显示尺度（统一缩放，不改变原始数据）
$('scaleInput').addEventListener('input', (e) => {
  state.scale = Number(e.target.value) || 1;
  for (const k of Object.keys(layers)) if (layers[k]) layers[k].scale.setScalar(state.scale);
});

// ---------------- API 封装 ----------------
async function api(path, opts = {}) {
  const res = await fetch(path, {
    headers: opts.body ? { 'content-type': 'application/json' } : undefined,
    ...opts,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw data;
  return data;
}

// ---------------- 场景载入 ----------------
async function loadScenarios() {
  const { scenarios } = await api('/api/scenarios');
  const box = $('scenarioList');
  box.innerHTML = '';
  scenarios.forEach((name) => {
    const b = document.createElement('button');
    b.className = 'btn';
    b.textContent = name;
    b.onclick = () => loadScenario(name);
    box.appendChild(b);
  });
}
async function loadScenario(name) {
  const r = await api('/api/load-scenario', { method: 'POST', body: JSON.stringify({ name }) });
  setCloud(r.cloud, r.scenario);
}
function setCloud(cloud, scenario) {
  state.cloud = cloud;
  state.run = null;
  state.selectedNoise.clear();
  clearAllLayers();
  renderCloud(cloud);
  const viewInfo = cloud.hasViews ? '含观测方向' : '无观测方向（朝向将标记歧义）';
  $('cloudMeta').innerHTML =
    `<b>${cloud.name}</b><br>点数 ${cloud.pointCount} · ${viewInfo}<br>` +
    `哈希 <code>${cloud.hash}</code> · 原点冻结不可改写` +
    (scenario ? `<br><span class="muted">${scenario.description || ''}</span>` : '');
  $('runBtn').disabled = false;
  $('removeNoiseBtn').disabled = true;
  $('restoreNoiseBtn').disabled = true;
  $('noiseList').innerHTML = '<span class="muted">运行重建后显示候选</span>';
  $('noiseCount').textContent = '运行后显示';
  resetPanels();
}

// ---------------- 上传 ----------------
$('fileInput').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const fd = new FormData();
  fd.append('file', file);
  try {
    const res = await fetch('/api/upload', { method: 'POST', body: fd });
    const data = await res.json();
    if (!res.ok) throw data;
    setCloud(data.cloud);
  } catch (err) {
    alert('上传失败：' + (err.error || '未知错误'));
  }
  e.target.value = '';
});

// ---------------- schema 驱动参数控件 ----------------
async function buildParamForm() {
  const { schema, defaults } = await api('/api/schema');
  const form = $('paramForm');
  form.innerHTML = '';
  const values = { ...defaults };
  schema.forEach((group) => {
    const h = document.createElement('div');
    h.className = 'param-stage';
    h.textContent = group.title;
    form.appendChild(h);
    group.params.forEach((p) => {
      const wrap = document.createElement('div');
      wrap.className = 'param';
      const lab = document.createElement('label');
      lab.textContent = p.label;
      wrap.appendChild(lab);
      let input;
      if (p.type === 'enum') {
        input = document.createElement('select');
        p.options.forEach((o) => { const op = document.createElement('option'); op.value = o; op.textContent = o; input.appendChild(op); });
      } else if (p.type === 'floatList') {
        input = document.createElement('input');
        input.type = 'text';
        input.value = JSON.stringify(p.default);
      } else {
        input = document.createElement('input');
        input.type = p.type === 'int' ? 'number' : 'number';
        input.step = p.step ?? (p.type === 'int' ? 1 : 0.1);
        if (p.min != null) input.min = p.min;
        if (p.max != null) input.max = p.max;
        input.value = p.default;
      }
      input.name = p.key;
      input.dataset.type = p.type;
      wrap.appendChild(input);
      form.appendChild(wrap);
    });
  });
  form._values = values;
}
function collectParams() {
  const out = {};
  $('paramForm').querySelectorAll('[name]').forEach((el) => {
    const t = el.dataset.type;
    if (t === 'enum') out[el.name] = el.value;
    else if (t === 'floatList') {
      try { out[el.name] = JSON.parse(el.value); } catch { out[el.name] = el.value.split(/[\s,]+/).map(Number).filter((x) => !Number.isNaN(x)); }
    } else if (t === 'int') out[el.name] = parseInt(el.value, 10);
    else out[el.name] = parseFloat(el.value);
  });
  return out;
}

// ---------------- 运行重建 ----------------
$('runBtn').addEventListener('click', async () => {
  $('runBtn').disabled = true;
  $('runBtn').textContent = '重建中…';
  hideFailure();
  try {
    const params = collectParams();
    const run = await api('/api/run', { method: 'POST', body: JSON.stringify(params) });
    state.run = run;
    renderRun(run);
    presentRun(run);
    presentNoise(run);
  } catch (err) {
    showFailure(err);
  } finally {
    $('runBtn').disabled = false;
    $('runBtn').textContent = '运行重建';
  }
});

function presentRun(run) {
  $('runMeta').innerHTML =
    `运行 <code>${run.runId}</code><br>方法 <b>${run.mesh.method}</b><br>` +
    `输入 ${run.frozen.pointCount} 点 · 哈希 <code>${run.frozen.cloudHash}</code><br>` +
    `<span class="muted">参数已快照冻结，原点不可改写</span>`;
  const q = run.qc;
  const dist = q.distances;
  $('qcPanel').innerHTML = `
    <div><span class="k">面片数</span><span class="v">${q.faceCount}</span></div>
    <div><span class="k">顶点数</span><span class="v">${q.vertexCount}</span></div>
    <div><span class="k">退化面</span><span class="v"><span class="pill ${q.degenerateCount ? 'bad' : 'ok'}">${q.degenerateCount}</span></span></div>
    <div><span class="k">连通区域</span><span class="v">${q.componentCount}</span></div>
    <div><span class="k">边界边</span><span class="v">${run.mesh.boundaryEdgeCount}</span></div>
    <div><span class="k">未支撑点</span><span class="v">${run.mesh.unsupported.length}</span></div>
    <div><span class="k">点到面距离 中位</span><span class="v">${dist.median ?? '—'}</span></div>
    <div><span class="k">点到面距离 p95</span><span class="v">${dist.p95 ?? '—'}</span></div>
    <div><span class="k">点到面距离 最大</span><span class="v">${dist.max ?? '—'}</span></div>
    <div><span class="k">未覆盖点</span><span class="v"><span class="pill ${dist.uncoveredCount ? 'bad' : 'ok'}">${dist.uncoveredCount}</span></span></div>`;
  $('topoNotice').textContent = q.topologyNotice;

  const region = $('regionPanel');
  if (run.mesh.regionReports && run.mesh.regionReports.length) {
    region.classList.remove('muted');
    region.innerHTML = run.mesh.regionReports.map((r) => `
      <div class="region-item">
        区域 #${r.regionIndex} · ${r.pointCount} 点 · 法线 [${r.normal.join(',')}]<br>
        观测栅格 ${r.gridCellsObserved} · <b style="color:var(--warn)">保留空栅格 ${r.gridCellsEmpty}</b> · 面片 ${r.faces}
      </div>`).join('');
  } else {
    region.className = 'region-list muted';
    region.textContent = 'BPA 后端无区域报告；开放区域以橙色边界边呈现。';
  }

  $('timingPanel').innerHTML = Object.entries(run.stageTimings)
    .map(([k, v]) => `<div class="row between"><span>${k}</span><span>${v} ms</span></div>`).join('');
}

// ---------------- 噪点候选与修订 ----------------
function presentNoise(run) {
  const cands = run.noise.candidates;
  $('noiseCount').textContent = `${cands.length} 个候选（仅标记，未删除）`;
  $('removeNoiseBtn').disabled = cands.length === 0;
  $('restoreNoiseBtn').disabled = false;
  const list = $('noiseList');
  list.innerHTML = '';
  cands.slice(0, 300).forEach((c) => {
    const item = document.createElement('label');
    item.className = 'noise-item';
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = true;
    cb.dataset.index = c.index;
    state.selectedNoise.add(c.index);
    cb.onchange = () => cb.checked ? state.selectedNoise.add(c.index) : state.selectedNoise.delete(c.index);
    const span = document.createElement('span');
    span.className = 'reasons';
    span.textContent = `#${c.index} · ${c.reasons.join('；')}`;
    span.title = c.reasons.join('\n');
    item.append(cb, span);
    list.appendChild(item);
  });
  if (cands.length > 300) list.insertAdjacentHTML('beforeend', `<div class="muted">仅显示前 300 / ${cands.length} 个</div>`);
}

$('removeNoiseBtn').addEventListener('click', async () => {
  const indices = [...state.selectedNoise];
  const r = await api('/api/revise', { method: 'POST', body: JSON.stringify({ mode: 'remove', indices, label: 'drop-noise' }) });
  setCloud(r.cloud);
});
$('restoreNoiseBtn').addEventListener('click', async () => {
  // 恢复：以最近一次运行的全部候选索引，从原始冻结点云重建回来（误删可恢复）
  const indices = state.run ? state.run.noise.candidates.map((c) => c.index) : [];
  const r = await api('/api/revise', { method: 'POST', body: JSON.stringify({ mode: 'restore', indices, label: 'restore-noise' }) });
  setCloud(r.cloud);
});

// ---------------- 失败面板（回传到点云重建阶段） ----------------
function showFailure(err) {
  const panel = $('failurePanel');
  panel.classList.remove('hidden');
  const pe = err.pipelineError;
  if (pe) {
    panel.innerHTML = `
      <h3>重建失败 · 阶段：${pe.failedStage}</h3>
      <div>${pe.message}</div>
      <pre>${pe.location ? JSON.stringify(pe.location, null, 2) : ''}</pre>
      <div class="notice" style="margin:6px 0">修复建议：${pe.hint}</div>
      <div class="muted">请回到左侧调整参数或在噪点面板恢复边界点后重新运行；缺墙与空洞不会被自动填补。</div>`;
  } else {
    panel.innerHTML = `<h3>请求失败</h3><pre>${err.error || JSON.stringify(err)}</pre>`;
  }
}
function hideFailure() { $('failurePanel').classList.add('hidden'); }
function resetPanels() {
  $('qcPanel').innerHTML = '';
  $('topoNotice').textContent = '';
  $('regionPanel').className = 'region-list muted';
  $('regionPanel').textContent = '运行后显示';
  $('timingPanel').innerHTML = '';
  $('runMeta').className = 'muted';
  $('runMeta').textContent = '尚无运行';
  hideFailure();
}

// ---------------- 启动 ----------------
buildParamForm();
loadScenarios();
