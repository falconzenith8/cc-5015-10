// 输出检查：退化面、连通区域、原点到生成面的距离统计
// 注意：距离小只说明几何贴合，不自动证明拓扑真实（报告中显式声明）
import { triArea, pointTriDistance, dist } from './vec.js';

export function checkMesh(mesh, points, { usedPoint = null, degenerateAreaThreshold = 1e-10 } = {}) {
  const { vertices, faces } = mesh;

  // 1) 退化面
  const degenerate = [];
  const areas = new Float64Array(faces.length);
  faces.forEach((t, fi) => {
    const a = triArea(vertices[t[0]], vertices[t[1]], vertices[t[2]]);
    areas[fi] = a;
    if (a < degenerateAreaThreshold) {
      degenerate.push({ faceIndex: fi, area: Number(a.toExponential(4)), vertices: t });
    }
  });

  // 2) 连通区域（边邻接 flood fill）；未被任何面片引用的孤立顶点不计入组件
  const adj = new Map();
  const addAdj = (a, b) => {
    if (!adj.has(a)) adj.set(a, []);
    adj.get(a).push(b);
  };
  const referenced = new Set();
  for (const [a, b, c] of faces) {
    referenced.add(a); referenced.add(b); referenced.add(c);
    addAdj(a, b); addAdj(b, a); addAdj(b, c); addAdj(c, b); addAdj(c, a); addAdj(a, c);
  }
  const compOf = new Int32Array(vertices.length).fill(-1);
  const components = [];
  for (const s of referenced) {
    if (compOf[s] !== -1) continue;
    const id = components.length;
    let vCount = 0, fSet = 0;
    const faceSet = new Set();
    const stack = [s];
    compOf[s] = id;
    while (stack.length) {
      const u = stack.pop();
      vCount++;
      for (const w of adj.get(u) || []) {
        if (compOf[w] === -1) { compOf[w] = id; stack.push(w); }
      }
    }
    for (let fi = 0; fi < faces.length; fi++) {
      const t = faces[fi];
      if (compOf[t[0]] === id) { faceSet.add(fi); fSet++; }
    }
    components.push({ componentIndex: id, vertices: vCount, faces: fSet });
  }
  components.sort((a, b) => b.faces - a.faces);

  // 3) 点到网格距离：用网格顶点建最近候选 + 候选三角片精查
  const stats = pointToMeshDistances(points, mesh);

  return {
    faceCount: faces.length,
    vertexCount: vertices.length,
    degenerateFaces: degenerate,
    degenerateCount: degenerate.length,
    components,
    componentCount: components.length,
    distances: stats,
    topologyNotice: '点到网格距离小仅表示几何贴合，不自动证明拓扑真实；空洞、缺墙与朝向需结合边界与法线标记人工确认。',
  };
}

// 每个点找最近网格顶点，再检查其关联三角片的精确距离
function pointToMeshDistances(points, mesh) {
  const { vertices, faces } = mesh;
  const vxFaces = new Map();
  faces.forEach((t, fi) => {
    for (const v of t) {
      if (!vxFaces.has(v)) vxFaces.set(v, []);
      vxFaces.get(v).push(fi);
    }
  });
  // 网格顶点空间栅格（找最近种子）+ 面片边顶点邻接（BFS 扩展）
  const grid = buildGrid(vertices);
  const vxAdj = Array.from({ length: vertices.length }, () => []);
  for (const [a, b, c] of faces) {
    vxAdj[a].push(b, c); vxAdj[b].push(a, c); vxAdj[c].push(a, b);
  }
  const perPoint = new Float64Array(points.length);
  let sum = 0, maxD = 0, maxIdx = -1;
  const uncovered = []; // 网格未覆盖的点（无任何可达面片），距离不可定义而非 0
  for (let i = 0; i < points.length; i++) {
    const seedV = nearestGridVertex(grid, vertices, points[i]);
    const checked = new Set();
    let best = Infinity;
    const queue = [seedV];
    const seen = new Set([seedV]);
    let hops = 0;
    while (queue.length && hops < 4096) {
      const v = queue.shift(); hops++;
      for (const fi of vxFaces.get(v) || []) {
        if (checked.has(fi)) continue;
        checked.add(fi);
        const t = faces[fi];
        const d = pointTriDistance(points[i], vertices[t[0]], vertices[t[1]], vertices[t[2]]);
        if (d < best) best = d;
      }
      const dv = dist(points[i], vertices[v]);
      if (dv < best) {
        for (const w of vxAdj[v] || []) if (!seen.has(w)) { seen.add(w); queue.push(w); }
      }
    }
    if (!Number.isFinite(best)) {
      uncovered.push(i);
      perPoint[i] = NaN;
      continue;
    }
    perPoint[i] = best;
    sum += best;
    if (best > maxD) { maxD = best; maxIdx = i; }
  }
  const finite = Array.from(perPoint).filter(Number.isFinite).sort((a, b) => a - b);
  const q = (p) => (finite.length ? finite[Math.min(finite.length - 1, Math.floor(p * finite.length))] : NaN);
  return {
    mean: finite.length ? Number((sum / finite.length).toExponential(6)) : null,
    median: finite.length ? Number(q(0.5).toExponential(6)) : null,
    p95: finite.length ? Number(q(0.95).toExponential(6)) : null,
    max: finite.length ? Number(maxD.toExponential(6)) : null,
    maxIndex: maxIdx,
    coveredCount: finite.length,
    uncoveredCount: uncovered.length,
    uncoveredIndices: uncovered,
    perPoint: Array.from(perPoint, (d) => (Number.isFinite(d) ? Number(d.toExponential(6)) : null)),
  };
}

function buildGrid(vertices) {
  // 统一边长栅格 + 顶点邻接链接（来自面片边）
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (const p of vertices) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[k]); mx[k] = Math.max(mx[k], p[k]); }
  const span = Math.max(...mx.map((v, k) => v - mn[k]), 1e-9);
  const cell = span / Math.cbrt(Math.max(vertices.length, 1)) * 2 || 1e-6;
  const cells = new Map();
  vertices.forEach((p, i) => {
    const key = p.map((x, k) => Math.floor((x - mn[k]) / cell)).join(',');
    if (!cells.has(key)) cells.set(key, []);
    cells.get(key).push(i);
  });
  return { cells, mn, cell };
}

function nearestGridVertex(grid, vertices, p) {
  const { cells, mn, cell } = grid;
  const base = p.map((x, k) => Math.floor((x - mn[k]) / cell));
  for (let ring = 0; ring < 64; ring++) {
    let best = -1, bestD = Infinity;
    for (let dx = -ring; dx <= ring; dx++) for (let dy = -ring; dy <= ring; dy++) for (let dz = -ring; dz <= ring; dz++) {
      if (Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) !== ring) continue;
      const key = `${base[0] + dx},${base[1] + dy},${base[2] + dz}`;
      for (const vi of cells.get(key) || []) {
        const d = dist(p, vertices[vi]);
        if (d < bestD) { bestD = d; best = vi; }
      }
    }
    if (best >= 0) return best;
  }
  return 0;
}
