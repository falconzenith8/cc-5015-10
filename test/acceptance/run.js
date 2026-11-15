// 点云重建自动验收：解析真值 + 独立度量 + 受控错误注入。
//
// 与 npm run selftest 的区别：
//  - selftest 是“旧样例跑到底”的回归（文件产出、旧约束保持）；
//  - 本验收集用解析曲面给出真值，额外注入离群点 / 采样空洞 / 密度骤变，
//    断言“有效区距离、拓扑、未观测区域处理”，并显式检查错误封闭；
//  - 所有阈值从 fixture 独立推导（点间距由点集最近邻测量、噪声幅度由构造给出），
//    绝不采用重建报告中的自述值，也不用默认值兜底。
//
// 有效区定义（独立由真值给出）：
//  - 排除未观测区域及其邻域（孔洞边缘的合法洞缘面不算错）；
//  - 排除真值面边界带（墙角/墙缘混合法线形成的过渡面单独按边带上界检查）；
//  - 其余面片内部采样到解析真值的距离必须落在独立容差内。
//
// 运行：
//   node test/acceptance/run.js            全量验收
//   node test/acceptance/run.js --replay   重放 failures/failure-points.json 中记录的失败点
//                                          （按 fixture 哈希 + 方法 + 参数 + 注入重建同一坏结果，
//                                           确认失败仍可复现；修复后重放应报告“不再复现”）
import { readFileSync, existsSync, mkdirSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { PointCloud } from '../../src/core/pointcloud.js';
import { runPipeline } from '../../src/core/pipeline.js';
import {
  pointToAnalyticDistance, expectedNormalAt, distanceToSurfaceBorder,
  pointInRegion, dilateRegion,
} from '../../src/core/analytic.js';
import {
  nearestNeighborSpacing, meshTopology, faceSamplesToAnalytic,
  closureCoverage, orientationAgreement, degenerateFaceIndices, edgeLengthStats,
  pointSetToMeshDistances,
} from '../../src/core/metrics.js';
import { fixturePath } from './generate.js';
import {
  injectFlippedViews, injectNoViews, injectShift, injectUnobservedFill, injectThinned,
} from './inject.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const FAILURE_DIR = resolve(__dirname, 'failures');

let pass = 0, fail = 0;
const checks = [];
const failurePoints = new Map();
// 负控是“应当失败”的对照项：它证明度量确实能抓到坏结果。
// 负控不参与通过/失败总账，但单独统计并在收尾显式报告，绝不靠计数器偏移隐瞒。
let negPass = 0, negFail = 0;

function check(caseKey, name, cond, detail = '', evidence = null) {
  checks.push({ case: caseKey, name, ok: !!cond, detail });
  if (cond) { pass++; return; }
  fail++;
  console.error(`  ✗ [${caseKey}] ${name} ${detail}`);
  if (evidence) addFailure(caseKey, name, evidence);
}

function checkNegativeControl(caseKey, name, cond, detail = '') {
  checks.push({ case: caseKey, name, ok: !!cond, detail, negativeControl: true });
  if (cond) { negPass++; return; }
  negFail++;
  console.error(`  ✗ 负控失效 [${caseKey}] ${name} ${detail}`);
}

function addFailure(caseKey, name, evidence) {
  if (!failurePoints.has(caseKey)) {
    failurePoints.set(caseKey, {
      case: caseKey, fixture: evidence.fixture, fixtureHash: evidence.fixtureHash,
      method: evidence.method, reason: name, params: evidence.params,
      injection: evidence.injection ?? null, points: [],
    });
  }
  failurePoints.get(caseKey).points.push(...evidence.points);
}
function section(t) { console.log(`\n=== ${t} ===`); }

// ---------------- fixture 与点云 ----------------
function loadFixture(name) {
  const path = fixturePath(name);
  if (!existsSync(path)) {
    console.error(`缺少 fixture ${path}；请先运行 node test/acceptance/generate.js`);
    process.exit(2);
  }
  const f = JSON.parse(readFileSync(path, 'utf8'));
  if (f.schema !== 'indoor-pc-acceptance/v1') throw new Error(`fixture schema 不匹配: ${f.schema}`);
  const h = createHash('sha256').update(JSON.stringify(f.points)).digest('hex').slice(0, 16);
  if (h !== f.hash) throw new Error(`fixture ${name} 哈希不匹配：磁盘=${h} 声明=${f.hash}`);
  return f;
}

function cleanSubset(f) {
  const exclude = new Set(f.metadata.cleanExcludeTags || []);
  const idx = [];
  for (let i = 0; i < f.points.length; i++) if (!exclude.has(f.tags[i])) idx.push(i);
  return { points: idx.map((i) => f.points[i]), views: idx.map((i) => f.views[i]), tags: idx.map((i) => f.tags[i]), originalIndices: idx };
}

// ---------------- 独立阈值推导 ----------------
// 核心容差：max(4×注入噪声幅度, 0.5×实测最近邻中位数)。
// 只有这两种量参与；重建的尺度/邻域/报告数值一律不参与。
function deriveTolerance(fixture, nnMedian) {
  const noise = fixture.metadata.noiseAmplitude;
  return {
    value: Math.max(4 * noise, 0.5 * nnMedian),
    rule: 'max(4×noiseAmplitude, 0.5×最近邻中位数)',
    noise,
    nnMedian,
  };
}

// 面片按有效区分类：
//  - excluded：落在（膨胀后的）未观测区域，或离真值面边界不足 borderFactor×nn；
//  - interior：其余面片（这些面片内部采样必须贴住真值）。
function classifyFaces(mesh, fixture, nnMedian, borderFactor = 2.0) {
  const dilated = fixture.unobserved.map((r) => dilateRegion(r, 0.75 * nnMedian));
  const V = mesh.vertices;
  const interior = [], border = [], excluded = [];
  mesh.faces.forEach(([a, b, c], fi) => {
    const cen = [0, 1, 2].map((k) => (V[a][k] + V[b][k] + V[c][k]) / 3);
    if (dilated.some((r) => pointInRegion(r, cen))) { excluded.push(fi); return; }
    const bd = distanceToSurfaceBorder(fixture.analytic, cen);
    if (bd >= borderFactor * nnMedian) interior.push(fi);
    else border.push(fi);
  });
  return { interior, border, excluded };
}

// BPA 是“插值”后端（顶点即采样点），期望面片内部紧贴真值。
// planar 是“栅格”后端（面片角点落在栅格上，与采样点不完全重合），
// 其连续面片内部偏差以栅格半对角线为自然上界；该上界从点间距独立推导，不来自重建报告。
function toleranceFor(method, tolValue, tolRule, nnMedian) {
  if (method === 'bpa') return { value: tolValue, rule: `有效区：${tolRule}` };
  const gridHalfDiag = Math.SQRT2 * nnMedian; // 栅格边长 ~nn，半对角线 ~0.707nn，取 √2×nn 作独立上界
  const bound = Math.max(tolValue * 2.5, gridHalfDiag);
  return { value: bound, rule: `栅格后端面片内部上界：max(2.5×${tolValue.toFixed(4)}, √2×点间距)` };
}

function evaluate(fixture, { points, views, method, params = {}, injection = null }) {
  const cloud = new PointCloud({ points, views, name: `${fixture.name}-${method}`, sourceFormat: 'json' });
  const result = runPipeline(cloud, { method, ...params });
  const mesh = result.mesh;
  const nn = nearestNeighborSpacing(points);
  const tol = deriveTolerance(fixture, nn.median);
  return {
    cloud, result, mesh, injection, fixture,
    topology: meshTopology(mesh),
    nn, tol,
    faceDist: faceSamplesToAnalytic(mesh, fixture.analytic, pointToAnalyticDistance),
    p2m: pointSetToMeshDistances(points, mesh),
    orientation: orientationAgreement(mesh, (p) => expectedNormalAt(fixture.analytic, p)),
    degenerate: degenerateFaceIndices(mesh),
    edges: edgeLengthStats(mesh),
  };
}

function evidence(ev, fixture, metric, limit, items, source, injection = null) {
  const pts = [];
  for (const item of items) {
    if (source === 'vertex') {
      const coords = ev.mesh.vertices[item.vertexIndex];
      if (!coords) continue;
      pts.push({ vertexIndex: item.vertexIndex, coords, value: item.distance, limit, metric });
    } else {
      const t = ev.mesh.faces[item.faceIndex];
      if (!t) continue;
      const V = ev.mesh.vertices;
      pts.push({
        faceIndex: item.faceIndex, vertices: t, region: item.region,
        centroid: [0, 1, 2].map((k) => Number(((V[t[0]][k] + V[t[1]][k] + V[t[2]][k]) / 3).toFixed(6))),
        value: item.maxDistance ?? item.value ?? 0, limit, metric,
      });
    }
  }
  return {
    fixture: fixture.name, fixtureHash: fixture.hash, method: ev.result.mesh.method,
    params: ev.result.frozen.paramsSnapshot, injection: injection ?? ev.injection, points: pts,
  };
}

// ---------------- 通用断言：有效区距离 / 拓扑 / 封闭 ----------------
function assertValidRegionDistance(fixture, ev, ck, { method }) {
  const { tol, nn } = ev;
  const cls = classifyFaces(ev.mesh, fixture, nn.median);
  const bound = toleranceFor(method, tol.value, tol.rule, nn.median);
  const perFace = ev.faceDist.perFaceMax;

  const interiorBad = cls.interior
    .map((fi) => ({ faceIndex: fi, maxDistance: perFace[fi] }))
    .filter((x) => x.maxDistance > bound.value);
  check(ck, '有效区面->真值距离在独立容差内', interiorBad.length === 0,
    `interior=${cls.interior.length} bad=${interiorBad.length} bound=${bound.value.toFixed(4)} (${bound.rule}) maxBad=${interiorBad.length ? Math.max(...interiorBad.map((x) => x.maxDistance)) : 0}`,
    evidence(ev, fixture, 'face->analytic', bound.value, interiorBad, 'face'));

  // 边界带（墙角/墙缘）仍须有有限上界：独立取 1.25×点间距，防止“倾斜大面片”借口边界带蒙混
  const borderLimit = 1.25 * nn.median;
  const borderBad = cls.border
    .map((fi) => ({ faceIndex: fi, maxDistance: perFace[fi] }))
    .filter((x) => x.maxDistance > borderLimit);
  check(ck, '边界带面片偏差不超过独立上界', borderBad.length === 0,
    `border=${cls.border.length} bad=${borderBad.length} limit=${borderLimit.toFixed(4)}`,
    evidence(ev, fixture, 'face->analytic(border)', borderLimit, borderBad, 'face'));

  // 有效区覆盖：干净采样点必须被面片覆盖（未覆盖意味着有效区出现空洞）
  check(ck, '有效区无未覆盖采样点', ev.p2m.uncoveredCount === 0,
    `uncovered=${ev.p2m.uncoveredCount}/${ev.p2m.count + ev.p2m.uncoveredCount}`);
}

function assertTopology(fixture, ev, ck, method) {
  const spec = fixture.topology[method] || {};
  check(ck, '无非流形边', ev.topology.nonManifoldEdgeCount <= (spec.nonManifoldMax ?? 0),
    `nonManifold=${ev.topology.nonManifoldEdgeCount}`);
  check(ck, '退化面为 0', ev.degenerate.length === 0, `degenerate=${ev.degenerate.length}`);
  if (spec.componentCountEquals != null) {
    check(ck, '连通区域数符合后端特性', ev.topology.componentCount === spec.componentCountEquals,
      `comps=${ev.topology.componentCount} expected=${spec.componentCountEquals}`);
  }
  if (spec.requireBoundaryEdges) {
    check(ck, '存在边界边（未观测开口证据）', ev.topology.boundaryEdgeCount > 0,
      `boundary=${ev.topology.boundaryEdgeCount}`);
  }
}

// 错误封闭：未观测区域被面片覆盖的面积占比。
// 判据完全独立：面积由面片几何给出，区域面积由构造给出。
// 洞缘的合法窄三角必然跨过孔边，但只占很小份额；把整洞盖住则是错误封闭。
// 阈值取 50%：干净洞缘远低于此（本验收集实测 BPA 13.9%），伪造回波远高于此（实测 96.9%）。
const CLOSURE_AREA_FRACTION = 0.5;

function assertNoFalseClosure(fixture, ev, ck) {
  const cov = closureCoverage(ev.mesh, fixture.unobserved);
  const bad = cov.faces.filter((f) => f.fraction > CLOSURE_AREA_FRACTION);
  check(ck, '未观测区域未被错误封闭（覆盖面积占比）', cov.fraction <= CLOSURE_AREA_FRACTION,
    `coveredFrac=${(cov.fraction * 100).toFixed(1)}% (${cov.coveredArea.toFixed(3)}/${cov.referenceArea.toFixed(3)}m²) limit=${(CLOSURE_AREA_FRACTION * 100).toFixed(0)}% badFaces=${bad.length}`,
    evidence(ev, fixture, 'closure-area', CLOSURE_AREA_FRACTION,
      bad.map((f) => ({ faceIndex: f.faceIndex, value: f.fraction, region: f.region })), 'face'));
}

// ---------------- 用例 1：盒式房间 + 孔洞 + 离群点 ----------------
function runBoxGlassHole(fixture) {
  const clean = cleanSubset(fixture);
  for (const method of fixture.metadata.methods) {
    const ck = `${fixture.name}/${method}`;
    const ev = evaluate(fixture, { points: clean.points, views: clean.views, method });
    assertValidRegionDistance(fixture, ev, ck, { method });
    assertTopology(fixture, ev, ck, method);
    assertNoFalseClosure(fixture, ev, ck);
    // 稀疏/空洞相邻的面片若无支撑，会产生超长边；独立上界 6×点间距
    const limit = 6 * ev.nn.median;
    check(ck, '无超长搭桥边', ev.edges.max <= limit,
      `edgeMax=${ev.edges.max?.toFixed(4)} limit=${limit.toFixed(4)}`);
  }
}

// ---------------- 用例 2：密度骤变 ----------------
function runBoxDensityJump(fixture) {
  const clean = cleanSubset(fixture);
  for (const method of fixture.metadata.methods) {
    const ck = `${fixture.name}/${method}`;
    const ev = evaluate(fixture, { points: clean.points, views: clean.views, method });
    assertValidRegionDistance(fixture, ev, ck, { method });
    assertTopology(fixture, ev, ck, method);

    // 稀疏核内不得出现“三顶点均在核外”的跨越面
    const spans = sparseSpanFaces(ev.mesh, fixture.sparseRegion);
    check(ck, '稀疏核内无无支撑跨越面', spans.length === 0, `spanFaces=${spans.length}`,
      evidence(ev, fixture, 'sparse-span', 0, spans.map((f) => ({ faceIndex: f.faceIndex })), 'face'));

    // 密度骤变下逐点局部尺度的离散必须真实存在（独立用点集测量，不读内核 scales）
    const spacingByRegion = measureSpacingDiscreteness(clean.points, fixture.sparseRegion);
    check(ck, '稀疏核点间距显著大于核外（密度骤变证据）',
      spacingByRegion.insideMedian !== null && spacingByRegion.outsideMedian !== null
        && spacingByRegion.insideMedian > spacingByRegion.outsideMedian * 1.8,
      `inside=${spacingByRegion.insideMedian?.toFixed(4)} outside=${spacingByRegion.outsideMedian?.toFixed(4)}`);
  }
}

function measureSpacingDiscreteness(points, region) {
  const inside = [], outside = [];
  // 逐点最近邻距离，按是否在稀疏核内分组。
  // 用全局点集求最近邻：核内点仍能借到核外近邻，反映“局部密度”而非“孤立子集距离”。
  const perPoint = nearestNeighborPerPoint(points);
  const inRegion = (p) => p[0] >= region.min[0] && p[0] <= region.max[0]
    && p[1] >= region.min[1] && p[1] <= region.max[1]
    && p[2] >= region.min[2] && p[2] <= region.max[2];
  for (let i = 0; i < points.length; i++) (inRegion(points[i]) ? inside : outside).push(perPoint[i]);
  inside.sort((a, b) => a - b); outside.sort((a, b) => a - b);
  const med = (arr) => (arr.length ? arr[arr.length >> 1] : null);
  return { insideMedian: med(inside), outsideMedian: med(outside), inPts: inside.length, outPts: outside.length };
}

function nearestNeighborPerPoint(points) {
  // 简单栅格加速的最近邻距离（独立实现，不依赖内核 stats）
  const n = points.length;
  let mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (const p of points) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[k]); mx[k] = Math.max(mx[k], p[k]); }
  const span = Math.max(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2], 1e-9);
  const cell = Math.max(span / Math.cbrt(n), 1e-9);
  const grid = new Map();
  points.forEach((p, i) => {
    const key = p.map((x, k) => Math.floor((x - mn[k]) / cell)).join(',');
    if (!grid.has(key)) grid.set(key, []);
    grid.get(key).push(i);
  });
  const out = new Float64Array(n).fill(Infinity);
  for (let i = 0; i < n; i++) {
    const base = points[i].map((x, k) => Math.floor((x - mn[k]) / cell));
    for (let ring = 0; ring < 64; ring++) {
      let found = ring === 0;
      for (let dx = -ring; dx <= ring; dx++) for (let dy = -ring; dy <= ring; dy++) for (let dz = -ring; dz <= ring; dz++) {
        if (Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) !== ring) continue;
        for (const j of grid.get(`${base[0] + dx},${base[1] + dy},${base[2] + dz}`) || []) {
          if (j === i) continue;
          found = true;
          const d = Math.hypot(points[i][0] - points[j][0], points[i][1] - points[j][1], points[i][2] - points[j][2]);
          if (d < out[i]) out[i] = d;
        }
      }
      if (found && ring > 0) break;
    }
  }
  return Array.from(out, (d) => (Number.isFinite(d) ? d : 0));
}

function sparseSpanFaces(mesh, region) {
  const V = mesh.vertices;
  const out = [];
  const inside = (p) => p[0] >= region.min[0] && p[0] <= region.max[0]
    && p[1] >= region.min[1] && p[1] <= region.max[1]
    && p[2] >= region.min[2] && p[2] <= region.max[2];
  mesh.faces.forEach((t, fi) => {
    const c = [0, 1, 2].map((k) => (V[t[0]][k] + V[t[1]][k] + V[t[2]][k]) / 3);
    if (!inside(c)) return;
    if (!inside(V[t[0]]) && !inside(V[t[1]]) && !inside(V[t[2]])) out.push({ faceIndex: fi });
  });
  return out;
}

// ---------------- 用例 3/4：解析曲面 ----------------
function runAnalyticSurface(fixture) {
  const clean = cleanSubset(fixture);
  for (const method of fixture.metadata.methods) {
    const ck = `${fixture.name}/${method}`;
    const ev = evaluate(fixture, { points: clean.points, views: clean.views, method });
    assertValidRegionDistance(fixture, ev, ck, { method });
    assertTopology(fixture, ev, ck, method);
    assertNoFalseClosure(fixture, ev, ck);
    const limit = 6 * ev.nn.median;
    check(ck, '无跨空洞超长边', ev.edges.max <= limit,
      `edgeMax=${ev.edges.max?.toFixed(4)} limit=${limit.toFixed(4)}`);
  }
}

// ---------------- 用例 5：法线极性正例/反例 ----------------
function runPlaneNormals(fixture) {
  const clean = cleanSubset(fixture);
  const expectDegrade = fixture.metadata.expectationMode === 'expect-degradation';
  for (const method of fixture.metadata.methods) {
    const ck = `${fixture.name}/${method}`;
    const ev = evaluate(fixture, { points: clean.points, views: clean.views, method });
    assertValidRegionDistance(fixture, ev, ck, { method });
    assertTopology(fixture, ev, ck, method);

    if (expectDegrade) {
      // 反例：传感器换到平面另一侧，法线朝向整体反转。
      // 期望“退化被捕获”：绕序与真值法线的一致率必须显著低，且保留反向面片索引供复跑。
      check(ck, '故意改变法线方向后定向一致率显著下降（退化被捕获）',
        ev.orientation.ratio !== null && ev.orientation.ratio < 0.2,
        `agreeRatio=${ev.orientation.ratio?.toFixed(3)} disagreeFaces=${ev.orientation.disagreements.length}`,
        evidence(ev, fixture, 'orientation', 0.2,
          ev.orientation.disagreements.map((d) => ({ faceIndex: d.faceIndex })), 'face'));
    } else {
      check(ck, '面片绕序与解析法线一致', ev.orientation.ratio !== null && ev.orientation.ratio >= 0.95,
        `agreeRatio=${ev.orientation.ratio?.toFixed(3)}`);
    }
  }
}

// ---------------- 噪点筛查（只标记不删、可恢复） ----------------
function runOutlierScreening(fixture) {
  const ck = `${fixture.name}/screening`;
  if (!fixture.metadata.outlierTags?.length) return;
  const cloud = new PointCloud({ points: fixture.points, views: fixture.views, name: fixture.name, sourceFormat: 'json' });
  const result = runPipeline(cloud, { method: fixture.metadata.methods[0] });
  const candSet = new Set(result.noise.candidates.map((c) => c.index));
  const farIdx = [], interiorIdx = [];
  fixture.tags.forEach((t, i) => {
    if (t === 'outlier-far') farIdx.push(i);
    if (t === 'outlier-interior') interiorIdx.push(i);
  });
  const farHit = farIdx.filter((i) => candSet.has(i)).length;
  check(ck, '远距离离群点全部被标记', farHit === farIdx.length, `hit=${farHit}/${farIdx.length}`);
  check(ck, '原点云哈希未被筛查改写', result.frozen.cloudHash === cloud.hash && cloud.n === fixture.pointCount,
    `hash=${cloud.hash}`);

  const removed = new Set(result.noise.candidates.map((c) => c.index));
  const keepMask = fixture.points.map((_, i) => !removed.has(i));
  const { cloud: revised } = cloud.revise({ keepMask, label: 'drop-noise' });
  const restoreMask = fixture.points.map((_, i) => removed.has(i));
  const { cloud: restored } = cloud.revise({ keepMask: restoreMask, label: 'restore-noise' });
  check(ck, '剔除候选产生独立新点云', revised.id !== cloud.id);
  check(ck, '候选可从原点恢复且原点不变',
    restored.n === removed.size && cloud.n === fixture.pointCount,
    `restored=${restored.n} removed=${removed.size}`);
}

// ---------------- 受控错误注入（正反例） ----------------
function runInjectedNegatives(fixture) {
  const ck = `${fixture.name}/inject`;
  const clean = cleanSubset(fixture);
  const method = fixture.metadata.methods[0];

  // (a) 未观测区域注入伪造回波 -> 错误封闭必须被检出
  if (fixture.unobserved?.length) {
    const filled = injectUnobservedFill({ ...fixture, points: clean.points, views: clean.views, tags: clean.tags });
    const ev = evaluate(fixture, {
      points: filled.points, views: filled.views, method,
      injection: { kind: 'unobserved-fill', seed: 9001, spacing: 0.08, description: filled.description },
    });
    const cov = closureCoverage(ev.mesh, fixture.unobserved);
    checkNegativeControl(ck, '注入伪造回波后错误封闭被捕获', cov.fraction > CLOSURE_AREA_FRACTION,
      `coveredFrac=${(cov.fraction * 100).toFixed(1)}% 未超过 ${(CLOSURE_AREA_FRACTION * 100).toFixed(0)}%（检测失效）`);
    if (cov.fraction <= CLOSURE_AREA_FRACTION) {
      // 负控自身失效也要留下执行证据，便于复跑定位
      addFailure(ck, '负控失效：伪造回波未触发错误封闭', {
        fixture: fixture.name, fixtureHash: fixture.hash, method: ev.result.mesh.method,
        params: ev.result.frozen.paramsSnapshot,
        injection: ev.injection,
        points: filled.injected.map((p, k) => ({ injectedCoords: p, region: fixture.unobserved[0].id, value: cov.fraction, limit: CLOSURE_AREA_FRACTION, metric: 'closure-negative-control', index: k })),
      });
    }
  }

  // (b) 整体平移 -> 面->真值距离必须显著变大
  {
    const shifted = injectShift({ points: clean.points, views: clean.views }, 0.15);
    const ev = evaluate(fixture, {
      points: shifted.points, views: shifted.views, method,
      injection: { kind: 'shift', delta: 0.15, description: shifted.description },
    });
    checkNegativeControl(ck, '注入整体平移 0.15m 后距离度量显著变大', ev.faceDist.max >= 0.10,
      `faceSampleMax=${ev.faceDist.max}`);
  }

  // (c) 故意翻转法线 -> 定向一致率下降
  {
    const flipped = injectFlippedViews({ points: clean.points, views: clean.views });
    const ev = evaluate(fixture, {
      points: flipped.points, views: flipped.views, method,
      injection: { kind: 'flip-views', description: flipped.description },
    });
    checkNegativeControl(ck, '注入翻转法线后定向一致率下降', ev.orientation.ratio === null || ev.orientation.ratio < 0.5,
      `agreeRatio=${ev.orientation.ratio?.toFixed(3)}`);
  }

  // (d) 移除全部观测 -> 朝向歧义被显式标记
  {
    const noViews = injectNoViews({ points: clean.points, views: clean.views });
    const ev = evaluate(fixture, {
      points: noViews.points, views: noViews.views, method,
      injection: { kind: 'no-views', description: noViews.description },
    });
    const ambiguous = ev.result.normals.flags.filter((f) => f & 4).length;
    checkNegativeControl(ck, '移除观测后朝向歧义被显式标记', ambiguous > 0, `flaggedNoView=${ambiguous}/${ev.result.normals.flags.length}`);
  }

  // (e) 抽稀一半采样 -> 有效区点->面距离不下降（覆盖度量确实在运算）
  {
    const thinned = injectThinned({ points: clean.points, views: clean.views, tags: clean.tags }, 0.5);
    if (thinned.points.length > 10) {
      const base = evaluate(fixture, { points: clean.points, views: clean.views, method });
      const thin = evaluate(fixture, {
        points: thinned.points, views: thinned.views, method,
        injection: { kind: 'thinned', keep: 0.5, seed: 7001, description: thinned.description },
      });
      checkNegativeControl(ck, '抽稀后有效区点->面距离不下降', (thin.p2m.p95 ?? 0) >= (base.p2m.p95 ?? 0),
        `baseP95=${base.p2m.p95} thinP95=${thin.p2m.p95}`);
    }
  }
}

// ---------------- 旧样例回归（改动范围复核） ----------------
function runLegacyRegression() {
  const dataDir = resolve(__dirname, '../../data');
  const required = ['s1_planar_room', 's2_density_jump', 's3_isolated_noise', 's4_missing_wall', 's5_glass_hole'];
  const missing = required.filter((n) => !existsSync(resolve(dataDir, `${n}.json`)));
  if (missing.length) {
    console.log(`  （跳过旧样例回归：缺少 ${missing.join(', ')}，先运行 npm run gen-data）`);
    return;
  }
  for (const name of required) {
    const d = JSON.parse(readFileSync(resolve(dataDir, `${name}.json`), 'utf8'));
    const cloud = new PointCloud({ points: d.points, views: d.views, name, sourceFormat: 'json' });
    for (const method of ['planar', 'bpa']) {
      const t0 = Date.now();
      const r = runPipeline(cloud, { method });
      const ms = Date.now() - t0;
      const topo = meshTopology(r.mesh);
      const ck = `legacy/${name}/${method}`;
      check(ck, '重建产出非空网格', r.mesh.faces.length > 0, `faces=${r.mesh.faces.length} ms=${ms}`);
      check(ck, '无非流形边', topo.nonManifoldEdgeCount === 0, `nonManifold=${topo.nonManifoldEdgeCount}`);
      if (name === 's4_missing_wall') {
        const V = r.mesh.vertices;
        let near = 0;
        for (const [a, b, c] of r.mesh.faces) if ((V[a][0] + V[b][0] + V[c][0]) / 3 > 1.95) near++;
        check(ck, '开口 +X 零面片（旧红线）', near === 0, `facesNearXp=${near}`);
      }
      if (name === 's5_glass_hole' && method === 'planar') {
        const empty = r.mesh.regionReports.reduce((s, x) => s + x.gridCellsEmpty, 0);
        check(ck, '玻璃洞保留空栅格', empty > 0, `emptyCells=${empty}`);
      }
    }
  }
}

// ---------------- 失败点重放 ----------------
// 按记录重建同一输入（fixture + 注入 + 方法 + 参数），把记录里每个点/面重新度量。
// 目的：失败点集不是静态日志，而是可执行、可复跑的回归入口。
// 退出码：仍有失败点复现 -> 0（复现成功，说明缺陷仍在，属预期）；
//         全部不再复现 -> 0 并报告已修复；无法重建输入 -> 2。
function replayFailurePoints() {
  const fpPath = resolve(FAILURE_DIR, 'failure-points.json');
  if (!existsSync(fpPath)) {
    console.log(`无失败点集可重放（${fpPath} 不存在）。`);
    return 0;
  }
  const doc = JSON.parse(readFileSync(fpPath, 'utf8'));
  console.log(`重放失败点集：${doc.count} 个用例，记录于 ${doc.generatedAt}`);
  let reproduced = 0, resolved = 0, invalid = 0;
  for (const c of doc.cases) {
    const fixture = loadFixture(c.fixture);
    if (fixture.hash !== c.fixtureHash) {
      console.error(`  ✗ [${c.case}] fixture 哈希不匹配（磁盘 ${fixture.hash} / 记录 ${c.fixtureHash}）：无法复现`);
      invalid++;
      continue;
    }
    const clean = cleanSubset(fixture);
    let points = clean.points, views = clean.views, tags = clean.tags;
    if (c.injection?.kind === 'unobserved-fill') {
      const filled = injectUnobservedFill({ ...fixture, points, views, tags }, { spacing: c.injection.spacing, seed: c.injection.seed });
      ({ points, views } = filled);
    } else if (c.injection?.kind === 'shift') {
      ({ points, views } = injectShift({ points, views }, c.injection.delta));
    } else if (c.injection?.kind === 'flip-views') {
      ({ points, views } = injectFlippedViews({ points, views }));
    } else if (c.injection?.kind === 'no-views') {
      ({ points, views } = injectNoViews({ points, views }));
    } else if (c.injection?.kind === 'thinned') {
      ({ points, views } = injectThinned({ points, views, tags }, c.injection.keep));
    } else if (c.injection) {
      console.error(`  ✗ [${c.case}] 未知注入类型 ${c.injection.kind}：无法复现`);
      invalid++;
      continue;
    }
    let ev;
    try {
      ev = evaluate(fixture, { points, views, method: c.method });
    } catch (e) {
      console.error(`  ✗ [${c.case}] 重建失败：${e.message}`);
      invalid++;
      continue;
    }
    const hits = recheckPoints(ev, c);
    if (hits.length) {
      reproduced++;
      console.log(`  ↻ [${c.case}] 复现 ${hits.length}/${c.points.length} 个失败点（${c.reason}）`);
    } else {
      resolved++;
      console.log(`  ✓ [${c.case}] 记录中的失败点已不再超标（${c.reason}）`);
    }
  }
  console.log(`\n重放结果：复现 ${reproduced} 个用例，已修复 ${resolved} 个用例，无法复现 ${invalid} 个用例`);
  return invalid ? 2 : 0;
}

// 用当前重建结果重新度量记录中的点/面，判断是否仍超过阈值。
// 阈值用当前运行独立推导的 limit，而不是记录里的旧值：这样“已修复”真的能显式报告，
// 同时把旧阈值与当前阈值一并写出，避免用宽松新阈值悄悄抹掉历史失败。
function recheckPoints(ev, c) {
  const perFace = ev.faceDist.perFaceMax;
  const hits = [];
  for (const p of c.points) {
    const limit = currentLimitFor(ev, p, c);
    if (p.metric === 'face->analytic' || p.metric === 'face->analytic(border)') {
      const d = perFace[p.faceIndex];
      if (d != null && d > limit * (1 + 1e-9)) hits.push({ ...p, replayed: d, currentLimit: limit });
    } else if (p.metric === 'closure-area' || p.metric === 'closure') {
      const region = ev.fixture.unobserved.find((r) => r.id === p.region);
      if (!region) continue;
      const cov = closureCoverage(ev.mesh, [region]);
      if (cov.fraction > limit) hits.push({ ...p, replayed: cov.fraction, currentLimit: limit });
    } else if (p.metric === 'orientation') {
      const dis = new Set(ev.orientation.disagreements.map((x) => x.faceIndex));
      if (dis.has(p.faceIndex)) hits.push({ ...p, replayed: 'disagreement', currentLimit: limit });
    } else if (p.metric === 'sparse-span') {
      const spans = sparseSpanFaces(ev.mesh, ev.fixture.sparseRegion);
      if (spans.some((s) => s.faceIndex === p.faceIndex)) hits.push({ ...p, replayed: 'span', currentLimit: limit });
    }
  }
  return hits;
}

function currentLimitFor(ev, p, c) {
  if (p.metric === 'face->analytic(border)') return 1.25 * ev.nn.median;
  if (p.metric === 'face->analytic') return toleranceFor(c.method, ev.tol.value, ev.tol.rule, ev.nn.median).value;
  if (p.metric === 'closure-area' || p.metric === 'closure') return CLOSURE_AREA_FRACTION;
  if (p.metric === 'orientation') return 0.2;
  return p.limit;
}

// ---------------- 主流程 ----------------
function main() {
  console.log('点云重建自动验收集');
  if (process.argv.includes('--replay')) process.exit(replayFailurePoints());

  const fixtures = [
    'box_glass_hole', 'box_density_jump', 'sphere_cap_hole',
    'cylinder_wedge_hole', 'plane_normals_ok', 'plane_normals_flipped',
  ].map(loadFixture);

  section('盒式房间：观测孔洞 + 离群点（有效区距离 / 拓扑 / 错误封闭）');
  for (const f of fixtures.filter((x) => x.name === 'box_glass_hole')) {
    runBoxGlassHole(f); runOutlierScreening(f); runInjectedNegatives(f);
  }
  section('盒式房间：密度骤变（有效区距离 / 无无支撑跨越面）');
  for (const f of fixtures.filter((x) => x.name === 'box_density_jump')) {
    runBoxDensityJump(f); runInjectedNegatives(f);
  }
  section('解析曲面：球面观测空洞');
  for (const f of fixtures.filter((x) => x.name === 'sphere_cap_hole')) {
    runAnalyticSurface(f); runInjectedNegatives(f);
  }
  section('解析曲面：圆柱楔形空洞');
  for (const f of fixtures.filter((x) => x.name === 'cylinder_wedge_hole')) runAnalyticSurface(f);
  section('法线极性正例 / 反例（故意翻转必须被捕获）');
  for (const f of fixtures.filter((x) => x.name.startsWith('plane_normals_'))) runPlaneNormals(f);
  section('旧样例回归（改动范围复核：旧样例运行到底）');
  runLegacyRegression();

  // 失败点集落盘（可复跑：fixture 哈希 + 方法 + 参数 + 点/面索引 + 指标 + 阈值）
  const fpPath = resolve(FAILURE_DIR, 'failure-points.json');
  if (failurePoints.size) {
    mkdirSync(FAILURE_DIR, { recursive: true });
    writeFileSync(fpPath, JSON.stringify({
      schema: 'indoor-pc-acceptance-failures/v1',
      generatedAt: new Date().toISOString(),
      count: failurePoints.size,
      cases: [...failurePoints.values()],
    }, null, 2));
    console.log(`\n失败点集已写出: ${fpPath}（${failurePoints.size} 个用例）`);
  } else if (existsSync(fpPath)) {
    console.log(`\n本次无失败；历史失败点集保留于 ${fpPath}`);
  }

  mkdirSync(FAILURE_DIR, { recursive: true });
  writeFileSync(resolve(FAILURE_DIR, 'last-run.json'), JSON.stringify({
    schema: 'indoor-pc-acceptance-report/v1',
    finishedAt: new Date().toISOString(),
    pass, fail, negativePass: negPass, negativeFail: negFail,
    checks,
    failures: [...failurePoints.values()],
  }, null, 2));

  console.log(`\n结果：通过 ${pass} 项，失败 ${fail} 项；负控 ${negPass} 通过 / ${negFail} 失效`);
  if (fail > 0 || negFail > 0) {
    console.error('验收失败：见上方 ✗ 项与 failures/failure-points.json');
    process.exit(1);
  }
  console.log('自动验收全部通过：有效区距离、拓扑与未观测区域处理均符合独立参照；受控错误均被暴露。');
}

if (process.argv[1] === fileURLToPath(import.meta.url)) main();
