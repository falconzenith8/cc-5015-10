// 独立网格度量：只依赖网格顶点/面片、采样点集与解析真值，
// 不复用重建内核的质检函数，也不从流水线报告里取预期值。
import { pointTriDistance, dist } from './vec.js';
import { KDTree } from './kdtree.js';
import { pointInRegion } from './analytic.js';

// 被任一三角面引用的顶点集合（BPA 的 vertices 会包含全部原始点，未引用者不构成表面）
export function referencedVertexMask(mesh) {
  const mask = new Uint8Array(mesh.vertices.length);
  for (const t of mesh.faces) { mask[t[0]] = 1; mask[t[1]] = 1; mask[t[2]] = 1; }
  return mask;
}

// 面片落在未观测区域内的“面积份额”（0..1）。重心单点会把洞缘合法窄三角
// 误判为封闭，也会漏掉重心在外、面积却横跨洞角的面片；面积份额两者都能分辨。
// 采样点用固定重心坐标（确定性），不依赖重建内核。
const AREA_SAMPLES = [
  [1 / 3, 1 / 3, 1 / 3],
  [0.6, 0.2, 0.2], [0.2, 0.6, 0.2], [0.2, 0.2, 0.6],
  [0.45, 0.45, 0.1], [0.45, 0.1, 0.45], [0.1, 0.45, 0.45],
];
export function faceRegionCoverage(mesh, regions) {
  const V = mesh.vertices;
  const out = [];
  mesh.faces.forEach(([a, b, c], fi) => {
    const ab = [V[b][0] - V[a][0], V[b][1] - V[a][1], V[b][2] - V[a][2]];
    const ac = [V[c][0] - V[a][0], V[c][1] - V[a][1], V[c][2] - V[a][2]];
    const area = Math.hypot(
      ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0],
    ) / 2;
    if (!(area > 0)) return;
    for (const region of regions) {
      let hit = 0;
      for (const [wa, wb, wc] of AREA_SAMPLES) {
        const q = [
          V[a][0] * wa + V[b][0] * wb + V[c][0] * wc,
          V[a][1] * wa + V[b][1] * wb + V[c][1] * wc,
          V[a][2] * wa + V[b][2] * wb + V[c][2] * wc,
        ];
        if (pointInRegion(region, q)) hit++;
      }
      if (!hit) continue;
      out.push({
        faceIndex: fi, region: region.id, area,
        fraction: hit / AREA_SAMPLES.length,
        coveredArea: area * (hit / AREA_SAMPLES.length),
        centroid: [
          Number(((V[a][0] + V[b][0] + V[c][0]) / 3).toFixed(6)),
          Number(((V[a][1] + V[b][1] + V[c][1]) / 3).toFixed(6)),
          Number(((V[a][2] + V[b][2] + V[c][2]) / 3).toFixed(6)),
        ],
      });
      break;
    }
  });
  return out;
}

// 未观测区域的参考面积（box 取面内两轴；球冠取球冠面积；楔形取弧长×高）。
// 仅用于把“覆盖面积”归一化成可判定的比例，不从重建输出取值。
export function regionReferenceArea(region) {
  switch (region.kind) {
    case 'box': {
      const spans = [0, 1, 2].map((k) => region.max[k] - region.min[k]).sort((a, b) => b - a);
      return spans[0] * spans[1];
    }
    case 'sphereCap': {
      const R = region.radiusMax;
      return 2 * Math.PI * R * Math.max(0, R - Math.min(R, region.zMin));
    }
    case 'cylindricalWedge': {
      const R = (region.rMin + region.rMax) / 2;
      const sweep = region.angleRange[1] - region.angleRange[0];
      const height = region.zMax - region.zMin;
      return Math.abs(sweep) * R * height;
    }
    default:
      throw new Error(`未知未观测区域类型: ${region.kind}`);
  }
}

export function closureCoverage(mesh, regions) {
  const faces = faceRegionCoverage(mesh, regions);
  const total = regions.reduce((s, r) => s + regionReferenceArea(r), 0);
  const covered = faces.reduce((s, f) => s + f.coveredArea, 0);
  return { faces, coveredArea: covered, referenceArea: total, fraction: total > 0 ? covered / total : 0 };
}

// 被引用网格顶点 -> 解析真值的距离分布。
// 这是"重建表面到真值"的方向，能发现点->面距离看不见的漂移（例如整体偏移、错误所在面）。
export function meshToAnalyticDistances(mesh, analytic, distFn) {
  const mask = referencedVertexMask(mesh);
  const perVertex = [];
  let sum = 0, maxD = 0, maxIndex = -1;
  for (let i = 0; i < mesh.vertices.length; i++) {
    if (!mask[i]) continue;
    const d = distFn(analytic, mesh.vertices[i]);
    perVertex.push({ vertexIndex: i, distance: d });
    sum += d;
    if (d > maxD) { maxD = d; maxIndex = i; }
  }
  const finite = perVertex.map((x) => x.distance).filter(Number.isFinite).sort((a, b) => a - b);
  const q = (p) => (finite.length ? finite[Math.min(finite.length - 1, Math.floor(p * finite.length))] : null);
  return {
    count: finite.length,
    mean: finite.length ? Number((sum / finite.length).toExponential(6)) : null,
    median: finite.length ? Number(q(0.5).toExponential(6)) : null,
    p95: finite.length ? Number(q(0.95).toExponential(6)) : null,
    max: finite.length ? Number(maxD.toExponential(6)) : null,
    maxIndex,
    // 超过给定容差（调用方从独立采样式推导）的证据顶点
    overTolerance: (tol) => perVertex.filter((x) => !(x.distance <= tol)),
    perVertex: perVertex.map((x) => Number(x.distance.toExponential(6))),
  };
}

// 退化面：面积相对最长边的平方近零（不使用内核阈值）
export function degenerateFaceIndices(mesh, relThreshold = 1e-8) {
  const V = mesh.vertices;
  const bad = [];
  mesh.faces.forEach(([a, b, c], fi) => {
    const ab = dist(V[a], V[b]), bc = dist(V[b], V[c]), ca = dist(V[c], V[a]);
    const emax = Math.max(ab, bc, ca);
    if (emax < 1e-12) { bad.push(fi); return; }
    const s = (ab + bc + ca) / 2;
    const area = Math.sqrt(Math.max(0, s * (s - ab) * (s - bc) * (s - ca)));
    if (area / (emax * emax) < relThreshold) bad.push(fi);
  });
  return bad;
}

// 面片朝向与期望法线的一致率（用于"故意翻法线"的负向对照）
// expectedNormalFn(centroid) 返回期望的法线方向，返回 null 表示该处不参与统计
export function orientationAgreement(mesh, expectedNormalFn) {
  const V = mesh.vertices, F = mesh.faces;
  let agree = 0, total = 0;
  const disagreements = [];
  F.forEach(([a, b, c], fi) => {
    const ab = [V[b][0] - V[a][0], V[b][1] - V[a][1], V[b][2] - V[a][2]];
    const ac = [V[c][0] - V[a][0], V[c][1] - V[a][1], V[c][2] - V[a][2]];
    const fn = [
      ab[1] * ac[2] - ab[2] * ac[1],
      ab[2] * ac[0] - ab[0] * ac[2],
      ab[0] * ac[1] - ab[1] * ac[0],
    ];
    const len = Math.hypot(fn[0], fn[1], fn[2]);
    if (len < 1e-12) return;
    const centroid = [
      (V[a][0] + V[b][0] + V[c][0]) / 3,
      (V[a][1] + V[b][1] + V[c][1]) / 3,
      (V[a][2] + V[b][2] + V[c][2]) / 3,
    ];
    const exp = expectedNormalFn(centroid);
    if (!exp) return;
    total++;
    const d = (fn[0] * exp[0] + fn[1] * exp[1] + fn[2] * exp[2]) / len;
    if (d > 0) agree++;
    else disagreements.push({ faceIndex: fi, dot: Number(d.toFixed(4)) });
  });
  return { agree, total, ratio: total ? agree / total : null, disagreements };
}

// 拓扑：连通区域（顶点并查集）、边界边、非流形边、最大组件占比
export function meshTopology(mesh) {
  const V = mesh.vertices, F = mesh.faces;
  const referenced = referencedVertexMask(mesh);
  const parent = new Int32Array(V.length);
  for (let i = 0; i < V.length; i++) parent[i] = i;
  const find = (x) => {
    let r = x;
    while (parent[r] !== r) r = parent[r];
    while (parent[x] !== r) { const n = parent[x]; parent[x] = r; x = n; }
    return r;
  };
  const union = (a, b) => { const ra = find(a), rb = find(b); if (ra !== rb) parent[ra] = rb; };
  const edgeUse = new Map();
  const ek = (a, b) => (a < b ? `${a},${b}` : `${b},${a}`);
  for (const [a, b, c] of F) {
    union(a, b); union(b, c);
    for (const [x, y] of [[a, b], [b, c], [c, a]]) {
      const k = ek(x, y);
      edgeUse.set(k, (edgeUse.get(k) || 0) + 1);
    }
  }
  const comps = new Map();
  for (let i = 0; i < V.length; i++) {
    if (!referenced[i]) continue;
    const r = find(i);
    if (!comps.has(r)) comps.set(r, { root: r, vertices: 0, faces: 0 });
    comps.get(r).vertices++;
  }
  for (const t of F) comps.get(find(t[0])).faces++;
  const sorted = Array.from(comps.values()).sort((a, b) => b.faces - a.faces);
  let boundaryEdges = 0, nonManifoldEdges = 0;
  for (const n of edgeUse.values()) {
    if (n === 1) boundaryEdges++;
    else if (n > 2) nonManifoldEdges++;
  }
  return {
    componentCount: sorted.length,
    components: sorted,
    largestComponentFaces: sorted.length ? sorted[0].faces : 0,
    largestComponentFraction: F.length ? (sorted.length ? sorted[0].faces / F.length : 0) : 0,
    boundaryEdgeCount: boundaryEdges,
    nonManifoldEdgeCount: nonManifoldEdges,
    edgeUseCount: edgeUse.size,
  };
}

// 面片边长统计（"跨空洞搭桥"的独立证据）
export function edgeLengthStats(mesh) {
  const V = mesh.vertices;
  const lens = [];
  for (const [a, b, c] of mesh.faces) {
    lens.push(dist(V[a], V[b]), dist(V[b], V[c]), dist(V[c], V[a]));
  }
  lens.sort((x, y) => x - y);
  const q = (p) => (lens.length ? lens[Math.min(lens.length - 1, Math.floor(p * lens.length))] : null);
  return {
    count: lens.length,
    min: lens.length ? lens[0] : null,
    median: lens.length ? q(0.5) : null,
    p95: lens.length ? q(0.95) : null,
    max: lens.length ? lens[lens.length - 1] : null,
  };
}

// 点集最近邻间距（独立由点集计算；采样噪声与间距的独立参照）
export function nearestNeighborSpacing(points) {
  if (points.length < 2) return { count: 0, min: null, median: null, q95: null, max: null };
  const tree = new KDTree(points);
  const ds = points.map((p, i) => {
    const kn = tree.knn(p, 2, i);
    return kn.length ? Math.sqrt(kn[0].d2) : NaN;
  }).filter(Number.isFinite).sort((a, b) => a - b);
  const q = (p) => (ds.length ? ds[Math.min(ds.length - 1, Math.floor(p * ds.length))] : null);
  return { count: ds.length, min: ds.length ? ds[0] : null, median: q(0.5), q95: q(0.95), max: ds.length ? ds[ds.length - 1] : null };
}

// 采样点 -> 网格距离（有效区覆盖）。只从"被面片引用"的顶点出发做邻域 BFS，
// 避免未引用顶点把搜索引到没有面片的区域；无面片可达时显式记为 uncovered，绝不当 0。
export function pointSetToMeshDistances(points, mesh) {
  const V = mesh.vertices, F = mesh.faces;
  const mask = referencedVertexMask(mesh);
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  let refCount = 0;
  for (let i = 0; i < V.length; i++) {
    if (!mask[i]) continue;
    refCount++;
    for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], V[i][k]); max[k] = Math.max(max[k], V[i][k]); }
  }
  if (!refCount) {
    return { count: 0, uncoveredCount: points.length, mean: null, median: null, p95: null, max: null, maxIndex: -1, uncoveredIndices: points.map((_, i) => i), perPoint: points.map(() => null) };
  }
  const span = Math.max(max[0] - min[0], max[1] - min[1], max[2] - min[2], 1e-9);
  const cell = Math.max((span / Math.sqrt(Math.max(refCount, 1))) * 1.5, 1e-6);
  const grid = new Map();
  for (let i = 0; i < V.length; i++) {
    if (!mask[i]) continue;
    const key = V[i].map((x, d) => Math.floor((x - min[d]) / cell)).join(',');
    if (!grid.has(key)) grid.set(key, []);
    grid.get(key).push(i);
  }
  const vxFaces = new Map();
  F.forEach((t, fi) => {
    for (const v of t) {
      if (!vxFaces.has(v)) vxFaces.set(v, []);
      vxFaces.get(v).push(fi);
    }
  });
  const vxAdj = Array.from({ length: V.length }, () => []);
  for (const [a, b, c] of F) { vxAdj[a].push(b, c); vxAdj[b].push(a, c); vxAdj[c].push(a, b); }

  const nearestReferenced = (p) => {
    const base = p.map((x, d) => Math.floor((x - min[d]) / cell));
    let best = -1, bestD = Infinity;
    // 以切比雪夫环扩展，直到环的下界距离已超过当前最优
    for (let ring = 0; ring < 256; ring++) {
      if (best >= 0 && (ring - 1) * cell > Math.sqrt(bestD)) break;
      for (let dx = -ring; dx <= ring; dx++) for (let dy = -ring; dy <= ring; dy++) for (let dz = -ring; dz <= ring; dz++) {
        if (Math.max(Math.abs(dx), Math.abs(dy), Math.abs(dz)) !== ring) continue;
        for (const vi of grid.get(`${base[0] + dx},${base[1] + dy},${base[2] + dz}`) || []) {
          const d = dist(p, V[vi]);
          if (d < bestD) { bestD = d; best = vi; }
        }
      }
    }
    return best;
  };

  const perPoint = new Float64Array(points.length);
  const uncoveredIndices = [];
  let sum = 0, maxD = 0, maxIndex = -1;
  for (let i = 0; i < points.length; i++) {
    const seed = nearestReferenced(points[i]);
    let best = Infinity;
    if (seed >= 0) {
      const checked = new Set();
      const queue = [seed], seen = new Set([seed]);
      let hops = 0;
      while (queue.length && hops++ < 8192) {
        const v = queue.shift();
        for (const fi of vxFaces.get(v) || []) {
          if (checked.has(fi)) continue;
          checked.add(fi);
          const t = F[fi];
          const d = pointTriDistance(points[i], V[t[0]], V[t[1]], V[t[2]]);
          if (d < best) best = d;
        }
        // 仅当顶点本身仍可能更近时才继续扩展（点到顶点距离是到面距离的上界）
        if (dist(points[i], V[v]) <= best) {
          for (const w of vxAdj[v]) if (!seen.has(w)) { seen.add(w); queue.push(w); }
        }
      }
    }
    if (!Number.isFinite(best)) {
      uncoveredIndices.push(i);
      perPoint[i] = NaN;
      continue;
    }
    perPoint[i] = best;
    sum += best;
    if (best > maxD) { maxD = best; maxIndex = i; }
  }
  const finite = Array.from(perPoint).filter(Number.isFinite).sort((a, b) => a - b);
  const q = (p) => (finite.length ? finite[Math.min(finite.length - 1, Math.floor(p * finite.length))] : null);
  return {
    count: finite.length,
    uncoveredCount: uncoveredIndices.length,
    uncoveredIndices,
    mean: finite.length ? Number((sum / finite.length).toExponential(6)) : null,
    median: finite.length ? Number(q(0.5).toExponential(6)) : null,
    p95: finite.length ? Number(q(0.95).toExponential(6)) : null,
    max: finite.length ? Number(maxD.toExponential(6)) : null,
    maxIndex,
    perPoint: Array.from(perPoint, (d) => (Number.isFinite(d) ? Number(d.toExponential(6)) : null)),
  };
}

// 面片内部（重心坐标采样）到解析真值的距离分布。
// 只测顶点会漏掉“弦跨空洞”的薄片：三个顶点都在真值面上，片内却悬空。
// 采样点由固定重心坐标给出（确定性），不含任何重建内核的中间量。
const BARYCENTRIC_SAMPLES = [
  [1 / 3, 1 / 3, 1 / 3],
  [0.6, 0.2, 0.2], [0.2, 0.6, 0.2], [0.2, 0.2, 0.6],
  [0.45, 0.45, 0.1], [0.45, 0.1, 0.45], [0.1, 0.45, 0.45],
];
export function faceSamplesToAnalytic(mesh, analytic, distFn) {
  const V = mesh.vertices;
  const perFaceMax = new Float64Array(mesh.faces.length);
  const offenders = [];
  let total = 0, sum = 0, maxD = 0, maxFace = -1;
  mesh.faces.forEach(([a, b, c], fi) => {
    let fm = 0;
    for (const [wa, wb, wc] of BARYCENTRIC_SAMPLES) {
      const q = [
        V[a][0] * wa + V[b][0] * wb + V[c][0] * wc,
        V[a][1] * wa + V[b][1] * wb + V[c][1] * wc,
        V[a][2] * wa + V[b][2] * wb + V[c][2] * wc,
      ];
      const d = distFn(analytic, q);
      total++; sum += d;
      if (d > fm) fm = d;
      if (d > maxD) { maxD = d; maxFace = fi; }
    }
    perFaceMax[fi] = fm;
    offenders.push({ faceIndex: fi, maxDistance: fm });
  });
  return {
    sampleCount: total,
    mean: total ? Number((sum / total).toExponential(6)) : null,
    max: total ? Number(maxD.toExponential(6)) : null,
    maxFace,
    perFaceMax: Array.from(perFaceMax, (d) => Number(d.toExponential(6))),
    // 超过容差的面片（按片内最大偏差），调用方从独立参照给出容差
    overTolerance: (tol) => offenders.filter((o) => o.maxDistance > tol),
  };
}
