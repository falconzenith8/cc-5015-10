// 法线估计（PCA，局部自适应邻域）+ 观测方向定向 + 歧义显式标记
// 约定：法线朝"朝向观测者"的一侧，即 n · (view - p) > 0（view 为传感器位置时）；
// 若 views 存的是单位观测方向 d（传感器 -> 点），则 n · (-d) > 0。
// 本平台合成数据使用传感器位置 viewPoint，判定式为 n·(viewPoint - p) > 0。
import { KDTree } from './kdtree.js';
import { symEig3 } from './eig.js';
import { dot, normalize, sub } from './vec.js';

export function estimateNormals(points, scales, views, opts = {}) {
  const {
    neighborhoodFactor = 2.0,   // 邻域半径 = 局部尺度 × 该因子（逐点不同）
    minNeighbors = 4,
    maxNeighbors = 64,
    viewIsPosition = true,      // true: views 为传感器位置；false: 为方向向量
    orientationConsensus = 0.6, // 邻居定向一致率低于此值 -> 标记歧义
  } = opts;

  const n = points.length;
  const tree = new KDTree(points);
  const normals = new Array(n);
  const flags = new Array(n).fill(0); // bit0: 无观测定向  bit1: 邻域不足/平面性差  bit2: 定向冲突
  const planarity = new Float64Array(n);
  const neighborIds = new Array(n);

  for (let i = 0; i < n; i++) {
    const radius = Math.max(scales[i] * neighborhoodFactor, 1e-9);
    let nb = tree.radius(points[i], radius, i);
    if (nb.length < minNeighbors) {
      const kn = tree.knn(points[i], Math.min(minNeighbors + 1, n), i);
      nb = kn;
      flags[i] |= 1; // 半径邻域不足，退化到 kNN
    }
    if (nb.length > maxNeighbors) {
      nb.sort((a, b) => a.d2 - b.d2);
      nb = nb.slice(0, maxNeighbors);
    }
    neighborIds[i] = nb.map((r) => r.i);

    // 先求邻域质心，再构造距离加权协方差（锚点 != 质心会导致特征向量倾斜）
    let cx = 0, cy = 0, cz = 0, wsum = 0;
    for (const r of nb) {
      const q = points[r.i];
      const w = 1 / (1 + r.d2 / (radius * radius + 1e-18));
      cx += w * q[0]; cy += w * q[1]; cz += w * q[2]; wsum += w;
    }
    cx /= wsum; cy /= wsum; cz /= wsum;
    let sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
    for (const r of nb) {
      const q = points[r.i];
      const w = 1 / (1 + r.d2 / (radius * radius + 1e-18));
      const dx = q[0] - cx, dy = q[1] - cy, dz = q[2] - cz;
      sxx += w * dx * dx; syy += w * dy * dy; szz += w * dz * dz;
      sxy += w * dx * dy; sxz += w * dx * dz; syz += w * dy * dz;
    }
    const inv = 1 / wsum;
    const cov = [
      [sxx * inv, sxy * inv, sxz * inv],
      [sxy * inv, syy * inv, syz * inv],
      [sxz * inv, syz * inv, szz * inv],
    ];
    const { values, vectors } = symEig3(cov);
    let normal = vectors[0]; // 最小特征值对应特征向量
    const span = values[2] - values[0] + 1e-18;
    // 平面性：最小特征值（法向变化）相对总变化越小越平面；比值大 -> 非平面/退化
    const surfaceVarRatio = values[0] / (values[2] + 1e-18);
    planarity[i] = surfaceVarRatio;
    if (surfaceVarRatio > 0.12 || values[2] < 1e-14) flags[i] |= 2;

    // 观测方向定向
    const v = views?.[i];
    if (v) {
      const ref = viewIsPosition ? sub(v, points[i]) : scale3(v, -1);
      if (dot(normal, ref) < 0) normal = [-normal[0], -normal[1], -normal[2]];
      normals[i] = normal;
    } else {
      flags[i] |= 4;
      normals[i] = normal;
    }
  }

  // MST 平滑传播：让无观测约束的点与已定向邻居保持一致
  propagateOrientation(normals, flags, neighborIds, points);

  // 传播后再做一致性检查：邻居法线反向比例过高 -> 定向歧义
  for (let i = 0; i < n; i++) {
    const nb = neighborIds[i];
    if (!nb || nb.length < 3) { flags[i] |= 2; continue; }
    let agree = 0;
    for (const j of nb) if (dot(normals[i], normals[j]) >= 0) agree++;
    if (agree / nb.length < orientationConsensus) flags[i] |= 8;
  }

  return {
    normals: normals.map((x) => normalize(x)),
    flags,
    flagLabels: flagLabelTable(),
    planarity: Array.from(planarity, (v) => Number(v.toFixed(4))),
  };
}

function scale3(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }

// 以法线相似度为边权的最小生成树（Prim），从带观测定向的点出发传播
function propagateOrientation(normals, flags, neighborIds, points) {
  const n = points.length;
  const inTree = new Uint8Array(n);
  const parent = new Int32Array(n).fill(-1);
  // 优先从有观测定向的点开始；没有任何观测时从 0 号点开始（整体朝向任意，全部标歧义）
  const anyView = flags.some((f) => !(f & 4));
  const startWeights = new Float64Array(n).fill(Infinity);
  let start = 0;
  if (anyView) {
    let best = -Infinity;
    for (let i = 0; i < n; i++) {
      if (flags[i] & 4) continue;
      // 选邻居最多的观测点作根
      const w = neighborIds[i]?.length || 0;
      if (w > best) { best = w; start = i; }
    }
  } else {
    for (let i = 0; i < n; i++) flags[i] |= 4;
  }
  startWeights[start] = 0;

  for (let iter = 0; iter < n; iter++) {
    let u = -1, bestW = Infinity;
    for (let i = 0; i < n; i++) if (!inTree[i] && startWeights[i] < bestW) { bestW = startWeights[i]; u = i; }
    if (u < 0) break;
    inTree[u] = 1;
    if (parent[u] >= 0) {
      if (dot(normals[u], normals[parent[u]]) < 0) {
        normals[u] = [-normals[u][0], -normals[u][1], -normals[u][2]];
      }
    }
    for (const j of neighborIds[u] || []) {
      if (inTree[j]) continue;
      const w = 1 - Math.abs(dot(normals[u], normals[j]));
      if (w < startWeights[j]) { startWeights[j] = w; parent[j] = u; }
    }
  }
  // 未连通点（极端孤立）保持原朝向并标记
  for (let i = 0; i < n; i++) if (!inTree[i]) flags[i] |= 2;
}

export function flagLabelTable() {
  return {
    1: '半径邻域不足，退化使用 kNN',
    2: '平面性差/特征值退化，法线不可靠',
    4: '无观测方向，朝向由邻居传播推断（朝向歧义）',
    8: '邻居定向冲突，法线方向无法唯一确定',
  };
}
