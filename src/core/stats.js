// 局部尺度估计：每个点的邻域尺度随局部密度变化，不使用对所有点都一样的固定半径
import { KDTree } from './kdtree.js';

export function estimateLocalScales(points, { probeK = 12 } = {}) {
  const tree = new KDTree(points);
  const n = points.length;
  const scales = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const kn = tree.knn(points[i], Math.min(probeK + 1, n), i);
    const ds = kn.map((r) => Math.sqrt(r.d2)).sort((a, b) => a - b);
    // 中位数作为该点的局部间距
    scales[i] = ds.length ? ds[ds.length >> 1] : 0;
  }
  // 用全局面板分位数做稳健裁剪，防止孤立点把局部尺度撑到无穷大
  const sorted = Array.from(scales).sort((a, b) => a - b);
  const q = (p) => sorted[Math.min(n - 1, Math.max(0, Math.floor(p * n)))];
  const lo = q(0.02), hi = q(0.98);
  for (let i = 0; i < n; i++) scales[i] = Math.min(hi, Math.max(lo, scales[i]));
  return { scales, tree, q02: lo, q50: q(0.5), q98: hi };
}
