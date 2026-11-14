// 噪点候选筛查（非破坏性）：只打标记，不删点；标记可在修订界面恢复
// 两类判据：
//  1) SOR（统计离群）：k 近邻平均距离的全局 z-score 超阈值
//  2) 孤立点：在 localScale * isolationFactor 半径内邻居数少于 minNeighbors
// 返回每个点的候选记录，保留全部量化指标，供人工裁决
import { KDTree } from './kdtree.js';

export function screenNoiseCandidates(points, scales, opts = {}) {
  const {
    sorK = 16,
    sorStdThreshold = 2.0,
    isolationFactor = 2.5,
    minNeighbors = 3,
  } = opts;
  const n = points.length;
  const tree = new KDTree(points);
  const meanDist = new Float64Array(n);
  const neighborCount = new Int32Array(n);

  for (let i = 0; i < n; i++) {
    const k = Math.min(sorK + 1, n);
    const kn = tree.knn(points[i], k, i);
    let sum = 0;
    for (const r of kn) sum += Math.sqrt(r.d2);
    meanDist[i] = kn.length ? sum / kn.length : 0;
    const rad = scales[i] * isolationFactor;
    neighborCount[i] = tree.radius(points[i], rad, i).length;
  }

  const mu = meanDist.reduce((a, b) => a + b, 0) / n;
  const sd = Math.sqrt(meanDist.reduce((a, b) => a + (b - mu) ** 2, 0) / n) || 1e-12;

  const candidates = [];
  for (let i = 0; i < n; i++) {
    const z = (meanDist[i] - mu) / sd;
    const sorHit = z > sorStdThreshold;
    const isoHit = neighborCount[i] < minNeighbors;
    if (sorHit || isoHit) {
      const reasons = [];
      if (sorHit) reasons.push(`SOR z=${z.toFixed(2)} > ${sorStdThreshold}`);
      if (isoHit) reasons.push(`孤立：${isolationFactor}×局部尺度内仅 ${neighborCount[i]} 个邻居 (<${minNeighbors})`);
      candidates.push({
        index: i,
        reasons,
        zScore: Number(z.toFixed(3)),
        meanKDist: Number(meanDist[i].toExponential(4)),
        neighborCount: neighborCount[i],
        localScale: Number(scales[i].toExponential(4)),
      });
    }
  }
  return { candidates, stats: { mu, sd, sorK, sorStdThreshold, isolationFactor, minNeighbors } };
}
