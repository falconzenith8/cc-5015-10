// 重建参数 schema：前端控件由内核 schema 驱动，不允许出现内核不认识的参数
// 每个参数含 key、类型、默认值、范围、中文说明与所属阶段
export const PARAM_SCHEMA = [
  {
    stage: 'noise', title: '噪点候选筛查',
    params: [
      { key: 'sorK', type: 'int', default: 16, min: 3, max: 128, label: 'SOR 邻居数 k' },
      { key: 'sorStdThreshold', type: 'float', default: 2.0, min: 0.5, max: 6, step: 0.1, label: 'SOR z-score 阈值' },
      { key: 'isolationFactor', type: 'float', default: 2.5, min: 1, max: 10, step: 0.1, label: '孤立判定半径（×局部尺度）' },
      { key: 'minNeighbors', type: 'int', default: 3, min: 1, max: 32, label: '最少邻居数' },
    ],
  },
  {
    stage: 'normals', title: '法线估计（自适应邻域）',
    params: [
      { key: 'neighborhoodFactor', type: 'float', default: 2.0, min: 0.5, max: 8, step: 0.1, label: '邻域半径（×逐点局部尺度）' },
      { key: 'minNeighborsN', type: 'int', default: 4, min: 3, max: 32, label: '最少邻居（不足退化 kNN 并标记）' },
      { key: 'orientationConsensus', type: 'float', default: 0.6, min: 0.3, max: 1, step: 0.05, label: '定向一致率（低于则标歧义）' },
    ],
  },
  {
    stage: 'reconstruct', title: '表面重建',
    params: [
      { key: 'method', type: 'enum', default: 'bpa', options: ['bpa', 'planar'], label: '重建后端（BPA 球旋转 / 平面区域生长）' },
      { key: 'radiusFactors', type: 'floatList', default: [1.1, 1.6, 2.4], label: 'BPA 球半径序列（×中位局部尺度，按序递进；需略大于局部点间距）' },
      { key: 'angleThresholdDeg', type: 'float', default: 75, min: 10, max: 179, step: 1, label: 'BPA 法线兼容角（度）' },
      { key: 'maxEdgeFactor', type: 'float', default: 6.0, min: 1, max: 20, step: 0.5, label: '最大边长（×中位局部尺度），禁止跨空洞搭桥' },
      { key: 'planarAngleDeg', type: 'float', default: 12, min: 1, max: 45, step: 1, label: '平面生长法线阈值（度）' },
      { key: 'planarDistanceFactor', type: 'float', default: 1.5, min: 0.5, max: 6, step: 0.1, label: '平面生长距离阈值（×局部尺度）' },
      { key: 'gridFactor', type: 'float', default: 1.0, min: 0.25, max: 4, step: 0.25, label: '平面栅格边长（×区域局部尺度），空栅格不补面' },
      { key: 'minRegionSize', type: 'int', default: 8, min: 3, max: 200, label: '最小平面区域点数' },
    ],
  },
];

export function defaultParams() {
  const p = {};
  for (const group of PARAM_SCHEMA) for (const d of group.params) p[d.key] = JSON.parse(JSON.stringify(d.default));
  return p;
}
