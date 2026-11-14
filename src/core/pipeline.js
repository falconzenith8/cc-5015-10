// 重建流水线编排：尺度 -> 噪点筛查 -> 法线 -> 重建 -> 质检
// 每次运行冻结参数快照与输入修订版本；失败必须带阶段与定位，不允许静默
import { estimateLocalScales } from './stats.js';
import { screenNoiseCandidates } from './noise.js';
import { estimateNormals } from './normals.js';
import { reconstructBPA } from './reconstruct/bpa.js';
import { reconstructPlanar } from './reconstruct/planar.js';
import { checkMesh } from './meshqc.js';
import { defaultParams } from './schema.js';

export function runPipeline(cloud, rawParams = {}) {
  const params = { ...defaultParams(), ...rawParams };
  const startedAt = new Date().toISOString();
  const stageTimings = {};
  const step = (name, fn) => {
    const t0 = performance.now();
    try {
      const r = fn();
      stageTimings[name] = Number((performance.now() - t0).toFixed(1));
      return r;
    } catch (err) {
      const failure = {
        failedStage: name,
        message: err.message,
        location: err.location || null,
        hint: stageHint(name),
      };
      throw new PipelineError(failure, stageTimings);
    }
  };

  // 1) 局部尺度（逐点不同，绝不使用全局固定邻域）
  const scaleInfo = step('local-scale', () => estimateLocalScales(cloud.points));

  // 2) 噪点候选筛查（只标记不删除）
  const noise = step('noise-screen', () =>
    screenNoiseCandidates(cloud.points, scaleInfo.scales, {
      sorK: params.sorK,
      sorStdThreshold: params.sorStdThreshold,
      isolationFactor: params.isolationFactor,
      minNeighbors: params.minNeighbors,
    }));

  // 3) 法线估计（观测方向定向 + 歧义标记）
  const normalResult = step('normal-estimation', () =>
    estimateNormals(cloud.points, scaleInfo.scales, cloud.views, {
      neighborhoodFactor: params.neighborhoodFactor,
      minNeighbors: params.minNeighborsN,
      orientationConsensus: params.orientationConsensus,
    }));

  // 4) 重建
  const recon = step('surface-reconstruction', () => {
    if (params.method === 'bpa') {
      const r = reconstructBPA(cloud.points, normalResult.normals, scaleInfo.scales, {
        radiusFactors: params.radiusFactors,
        angleThresholdDeg: params.angleThresholdDeg,
        maxEdgeFactor: params.maxEdgeFactor,
      });
      return toUnifiedMesh(cloud, r, normalResult.normals);
    }
    if (params.method === 'planar') {
      const r = reconstructPlanar(cloud.points, normalResult.normals, scaleInfo.scales, normalResult.planarity, {
        angleThresholdDeg: params.planarAngleDeg,
        distanceFactor: params.planarDistanceFactor,
        gridFactor: params.gridFactor,
        minRegionSize: params.minRegionSize,
      });
      return { ...r, vertexNormals: r.vertices.map(() => null) };
    }
    const err = new Error(`未知重建方法: ${params.method}`);
    err.location = { param: 'method' };
    throw err;
  });

  if (!recon.faces.length) {
    throw new PipelineError({
      failedStage: 'surface-reconstruction',
      message: '重建产生 0 个面片：当前参数下没有任何点能形成受支持的表面。',
      location: {
        unsupportedPreview: recon.unsupported.slice(0, 20),
        radiusLog: recon.radiusLog || null,
        regionCount: recon.regionReports?.length || 0,
      },
      hint: '调大球半径序列/平面距离阈值，或先在修订面板恢复被剔除的边界点；缺墙与空洞本身不会被自动填补。',
    }, stageTimings);
  }

  // 5) 质检
  const qc = step('mesh-qc', () => checkMesh(recon, cloud.points, { usedPoint: recon.usedPoint }));

  return {
    runId: `run_${Date.now().toString(36)}_${Math.random().toString(36).slice(2, 8)}`,
    startedAt,
    finishedAt: new Date().toISOString(),
    frozen: {
      cloudId: cloud.id,
      cloudHash: cloud.hash,
      cloudName: cloud.name,
      pointCount: cloud.n,
      paramsSnapshot: JSON.parse(JSON.stringify(params)),
    },
    scales: {
      perPoint: Array.from(scaleInfo.scales, (v) => Number(v.toExponential(5))),
      q02: Number(scaleInfo.q02.toExponential(5)),
      q50: Number(scaleInfo.q50.toExponential(5)),
      q98: Number(scaleInfo.q98.toExponential(5)),
    },
    noise,
    normals: {
      values: normalResult.normals.map((x) => x.map((v) => Number(v.toFixed(5)))),
      flags: Array.from(normalResult.flags),
      flagLabels: normalResult.flagLabels,
      planarity: normalResult.planarity,
    },
    mesh: {
      vertices: recon.vertices,
      faces: recon.faces,
      vertexNormals: recon.vertexNormals || null,
      method: recon.method,
      boundaryEdges: recon.boundaryEdges,
      boundaryEdgeCount: recon.boundaryEdgeCount,
      radiusLog: recon.radiusLog || null,
      regionReports: recon.regionReports || null,
      unsupported: recon.unsupported,
    },
    qc,
    stageTimings,
  };
}

// BPA 输出的是点索引面片，转换为独立顶点数组的统一网格
function toUnifiedMesh(cloud, r, normals) {
  return {
    vertices: cloud.points.map((p) => p.slice()),
    faces: r.faces,
    vertexNormals: normals.map((x) => x.slice()),
    method: r.method,
    usedPoint: r.usedPoint,
    unsupported: r.unsupported,
    boundaryEdges: r.boundaryEdges,
    boundaryEdgeCount: r.boundaryEdgeCount,
    radiusLog: r.radiusLog,
    regionReports: null,
  };
}

function stageHint(name) {
  const hints = {
    'local-scale': '检查点集是否含非数值坐标或重复点。',
    'noise-screen': '降低 SOR 阈值过激程度，确认点数量满足 kNN。',
    'normal-estimation': '调大邻域半径因子；平面性差的区域已在法线标记中列出。',
    'surface-reconstruction': '调整球半径/平面阈值；查看 unsupported 定位，不要期待自动封洞。',
    'mesh-qc': '网格规模异常，检查退化面列表。',
  };
  return hints[name] || '检查输入数据与参数。';
}

export class PipelineError extends Error {
  constructor(failure, timings) {
    super(failure.message);
    this.name = 'PipelineError';
    this.failure = failure;
    this.timings = timings;
  }
}
