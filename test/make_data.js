// 合成房间点集生成：用于验证的四个标准场景
// 输出 JSON：{ name, scenario, points, views(传感器位置), meta }
// views 为每个点对应的观测传感器位置（法线定向用 n·(view-p)>0）
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const __dirname = dirname(fileURLToPath(import.meta.url));
const OUT = resolve(__dirname, '../data');

// 伪随机（固定种子保证可复现）
function rng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

function jitter(rand, amp) { return (rand() - 0.5) * 2 * amp; }

// 房间内表面的六个面（w,d,h 为完整尺寸）；skipFace 可缺一面墙
function roomShell({ w = 4, d = 3, h = 2.6, spacing = 0.12, jitterAmp = 0.004, seed = 1, skipFace = null, densityPatch = null, glassHole = null }) {
  const rand = rng(seed);
  const points = [], views = [], tags = [];
  const center = [0, 0, 0];
  // 传感器：房间中心偏上
  const sensor = [0, h * 0.55, 0];

  const faces = {
    floor: { n: [0, 1, 0], gen: () => [jitter(rand, jitterAmp), -h / 2, jitter(rand, jitterAmp)], ranges: [w, d] },
    ceiling: { n: [0, -1, 0], gen: () => [jitter(rand, jitterAmp), h / 2, jitter(rand, jitterAmp)], ranges: [w, d] },
    wallXp: { n: [-1, 0, 0], gen: () => [w / 2, jitter(rand, jitterAmp), jitter(rand, jitterAmp)], ranges: [d, h] },
    wallXm: { n: [1, 0, 0], gen: () => [-w / 2, jitter(rand, jitterAmp), jitter(rand, jitterAmp)], ranges: [d, h] },
    wallZp: { n: [0, 0, -1], gen: () => [jitter(rand, jitterAmp), jitter(rand, jitterAmp), d / 2], ranges: [w, h] },
    wallZm: { n: [0, 0, 1], gen: () => [jitter(rand, jitterAmp), jitter(rand, jitterAmp), -d / 2], ranges: [w, h] },
  };

  for (const [faceName, face] of Object.entries(faces)) {
    if (faceName === skipFace) continue;
    const [ra, rb] = face.ranges;
    const na = Math.ceil(ra / spacing), nb = Math.ceil(rb / spacing);
    for (let ia = 0; ia < na; ia++) {
      for (let ib = 0; ib < nb; ib++) {
        let p = face.gen();
        // 按面把网格坐标放回去（gen 只产生了抖动，需要规则坐标）
        p = placeOnFace(faceName, ia, ib, na, nb, w, d, h, rand, jitterAmp);
        // 玻璃缺测：矩形区域内直接跳过（模拟无回波）
        if (glassHole && faceName === glassHole.face && insideRect(p, glassHole)) continue;
        // 密度骤变：补丁区域按降采样比例稀疏化
        if (densityPatch && faceName === densityPatch.face && insideRect(p, densityPatch) && rand() > densityPatch.keep) continue;
        points.push(p);
        views.push(sensor.slice());
        tags.push(faceName);
      }
    }
  }
  return { points, views, tags, sensor };
}

function placeOnFace(faceName, ia, ib, na, nb, w, d, h, rand, amp) {
  // 传感器噪声：主要沿切平面抖动，仅极小的法线向深度噪声（模拟真实测距）
  const tj = () => jitter(rand, amp);
  const nj = () => jitter(rand, amp * 0.15);
  switch (faceName) {
    case 'floor': return [-w / 2 + (ia + 0.5) * (w / na) + tj(), -h / 2 + nj(), -d / 2 + (ib + 0.5) * (d / nb) + tj()];
    case 'ceiling': return [-w / 2 + (ia + 0.5) * (w / na) + tj(), h / 2 + nj(), -d / 2 + (ib + 0.5) * (d / nb) + tj()];
    case 'wallXp': return [w / 2 + nj(), -h / 2 + (ib + 0.5) * (h / nb) + tj(), -d / 2 + (ia + 0.5) * (d / na) + tj()];
    case 'wallXm': return [-w / 2 + nj(), -h / 2 + (ib + 0.5) * (h / nb) + tj(), -d / 2 + (ia + 0.5) * (d / na) + tj()];
    case 'wallZp': return [-w / 2 + (ia + 0.5) * (w / na) + tj(), -h / 2 + (ib + 0.5) * (h / nb) + tj(), d / 2 + nj()];
    case 'wallZm': return [-w / 2 + (ia + 0.5) * (w / na) + tj(), -h / 2 + (ib + 0.5) * (h / nb) + tj(), -d / 2 + nj()];
  }
}

function insideRect(p, rect) {
  return p.every((_, k) => p[k] >= rect.min[k] - 1e-9 && p[k] <= rect.max[k] + 1e-9);
}

// 场景 1：已知平面房间（完整六面，规则密度）
function scenarioPlanarRoom() {
  const { points, views, tags } = roomShell({ seed: 11, spacing: 0.12 });
  return {
    name: 's1_planar_room',
    scenario: 'known-planar-room',
    description: '完整六面平面房间，规则点密度，含观测位置',
    points, views, tags,
    expectations: '两个后端都应重建出 6 个平面区域；退化面≈0；点到网格距离中位数接近采样间距量级。',
  };
}

// 场景 2：密度骤变（一面墙局部稀疏 10 倍）
function scenarioDensityJump() {
  const { points, views, tags } = roomShell({
    seed: 22, spacing: 0.12,
    densityPatch: { face: 'wallZp', min: [-0.6, -0.2, 1.49], max: [0.6, 1.2, 1.51], keep: 0.1 },
  });
  return {
    name: 's2_density_jump',
    scenario: 'density-jump',
    description: '墙 wallZp 中央 1.2×1.4m 区域采样密度骤降 90%',
    points, views, tags,
    expectations: '局部尺度在补丁处显著增大；重建不得在稀疏区生成跨度过大的面片，边界边数量应上升，空洞保留。',
  };
}

// 场景 3：孤立噪点（完整房间 + 空间离群点）
function scenarioIsolatedNoise() {
  const base = roomShell({ seed: 33, spacing: 0.14 });
  const rand = rng(99);
  const noisePts = [], noiseViews = [], noiseTags = [];
  // 房间内部悬浮噪点 + 远离房间的离群点
  for (let i = 0; i < 25; i++) {
    const p = [(rand() - 0.5) * 2, (rand() - 0.5) * 1.5, (rand() - 0.5) * 1.5];
    noisePts.push(p); noiseViews.push(base.sensor.slice()); noiseTags.push('noise-interior');
  }
  for (let i = 0; i < 8; i++) {
    const p = [4 + rand() * 3, rand() * 4, -3 - rand() * 2];
    noisePts.push(p); noiseViews.push(null); noiseTags.push('noise-outlier');
  }
  return {
    name: 's3_isolated_noise',
    scenario: 'isolated-noise',
    description: '完整房间叠加 25 个室内悬浮点与 8 个远距离群点（离群点无观测方向）',
    points: [...base.points, ...noisePts],
    views: [...base.views, ...noiseViews],
    tags: [...base.tags, ...noiseTags],
    expectations: '噪点筛查应命中全部远距离群点与多数悬浮点；候选可恢复；剔除后重建不受离群点搭桥影响。',
  };
}

// 场景 4：缺一面墙（遮挡空洞，禁止补墙）
function scenarioMissingWall() {
  const { points, views, tags } = roomShell({ seed: 44, spacing: 0.12, skipFace: 'wallXp' });
  return {
    name: 's4_missing_wall',
    scenario: 'missing-wall',
    description: '完整房间移除 +X 整面墙，模拟未观察区域',
    points, views, tags,
    expectations: '重建结果必须在 +X 侧保持开放：边界边明显、无任何跨越开口的面片、连通区域不被封口。',
  };
}

// 附加场景 5：玻璃缺测（墙面上矩形无回波）
function scenarioGlassHole() {
  const { points, views, tags } = roomShell({
    seed: 55, spacing: 0.12,
    glassHole: { face: 'wallZm', min: [-0.7, 0.2, -1.51], max: [0.7, 1.6, -1.49] },
  });
  return {
    name: 's5_glass_hole',
    scenario: 'glass-missing-returns',
    description: 'wallZm 墙面中央 1.4×1.4m 矩形无回波（玻璃），其余正常',
    points, views, tags,
    expectations: '矩形区域作为真洞保留：平面后端空栅格数>0，BPA 边界边沿矩形分布；绝不可自动封洞。',
  };
}

const scenarios = {
  s1: scenarioPlanarRoom(),
  s2: scenarioDensityJump(),
  s3: scenarioIsolatedNoise(),
  s4: scenarioMissingWall(),
  s5: scenarioGlassHole(),
};

mkdirSync(OUT, { recursive: true });
for (const [key, data] of Object.entries(scenarios)) {
  const path = `${OUT}/${data.name}.json`;
  writeFileSync(path, JSON.stringify(data, null, 1));
  console.log(`写出 ${path}  点数=${data.points.length}`);
}
