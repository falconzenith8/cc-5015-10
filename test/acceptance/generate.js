// 重建自动验收集：确定性生成解析曲面点云 fixture。
// fixture 含解析真值、未观测区域、注入扰动标签与“期望如何独立推导”的说明；
// 生成后写入 fixtures/ 并入库。
//
// 运行：node test/acceptance/generate.js [--check]
//   --check 只校验磁盘上的 fixture 与生成结果一致（CI / 入库一致性）
import { mkdirSync, writeFileSync, readFileSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeBoxRoom } from '../../src/core/analytic.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
export const FIXTURE_DIR = resolve(__dirname, 'fixtures');

function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
}
const jitter = (rand, amp) => (rand() - 0.5) * 2 * amp;
const round = (x, d = 5) => Number(x.toFixed(d));

// 盒式房间内表面；skip 缺面；hole 观测空洞；density 稀疏补丁
function sampleBoxRoom({ min, max, spacing, seed, noise = 0.002, skip = [], hole = null, density = null }) {
  const rand = rng(seed);
  const [x0, y0, z0] = min, [x1, y1, z1] = max;
  const sensor = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2];
  const faces = {
    xmin: { norm: [1, 0, 0], dims: [y1 - y0, z1 - z0], gen: (a, b, jn, jt) => [x0 + jn, y0 + a + jt(0), z0 + b + jt(1)] },
    xmax: { norm: [-1, 0, 0], dims: [y1 - y0, z1 - z0], gen: (a, b, jn, jt) => [x1 + jn, y0 + a + jt(0), z0 + b + jt(1)] },
    ymin: { norm: [0, 1, 0], dims: [x1 - x0, z1 - z0], gen: (a, b, jn, jt) => [x0 + a + jt(0), y0 + jn, z0 + b + jt(1)] },
    ymax: { norm: [0, -1, 0], dims: [x1 - x0, z1 - z0], gen: (a, b, jn, jt) => [x0 + a + jt(0), y1 + jn, z0 + b + jt(1)] },
    zmin: { norm: [0, 0, 1], dims: [x1 - x0, y1 - y0], gen: (a, b, jn, jt) => [x0 + a + jt(0), y0 + b + jt(1), z0 + jn] },
    zmax: { norm: [0, 0, -1], dims: [x1 - x0, y1 - y0], gen: (a, b, jn, jt) => [x0 + a + jt(0), y0 + b + jt(1), z1 + jn] },
  };
  const points = [], views = [], tags = [];
  for (const [id, f] of Object.entries(faces)) {
    if (skip.includes(id)) continue;
    const [da, db] = f.dims;
    const na = Math.max(2, Math.round(da / spacing));
    const nb = Math.max(2, Math.round(db / spacing));
    for (let ia = 0; ia < na; ia++) {
      for (let ib = 0; ib < nb; ib++) {
        const a = (ia + 0.5) * (da / na);
        const b = (ib + 0.5) * (db / nb);
        const p = f.gen(a, b, jitter(rand, noise), (k) => jitter(rand, spacing * 0.06));
        if (hole && hole.face === id && pointInBox(p, hole.min, hole.max)) continue;
        let tag = id;
        if (density && density.face === id && pointInBox(p, density.min, density.max)) {
          if (rand() > density.keep) continue;
          tag = 'sparse-patch';
        }
        points.push(p); views.push(sensor.slice()); tags.push(tag);
      }
    }
  }
  return { points, views, tags, sensor };
}

function pointInBox(p, min, max, eps = 1e-9) {
  return p[0] >= min[0] - eps && p[0] <= max[0] + eps
    && p[1] >= min[1] - eps && p[1] <= max[1] + eps
    && p[2] >= min[2] - eps && p[2] <= max[2] + eps;
}

// 球面 Fibonacci 分布；capZ 以上为观测空洞；传感器在球心
function sampleSphere({ radius, count, seed, noise = 0.002, capZ = null }) {
  const rand = rng(seed);
  const points = [], views = [], tags = [];
  const golden = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < count; i++) {
    const z = 1 - (2 * (i + 0.5)) / count;
    if (capZ != null && z > capZ) continue;
    const r = Math.sqrt(Math.max(0, 1 - z * z));
    const th = golden * i;
    const rr = radius + jitter(rand, noise);
    points.push([rr * r * Math.cos(th), rr * r * Math.sin(th), rr * z]);
    views.push([0, 0, 0]);
    tags.push('sphere');
  }
  return { points, views, tags };
}

// 圆柱侧壁；wedge 去掉一段方位角；传感器在轴心
function sampleCylinder({ radius, zMin, zMax, countTheta, countZ, seed, noise = 0.002, wedge = null }) {
  const rand = rng(seed);
  const points = [], views = [], tags = [];
  for (let i = 0; i < countTheta; i++) {
    for (let j = 0; j < countZ; j++) {
      const th = (2 * Math.PI * (i + 0.5)) / countTheta;
      if (wedge) {
        const deg = (th * 180) / Math.PI;
        const inside = wedge.fromDeg <= wedge.toDeg
          ? deg >= wedge.fromDeg && deg <= wedge.toDeg
          : deg >= wedge.fromDeg || deg <= wedge.toDeg;
        if (inside) continue;
      }
      const rr = radius + jitter(rand, noise);
      const z = zMin + ((j + 0.5) / countZ) * (zMax - zMin) + jitter(rand, noise * 0.5);
      points.push([rr * Math.cos(th), rr * Math.sin(th), z]);
      views.push([0, 0, (zMin + zMax) / 2]);
      tags.push('cylinder');
    }
  }
  return { points, views, tags };
}

// 平面 x=0；传感器在 sensorX 侧
function samplePlane({ half, spacing, seed, noise = 0.001, sensorX = 2 }) {
  const rand = rng(seed);
  const points = [], views = [], tags = [];
  const nu = Math.round((2 * half) / spacing), nv = Math.round((2 * half) / spacing);
  for (let i = 0; i < nu; i++) {
    for (let j = 0; j < nv; j++) {
      points.push([
        jitter(rand, noise),
        -half + (i + 0.5) * ((2 * half) / nu) + jitter(rand, spacing * 0.04),
        -half + (j + 0.5) * ((2 * half) / nv) + jitter(rand, spacing * 0.04),
      ]);
      views.push([sensorX, 0, 0]);
      tags.push('plane');
    }
  }
  return { points, views, tags };
}

// 注入离群点：室内悬浮点（有观测）+ 远距离群点（无观测）
function addOutliers({ points, views, tags }, { seed, interior = 16, far = 6, sensor = [0, 0, 0], span = 1.0 }) {
  const rand = rng(seed);
  for (let i = 0; i < interior; i++) {
    points.push([(rand() - 0.5) * span, (rand() - 0.5) * span * 0.8, (rand() - 0.5) * span * 0.8]);
    views.push(sensor.slice());
    tags.push('outlier-interior');
  }
  for (let i = 0; i < far; i++) {
    points.push([8 + rand() * 3, rand() * 4, -7 - rand() * 3]);
    views.push(null);
    tags.push('outlier-far');
  }
  return { points, views, tags };
}

function finalize({ points, views, tags }, meta) {
  // 确定性排序（坐标字典序），保证同一生成器在任何机器上产出同一哈希
  const idx = points.map((_, i) => i);
  idx.sort((a, b) => (points[a][0] - points[b][0]) || (points[a][1] - points[b][1]) || (points[a][2] - points[b][2]));
  const P = idx.map((i) => points[i].map((x) => round(x)));
  const V = idx.map((i) => (views[i] ? views[i].map((x) => round(x)) : null));
  const T = idx.map((i) => tags[i]);
  const hash = createHash('sha256').update(JSON.stringify(P)).digest('hex').slice(0, 16);
  return { schema: 'indoor-pc-acceptance/v1', ...meta, pointCount: P.length, hash, points: P, views: V, tags: T };
}

const box = { min: [-1.2, -0.9, -1.0], max: [1.2, 0.9, 1.0] };
const boxCenter = [(box.min[0] + box.max[0]) / 2, (box.min[1] + box.max[1]) / 2, (box.min[2] + box.max[2]) / 2];

function boxFaces(min, max, skip = []) {
  return makeBoxRoom({ min, max, skip }).faces;
}

export function buildFixtures() {
  const out = [];

  // 1) 盒式房间 + 观测孔洞 + 离群点：孔洞保持开放，干净点云表面距离受检
  {
    const hole = { face: 'zmax', min: [-0.45, -0.35, 0.99], max: [0.45, 0.35, 1.01] };
    const s = sampleBoxRoom({ ...box, spacing: 0.135, seed: 1101, noise: 0.002, hole });
    const withOutliers = addOutliers(s, { seed: 1102, interior: 14, far: 6, sensor: s.sensor, span: 0.9 });
    out.push(finalize(withOutliers, {
      name: 'box_glass_hole',
      description: '盒式房间 zmax 墙中央 0.9×0.7m 观测空洞（玻璃无回波），另注入 14 个室内悬浮点与 6 个远距离离群点',
      analytic: { kind: 'boxRoom', faces: boxFaces(box.min, box.max) },
      unobserved: [{
        id: 'glass-hole',
        kind: 'box',
        min: [hole.min[0], hole.min[1], 1.0],
        max: [hole.max[0], hole.max[1], 1.02],
      }],
      orientation: { kind: 'towardPoint', point: boxCenter },
      metadata: {
        noiseAmplitude: 0.002,
        spacingTarget: 0.135,
        methods: ['planar', 'bpa'],
        cleanExcludeTags: ['outlier-interior', 'outlier-far'],
        outlierTags: ['outlier-interior', 'outlier-far'],
        derivation: '点间距由 fixture 点集的最近邻中位数独立测量；有效区距离容差取 max(4×注入噪声, 0.5×点间距)。孔洞区域由构造坐标直接给出；错误封闭按“面片落在孔内的面积份额 > 50%”判定（洞缘合法窄三角必然跨边但占比很小），绝不采用重建报告中的自述值。',
      },
      topology: {
        planar: { nonManifoldMax: 0, requireBoundaryEdges: true },
        bpa: { nonManifoldMax: 0, componentCountEquals: 1, requireBoundaryEdges: true },
      },
    }));
  }

  // 2) 盒式房间 + 密度骤变：稀疏核不得被无观测支撑面片跨越
  {
    const sparse = { face: 'zmax', min: [-0.5, -0.3, 0.99], max: [0.5, 0.3, 1.01] };
    const s = sampleBoxRoom({ ...box, spacing: 0.135, seed: 1201, noise: 0.002, density: { ...sparse, keep: 0.12 } });
    out.push(finalize(s, {
      name: 'box_density_jump',
      description: '盒式房间 zmax 墙中央 1.0×0.6m 区域仅保留 12% 采样（密度骤变），无观测孔洞',
      analytic: { kind: 'boxRoom', faces: boxFaces(box.min, box.max) },
      unobserved: [],
      sparseRegion: { id: 'sparse-core', kind: 'box', min: [-0.42, -0.22, 0.98], max: [0.42, 0.22, 1.02] },
      orientation: { kind: 'towardPoint', point: boxCenter },
      metadata: {
        noiseAmplitude: 0.002,
        spacingTarget: 0.135,
        methods: ['planar', 'bpa'],
        cleanExcludeTags: [],
        outlierTags: [],
        derivation: '点间距由最近邻中位数独立测量；稀疏核由构造坐标给出。面片重心在核内且三个顶点均在核外 -> 判为无观测支撑的跨越面（反例应由该项捕获）。',
      },
      topology: {
        planar: { nonManifoldMax: 0 },
        bpa: { nonManifoldMax: 0, componentCountEquals: 1 },
      },
    }));
  }

  // 3) 球面 + 顶部观测空洞 + 离群点（解析曲面，BPA 适用）
  {
    const s = sampleSphere({ radius: 1, count: 700, seed: 1301, noise: 0.002, capZ: 0.55 });
    const withOutliers = addOutliers(s, { seed: 1302, interior: 12, far: 6, sensor: [0, 0, 0], span: 0.8 });
    out.push(finalize(withOutliers, {
      name: 'sphere_cap_hole',
      description: 'R=1 球面（Fibonacci 采样）顶部 z>0.55 无观测，注入 12 个室内悬浮点与 6 个远距离离群点',
      analytic: { kind: 'sphere', center: [0, 0, 0], radius: 1 },
      unobserved: [{ id: 'cap-hole', kind: 'sphereCap', center: [0, 0, 0], radiusMax: 1.02, zMin: 0.56 }],
      orientation: { kind: 'towardPoint', point: [0, 0, 0] },
      metadata: {
        noiseAmplitude: 0.002,
        spacingTarget: null,
        methods: ['bpa'],
        cleanExcludeTags: ['outlier-interior', 'outlier-far'],
        outlierTags: ['outlier-interior', 'outlier-far'],
        derivation: '球面真值由解析半径给出；点间距由 fixture 最近邻中位数独立测量。顶部空洞由构造 z 阈值给出；错误封闭按面片落在孔内的面积份额 > 50% 判定。',
      },
      topology: { bpa: { nonManifoldMax: 0, componentCountEquals: 1, requireBoundaryEdges: true } },
    }));
  }

  // 4) 圆柱侧壁 + 方位角楔形空洞（第二个解析曲面）
  {
    const s = sampleCylinder({ radius: 0.8, zMin: -0.8, zMax: 0.8, countTheta: 48, countZ: 13, seed: 1401, noise: 0.002, wedge: { fromDeg: 35, toDeg: 90 } });
    out.push(finalize(s, {
      name: 'cylinder_wedge_hole',
      description: 'R=0.8 圆柱侧壁，方位角 35°–90° 楔形区域无观测（遮挡）',
      analytic: { kind: 'cylinderLateral', center: [0, 0], radius: 0.8, zMin: -0.8, zMax: 0.8 },
      unobserved: [{
        id: 'wedge-hole',
        kind: 'cylindricalWedge',
        center: [0, 0],
        rMin: 0.72,
        rMax: 0.88,
        zMin: -0.82,
        zMax: 0.82,
        angleRange: [(35 * Math.PI) / 180, (90 * Math.PI) / 180],
      }],
      orientation: { kind: 'towardPoint', point: [0, 0, 0] },
      metadata: {
        noiseAmplitude: 0.002,
        spacingTarget: null,
        methods: ['bpa'],
        cleanExcludeTags: [],
        outlierTags: [],
        derivation: '圆柱侧壁真值由解析半径与轴向范围给出；楔形空洞由构造方位角区间给出。',
      },
      topology: { bpa: { nonManifoldMax: 0, componentCountEquals: 1, requireBoundaryEdges: true } },
    }));
  }

  // 5) 法线极性正例 / 反例：同一平面从 +X 与 -X 观测
  for (const variant of ['ok', 'flipped']) {
    const sensorX = variant === 'ok' ? 2 : -2;
    const s = samplePlane({ half: 0.8, spacing: 0.11, seed: 1501, noise: 0.001, sensorX });
    out.push(finalize(s, {
      name: `plane_normals_${variant}`,
      description: variant === 'ok'
        ? '平面从 +X 侧观测：法线应朝向传感器（+X），验证正例定向一致'
        : '同一平面改从 -X 侧观测（故意改变法线方向）：定向一致率应显著下降，退化必须被捕获',
      analytic: { kind: 'plane', point: [0, 0, 0], normal: [1, 0, 0] },
      unobserved: [],
      orientation: { kind: 'normal', normal: [1, 0, 0] },
      metadata: {
        noiseAmplitude: 0.001,
        spacingTarget: 0.11,
        methods: ['bpa'],
        cleanExcludeTags: [],
        outlierTags: [],
        expectationMode: variant === 'ok' ? 'normal' : 'expect-degradation',
        derivation: '平面真值法线由定义直接给出。正例要求面片绕序与 +X 同向占比高；反例（传感器换侧）要求同向占比显著下降——这是对“故意改变法线方向”的显式捕获，失败面片索引会被保留以便复跑。',
      },
      topology: { bpa: { nonManifoldMax: 0 } },
    }));
  }

  return out;
}

export function fixturePath(name) { return resolve(FIXTURE_DIR, `${name}.json`); }

function main() {
  const check = process.argv.includes('--check');
  mkdirSync(FIXTURE_DIR, { recursive: true });
  const fixtures = buildFixtures();
  let mismatch = 0;
  for (const f of fixtures) {
    const path = fixturePath(f.name);
    const text = JSON.stringify(f, null, 0);
    if (check) {
      if (!existsSync(path)) { console.error(`缺少 fixture: ${path}`); mismatch++; continue; }
      if (readFileSync(path, 'utf8') !== text) { console.error(`fixture 与生成器不一致: ${path}`); mismatch++; }
      else console.log(`ok ${f.name} (${f.pointCount} 点, hash=${f.hash})`);
    } else {
      writeFileSync(path, text);
      console.log(`写出 ${path} (${f.pointCount} 点, hash=${f.hash})`);
    }
  }
  if (check && mismatch) process.exit(1);
}

if (process.argv[1] === fileURLToPath(import.meta.url)) main();
