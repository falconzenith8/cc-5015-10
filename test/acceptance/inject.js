// 受控错误注入：对 fixture 点云做确定性破坏，供验收集验证"坏结果会被暴露"。
// 每种注入都返回 {points, views, description}，且必须能重复生成。
import { createHash } from 'node:crypto';

function rng(seed) {
  let s = seed >>> 0;
  return () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
}

// 1) 反转全部观测方向（把传感器放到点的另一侧）——故意改变法线方向
export function injectFlippedViews(fixture) {
  const views = fixture.points.map((p, i) => {
    const v = fixture.views[i];
    if (!v) return null;
    return [2 * p[0] - v[0], 2 * p[1] - v[1], 2 * p[2] - v[2]];
  });
  return {
    points: fixture.points,
    views,
    description: '把每个点的传感器位置镜像到点的另一侧：法线朝向被整体反转',
    hash: createHash('sha256').update(JSON.stringify(views)).digest('hex').slice(0, 16),
  };
}

// 2) 直接去掉观测（views 全 null）：法线朝向不可唯一确定，必须被标记歧义
export function injectNoViews(fixture) {
  const views = fixture.points.map(() => null);
  return {
    points: fixture.points,
    views,
    description: '移除全部观测方向：法线朝向不可唯一确定，应全部进入歧义标记',
    hash: createHash('sha256').update(JSON.stringify(views)).digest('hex').slice(0, 16),
  };
}

// 3) 把所有点沿 +X 平移 delta：表面应整体偏移 delta，网格->解析距离应显式变大
export function injectShift(fixture, delta = 0.15) {
  const points = fixture.points.map((p) => [p[0] + delta, p[1], p[2]]);
  return {
    points,
    views: fixture.views,
    description: `整体沿 +X 平移 ${delta}m：网格到解析真值的距离应整体增大到约 ${delta}m`,
    hash: createHash('sha256').update(JSON.stringify(points)).digest('hex').slice(0, 16),
  };
}

// 4) 在未观测区域内部注入密集点（伪造回波）：错误封闭检测必须命中
export function injectUnobservedFill(fixture, { spacing = 0.08, seed = 9001 } = {}) {
  const region = fixture.unobserved[0];
  if (!region) throw new Error('fixture 无未观测区域，无法注入伪造回波');
  const rand = rng(seed);
  const pts = regionFillPoints(region, spacing, rand);
  const points = [...fixture.points, ...pts];
  const sensor = fixture.orientation?.point || [0, 0, 0];
  const views = [...fixture.views, ...pts.map(() => sensor.slice())];
  const tags = [...fixture.tags, ...pts.map(() => 'injected-unobserved-fill')];
  return {
    points,
    views,
    tags,
    injected: pts,
    description: `在未观测区域 ${region.id} 内注入 ${pts.length} 个伪造回波点：错误封闭检测必须命中`,
    hash: createHash('sha256').update(JSON.stringify(points)).digest('hex').slice(0, 16),
  };
}

function regionFillPoints(region, spacing, rand) {
  const out = [];
  if (region.kind === 'box') {
    const [mn, mx] = [region.min, region.max];
    const na = Math.max(1, Math.round((mx[0] - mn[0]) / spacing));
    const nb = Math.max(1, Math.round((mx[1] - mn[1]) / spacing));
    const nc = Math.max(1, Math.round((mx[2] - mn[2]) / spacing));
    for (let a = 0; a < na; a++) for (let b = 0; b < nb; b++) for (let c = 0; c < nc; c++) {
      out.push([
        Number((mn[0] + (a + 0.5) * ((mx[0] - mn[0]) / na) + (rand() - 0.5) * 0.004).toFixed(5)),
        Number((mn[1] + (b + 0.5) * ((mx[1] - mn[1]) / nb) + (rand() - 0.5) * 0.004).toFixed(5)),
        Number((mn[2] + (c + 0.5) * ((mx[2] - mn[2]) / nc) + (rand() - 0.5) * 0.004).toFixed(5)),
      ]);
    }
    return out;
  }
  if (region.kind === 'sphereCap') {
    // 在 z>=zMin 的球冠上铺一层点（半径取 radiusMax*0.99，避免完全落在判定边界）
    const R = region.radiusMax * 0.995;
    const count = Math.max(16, Math.round((4 * Math.PI * R * R * 0.4) / (spacing * spacing)));
    const golden = Math.PI * (3 - Math.sqrt(5));
    for (let i = 0; i < count; i++) {
      const z = region.zMin + ((R - region.zMin) * (i + 0.5)) / count;
      const r = Math.sqrt(Math.max(0, R * R - z * z));
      const th = golden * i;
      out.push([
        Number((r * Math.cos(th)).toFixed(5)), Number((r * Math.sin(th)).toFixed(5)), Number(z.toFixed(5)),
      ]);
    }
    return out;
  }
  if (region.kind === 'cylindricalWedge') {
    const [a0, a1] = region.angleRange;
    const na = Math.max(2, Math.round(((a1 - a0) * region.rMax) / spacing));
    const nz = Math.max(2, Math.round((region.zMax - region.zMin) / spacing));
    for (let a = 0; a < na; a++) for (let b = 0; b < nz; b++) {
      const th = a0 + ((a1 - a0) * (a + 0.5)) / na;
      const R = (region.rMin + region.rMax) / 2;
      out.push([
        Number((R * Math.cos(th)).toFixed(5)), Number((R * Math.sin(th)).toFixed(5)),
        Number((region.zMin + ((b + 0.5) * (region.zMax - region.zMin)) / nz).toFixed(5)),
      ]);
    }
    return out;
  }
  throw new Error(`未知未观测区域类型: ${region.kind}`);
}

// 5) 抽掉一半采样（人为制造稀疏）：有效区距离应上升、边界应上升
export function injectThinned(fixture, keep = 0.5) {
  const rand = rng(7001);
  const keepMask = fixture.points.map(() => rand() < keep);
  // 保证至少保留 4 个点，避免退化成 0
  if (keepMask.filter(Boolean).length < 4) return { points: fixture.points, views: fixture.views, description: '抽稀失败（保留不足）', hash: 'noop' };
  const points = fixture.points.filter((_, i) => keepMask[i]);
  const views = fixture.views.filter((_, i) => keepMask[i]);
  const tags = fixture.tags.filter((_, i) => keepMask[i]);
  return {
    points, views, tags,
    description: `随机保留 ${Math.round(keep * 100)}% 采样：表面覆盖应下降、边界边应增加`,
    hash: createHash('sha256').update(JSON.stringify(points)).digest('hex').slice(0, 16),
  };
}
