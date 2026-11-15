// 解析曲面真值：与重建算法完全独立，仅由几何定义给出距离、法线与区域归属。
// 验收测试用这里的值作参照，绝不从重建输出反推期望。
import { dot, norm, sub } from './vec.js';

// 点到解析曲面的最近距离
export function pointToAnalyticDistance(analytic, p) {
  switch (analytic.kind) {
    case 'plane': {
      const n = normalizeSafe(analytic.normal);
      return Math.abs(dot(n, sub(p, analytic.point)));
    }
    case 'sphere': {
      const d = Math.hypot(p[0] - analytic.center[0], p[1] - analytic.center[1], p[2] - analytic.center[2]);
      return Math.abs(d - analytic.radius);
    }
    case 'cylinderLateral': {
      // 仅侧壁：轴向范围外的点按到对应端缘的距离计
      const rho = Math.hypot(p[0] - analytic.center[0], p[1] - analytic.center[1]);
      const dz = p[2] < analytic.zMin ? analytic.zMin - p[2] : p[2] > analytic.zMax ? p[2] - analytic.zMax : 0;
      return Math.hypot(rho - analytic.radius, dz);
    }
    case 'boxRoom': {
      let best = Infinity;
      for (const face of analytic.faces) {
        const d = pointToRectDistance(p, face.point, face.u, face.v, face.halfU, face.halfV);
        if (d < best) best = d;
      }
      return best;
    }
    default:
      throw new Error(`未知解析曲面类型: ${analytic.kind}`);
  }
}

// 解析曲面在该点处的期望朝向（指向观测侧/房间内部）。
// 重建面片的绕序应与该方向一致；用于“故意翻转法线”的退化捕获。
export function expectedNormalAt(analytic, p) {
  switch (analytic.kind) {
    case 'plane':
      return normalizeSafe(analytic.normal);
    case 'sphere': {
      // 球心处方向未定义：返回 null 让调用方跳过，而不是抛出/给零向量
      const d = sub(analytic.center, p);
      return norm(d) < 1e-9 ? null : normalizeSafe(d);
    }
    case 'cylinderLateral': {
      const r = [analytic.center[0] - p[0], analytic.center[1] - p[1], 0];
      return norm(r) < 1e-9 ? null : normalizeSafe(r);
    }
    case 'boxRoom': {
      const face = nearestBoxFace(analytic, p);
      return face ? normalizeSafe(face.normal) : null;
    }
    default:
      throw new Error(`未知解析曲面类型: ${analytic.kind}`);
  }
}

// 点归属于哪张解析面（盒式房间）；超出所有面矩形范围时返回 null
export function boxFaceAt(analytic, p, extentTol = 1e-9) {
  let best = null, bestD = Infinity;
  for (const face of analytic.faces) {
    const d = pointToRectDistance(p, face.point, face.u, face.v, face.halfU, face.halfV);
    if (d > bestD) continue;
    const dn = Math.abs(dot(face.normal, sub(p, face.point)));
    const du = Math.abs(dot(sub(p, face.point), face.u));
    const dv = Math.abs(dot(sub(p, face.point), face.v));
    if (du > face.halfU + extentTol || dv > face.halfV + extentTol) continue;
    if (dn < bestD) { bestD = dn; best = face; }
  }
  return best;
}

function nearestBoxFace(analytic, p) {
  let best = null, bestD = Infinity;
  for (const face of analytic.faces) {
    const d = Math.abs(dot(face.normal, sub(p, face.point)));
    const du = Math.abs(dot(sub(p, face.point), face.u));
    const dv = Math.abs(dot(sub(p, face.point), face.v));
    if (du <= face.halfU && dv <= face.halfV && d < bestD) { bestD = d; best = face; }
  }
  return best;
}

function normalizeSafe(v) {
  const n = norm(v);
  if (n < 1e-15) throw new Error('解析法线为零向量');
  return [v[0] / n, v[1] / n, v[2] / n];
}

// 点到矩形（中心 point、正交切向 u/v、半宽 halfU/halfV）的距离
function pointToRectDistance(p, center, u, v, halfU, halfV) {
  const d = sub(p, center);
  const du = Math.max(-halfU, Math.min(halfU, dot(d, u)));
  const dv = Math.max(-halfV, Math.min(halfV, dot(d, v)));
  const q = [
    center[0] + u[0] * du + v[0] * dv,
    center[1] + u[1] * du + v[1] * dv,
    center[2] + u[2] * du + v[2] * dv,
  ];
  return Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
}

// 未观测区域判定（原样，用于“错误封闭”的保守检出）
export function pointInRegion(region, p) {
  switch (region.kind) {
    case 'box': {
      const [mn, mx] = [region.min, region.max];
      return p[0] >= mn[0] && p[0] <= mx[0] && p[1] >= mn[1] && p[1] <= mx[1] && p[2] >= mn[2] && p[2] <= mx[2];
    }
    case 'sphereCap': {
      const d = Math.hypot(p[0] - region.center[0], p[1] - region.center[1], p[2] - region.center[2]);
      return p[2] >= region.zMin && d <= region.radiusMax;
    }
    case 'cylindricalWedge': {
      const dx = p[0] - region.center[0], dy = p[1] - region.center[1];
      const rho = Math.hypot(dx, dy);
      if (p[2] < region.zMin || p[2] > region.zMax) return false;
      if (rho < region.rMin || rho > region.rMax) return false;
      let ang = Math.atan2(dy, dx);
      if (ang < 0) ang += 2 * Math.PI;
      const [a0, a1] = region.angleRange;
      return a0 <= a1 ? ang >= a0 && ang <= a1 : ang >= a0 || ang <= a1;
    }
    default:
      throw new Error(`未知未观测区域类型: ${region.kind}`);
  }
}

// 膨胀未观测区域：用于“有效区”剔除孔洞边缘带，避免把合法洞缘面误判。
export function dilateRegion(region, margin) {
  switch (region.kind) {
    case 'box':
      return {
        ...region,
        min: [region.min[0] - margin, region.min[1] - margin, region.min[2] - margin],
        max: [region.max[0] + margin, region.max[1] + margin, region.max[2] + margin],
      };
    case 'sphereCap':
      return { ...region, zMin: region.zMin - margin, radiusMax: region.radiusMax + margin };
    case 'cylindricalWedge': {
      const rMid = (region.rMin + region.rMax) / 2;
      const da = margin / Math.max(rMid, 1e-9);
      return {
        ...region,
        rMin: region.rMin - margin,
        rMax: region.rMax + margin,
        zMin: region.zMin - margin,
        zMax: region.zMax + margin,
        angleRange: [region.angleRange[0] - da, region.angleRange[1] + da],
      };
    }
    default:
      throw new Error(`未知未观测区域类型: ${region.kind}`);
  }
}

// 构造盒式房间的六个矩形面（显式内向法线，指向房间内部/观测侧）
export function makeBoxRoom({ min, max, skip = [] }) {
  const [x0, y0, z0] = min;
  const [x1, y1, z1] = max;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, cz = (z0 + z1) / 2;
  const hx = (x1 - x0) / 2, hy = (y1 - y0) / 2, hz = (z1 - z0) / 2;
  const all = {
    xmin: { id: 'xmin', point: [x0, cy, cz], u: [0, 1, 0], v: [0, 0, 1], halfU: hy, halfV: hz, normal: [1, 0, 0] },
    xmax: { id: 'xmax', point: [x1, cy, cz], u: [0, 1, 0], v: [0, 0, 1], halfU: hy, halfV: hz, normal: [-1, 0, 0] },
    ymin: { id: 'ymin', point: [cx, y0, cz], u: [1, 0, 0], v: [0, 0, 1], halfU: hx, halfV: hz, normal: [0, 1, 0] },
    ymax: { id: 'ymax', point: [cx, y1, cz], u: [1, 0, 0], v: [0, 0, 1], halfU: hx, halfV: hz, normal: [0, -1, 0] },
    zmin: { id: 'zmin', point: [cx, cy, z0], u: [1, 0, 0], v: [0, 1, 0], halfU: hx, halfV: hy, normal: [0, 0, 1] },
    zmax: { id: 'zmax', point: [cx, cy, z1], u: [1, 0, 0], v: [0, 1, 0], halfU: hx, halfV: hy, normal: [0, 0, -1] },
  };
  return { kind: 'boxRoom', faces: Object.values(all).filter((f) => !skip.includes(f.id)) };
}

// 真值面“边界”距离：点到所属解析面矩形边界的平面内距离（非盒式曲面返回 Infinity 或端缘距离）。
// 验收的“有效区”= 离真值边界足够远且不在未观测区域内的区域；墙角/墙缘的混合法线伪面不算有效区。
export function distanceToSurfaceBorder(analytic, p) {
  switch (analytic.kind) {
    case 'boxRoom': {
      const face = boxFaceAt(analytic, p);
      if (!face) return 0;
      const du = Math.abs(dot(sub(p, face.point), face.u));
      const dv = Math.abs(dot(sub(p, face.point), face.v));
      return Math.min(face.halfU - du, face.halfV - dv);
    }
    case 'sphere':
      return Infinity; // 完整球面除孔洞外无外部边界；孔缘由未观测区域负责
    case 'cylinderLateral':
      return Math.min(p[2] - analytic.zMin, analytic.zMax - p[2]);
    case 'plane': {
      // 采样平面为 half×half 的正方形（构造约定），距离其边界
      if (!analytic.half) return Infinity;
      return Math.min(analytic.half - Math.abs(p[1]), analytic.half - Math.abs(p[2]));
    }
    default:
      throw new Error(`未知解析曲面类型: ${analytic.kind}`);
  }
}
