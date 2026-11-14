// 平面区域生长重建（面向室内平面房间）
// 诚实性原则：
//  - 只在"实际观测到点"的栅格内生成面片，空栅格一律留白——遮挡空洞/玻璃缺测/缺墙不会被补成墙面
//  - 区域之间不做强制缝合，间隙作为边界报告
//  - 无法归入任何平面区域的点列入 unsupported 并给出原因
import { KDTree } from '../kdtree.js';
import { cross, dot, normalize, sub, add, scale } from '../vec.js';
import { symEig3 } from '../eig.js';

const median = (arr) => { const s = Array.from(arr).sort((a, b) => a - b); return s[s.length >> 1]; };

export function reconstructPlanar(points, normals, scales, planarity, opts = {}) {
  const {
    angleThresholdDeg = 12,       // 法线一致阈值
    distanceFactor = 1.5,         // 到平面距离阈值 = 点局部尺度 × 该值
    minRegionSize = 8,
    gridFactor = 1.0,             // 栅格边长 = 区域点局部尺度中位数 × 该值
    mergeAngleDeg = 8,            // 共面区域合并：法线夹角阈值
    mergeDistanceFactor = 1.0,    // 共面区域合并：平面间距（×中位局部尺度）
  } = opts;

  const n = points.length;
  const tree = new KDTree(points);
  const cosThr = Math.cos((angleThresholdDeg * Math.PI) / 180);
  const globalMed = median(Array.from(scales));

  // 1) 区域生长：种子按平面性从好到差
  const regionOf = new Int32Array(n).fill(-1);
  const rawRegions = [];
  const seedOrder = [...Array(n).keys()].sort((a, b) => planarity[a] - planarity[b]);

  for (const seed of seedOrder) {
    if (regionOf[seed] !== -1) continue;
    const members = [seed];
    regionOf[seed] = rawRegions.length;
    const queue = [seed];
    while (queue.length) {
      const i = queue.shift();
      const rad = scales[i] * distanceFactor * 2;
      for (const q of tree.radius(points[i], rad)) {
        const j = q.i;
        if (regionOf[j] !== -1) continue;
        if (Math.abs(dot(normals[i], normals[j])) < cosThr) continue;
        const d = Math.abs(dot(sub(points[j], points[seed]), normals[seed]));
        if (d > scales[j] * distanceFactor * 2) continue;
        regionOf[j] = rawRegions.length;
        members.push(j);
        queue.push(j);
      }
    }
    if (members.length < minRegionSize) {
      for (const m of members) regionOf[m] = -1;
      continue;
    }
    rawRegions.push(members);
  }

  // 2) 每个初始区域拟合平面，再合并同属一张物理平面的碎片（同墙的不同子区域）
  const fitted = rawRegions.map((members) => ({ members, ...fitPlane(points, members, normals) }));
  const regions = mergeCoplanar(fitted, points, normals, mergeAngleDeg, globalMed * mergeDistanceFactor);

  // 3) 每个合并后区域做 2D 栅格三角化（只覆盖有点栅格，空栅格留白）
  const faces = [];
  const vertexMap = new Map();
  const vertices = [];
  const usedPoint = new Uint8Array(n);

  // 全局焊接键：量化到约 1mm（远小于采样间距 ~0.12m），同物理位置的理论角点跨区域共享顶点
  const vertexKey = (x, y, z) => `${Math.round(x * 1000)},${Math.round(y * 1000)},${Math.round(z * 1000)}`;
  const getVertex = (p) => {
    const key = vertexKey(p[0], p[1], p[2]);
    if (vertexMap.has(key)) return vertexMap.get(key);
    const id = vertices.push(p.slice()) - 1;
    vertexMap.set(key, id);
    return id;
  };

  const regionReports = [];

  for (let ri = 0; ri < regions.length; ri++) {
    const members = regions[ri].members;
    const { centroid, normal } = regions[ri];
    const ref = Math.abs(normal[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const u = normalize(cross(ref, normal));
    const v = normalize(cross(normal, u));
    const cell = median(members.map((i) => scales[i])) * gridFactor;

    const cells = new Map(); // "gx,gy" -> 代表点索引
    let minGx = Infinity, maxGx = -Infinity, minGy = Infinity, maxGy = -Infinity;
    for (const i of members) {
      const d = sub(points[i], centroid);
      const pu = dot(d, u), pv = dot(d, v);
      const gx = Math.floor(pu / cell), gy = Math.floor(pv / cell);
      const key = `${gx},${gy}`;
      if (!cells.has(key)) cells.set(key, i);
      minGx = Math.min(minGx, gx); maxGx = Math.max(maxGx, gx);
      minGy = Math.min(minGy, gy); maxGy = Math.max(maxGy, gy);
      usedPoint[i] = 1;
    }

    // 每个有数据栅格生成完全落在本栅格内的面片。
    // 角点须通过数据支撑检验：无支撑角点（墙角/开口/空栅格边缘）直接收缩到本栅格代表点，
    // 面片因此严格位于有数据一侧，绝不探入未观察区域半步，且不产生退化面。
    const memberSet = new Set(members);
    const repVertex = new Map();
    for (const key of cells.keys()) repVertex.set(key, getVertex(points[cells.get(key)]));
    // 按栅格坐标缓存理论角点顶点：相邻栅格共享同一顶点；vertexKey 已做位置焊接
    const cornerVertex = new Map();
    const supportedAt = (theoretical) => {
      // 探测半径取半栅格：角点仅当其紧邻（同属四周栅格内）有真实点才算支撑，
      // 避免洞缘角点借到一栅格之外的点而把面片探入未观察区域
      const probe = cell * 0.5;
      for (const q of tree.radius(theoretical, probe)) {
        if (memberSet.has(q.i)) return true;
      }
      return false;
    };
    const cornerPos = (gx, gy) => add(add(centroid, scale(u, gx * cell)), scale(v, gy * cell));
    const cornerV = (gx, gy) => {
      const k = `c:${gx},${gy}`;
      if (!cornerVertex.has(k)) cornerVertex.set(k, getVertex(cornerPos(gx, gy)));
      return cornerVertex.get(k);
    };

    let patchFaces = 0;
    for (const key of cells.keys()) {
      const [gx, gy] = key.split(',').map(Number);
      const rep = repVertex.get(key);
      const resolve = (cx, cy) => (supportedAt(cornerPos(cx, cy)) ? cornerV(cx, cy) : rep);
      const c00 = resolve(gx, gy);
      const c10 = resolve(gx + 1, gy);
      const c11 = resolve(gx + 1, gy + 1);
      const c01 = resolve(gx, gy + 1);

      // 去重顶点后按环序三角化：3 顶点 1 面，4 顶点 2 面
      const ring = [];
      for (const vid of [c00, c10, c11, c01]) if (!ring.includes(vid)) ring.push(vid);
      if (ring.length < 3) continue;
      const addOriented = (a, b, c) => {
        const faceN = normalize(cross(sub(vertices[b], vertices[a]), sub(vertices[c], vertices[a])));
        if (dot(faceN, normal) >= 0) faces.push([a, b, c]);
        else faces.push([a, c, b]);
      };
      if (ring.length === 3) {
        addOriented(ring[0], ring[1], ring[2]);
        patchFaces++;
      } else {
        addOriented(ring[0], ring[1], ring[2]);
        addOriented(ring[0], ring[2], ring[3]);
        patchFaces += 2;
      }
    }
    const totalCells = (maxGx - minGx + 1) * (maxGy - minGy + 1);
    const emptyCells = totalCells - cells.size;

    regionReports.push({
      regionIndex: ri,
      pointCount: members.length,
      normal: normal.map((x) => Number(x.toFixed(4))),
      centroid: centroid.map((x) => Number(x.toFixed(4))),
      gridCellsObserved: cells.size,
      gridCellsEmpty: emptyCells,
      faces: patchFaces,
    });
  }

  // 4) 压缩顶点：剔除未被任何面片引用的代表点顶点，并重映射面片索引
  const referenced = new Set();
  for (const [a, b, c] of faces) { referenced.add(a); referenced.add(b); referenced.add(c); }
  const remap = new Int32Array(vertices.length).fill(-1);
  const compactVerts = [];
  for (const vi of referenced) { remap[vi] = compactVerts.length; compactVerts.push(vertices[vi]); }
  const compactFaces = faces.map(([a, b, c]) => [remap[a], remap[b], remap[c]]);
  vertices.length = 0;
  vertices.push(...compactVerts);
  faces.length = 0;
  faces.push(...compactFaces);

  // 5) 边界边（只出现一次）
  const edgeCount = new Map();
  const ek = (a, b) => (a < b ? a * vertices.length + b : b * vertices.length + a);
  for (const [a, b, c] of faces) {
    for (const [x, y] of [[a, b], [b, c], [c, a]]) edgeCount.set(ek(x, y), (edgeCount.get(ek(x, y)) || 0) + 1);
  }
  const boundaryEdges = [];
  for (const [k, c] of edgeCount) if (c === 1) boundaryEdges.push([Math.floor(k / vertices.length), k % vertices.length]);

  const unsupported = [];
  for (let i = 0; i < n; i++) {
    if (usedPoint[i]) continue;
    const nb = tree.radius(points[i], scales[i] * 2.5, i);
    unsupported.push({
      index: i,
      reason: nb.length < 4
        ? '邻域过稀疏，未形成平面区域（噪点/密度骤变）'
        : '法线或平面距离不一致，未归入任何平面区域（非平面结构或空洞边缘）',
      neighbors: nb.length,
    });
  }

  return {
    vertices,
    faces,
    usedPoint: Array.from(usedPoint),
    unsupported,
    boundaryEdges,
    boundaryEdgeCount: boundaryEdges.length,
    regionReports,
    method: 'planar',
  };
}

// 合并共面碎片：法线夹角小且平面间距小的区域并为同一物理平面，合并后重新拟合
function mergeCoplanar(fitted, points, normals, angleDeg, distTol) {
  const cosMerge = Math.cos((angleDeg * Math.PI) / 180);
  const alive = fitted.map(() => true);
  let changed = true;
  while (changed) {
    changed = false;
    for (let i = 0; i < fitted.length; i++) {
      if (!alive[i]) continue;
      for (let j = i + 1; j < fitted.length; j++) {
        if (!alive[j]) continue;
        const A = fitted[i], B = fitted[j];
        if (Math.abs(dot(A.normal, B.normal)) < cosMerge) continue;
        const gap = Math.abs(dot(sub(B.centroid, A.centroid), A.normal));
        if (gap > distTol) continue;
        A.members = A.members.concat(B.members);
        Object.assign(A, fitPlane(points, A.members, normals));
        alive[j] = false;
        changed = true;
      }
    }
  }
  return fitted.filter((_, i) => alive[i]);
}

function fitPlane(points, members, normals) {
  let cx = 0, cy = 0, cz = 0;
  for (const i of members) { cx += points[i][0]; cy += points[i][1]; cz += points[i][2]; }
  const m = members.length;
  const centroid = [cx / m, cy / m, cz / m];
  let sxx = 0, syy = 0, szz = 0, sxy = 0, sxz = 0, syz = 0;
  for (const i of members) {
    const dx = points[i][0] - centroid[0], dy = points[i][1] - centroid[1], dz = points[i][2] - centroid[2];
    sxx += dx * dx; syy += dy * dy; szz += dz * dz;
    sxy += dx * dy; sxz += dx * dz; syz += dy * dz;
  }
  const inv = 1 / m;
  const cov = [[sxx * inv, sxy * inv, sxz * inv], [sxy * inv, syy * inv, syz * inv], [sxz * inv, syz * inv, szz * inv]];
  const { vectors } = symEig3(cov);
  let normal = vectors[0];
  if (normals) {
    let ax = 0, ay = 0, az = 0;
    for (const i of members) { ax += normals[i][0]; ay += normals[i][1]; az += normals[i][2]; }
    if (dot(normal, [ax, ay, az]) < 0) normal = [-normal[0], -normal[1], -normal[2]];
  }
  return { centroid, normal: normalize(normal) };
}
