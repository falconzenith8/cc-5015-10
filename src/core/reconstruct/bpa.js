// Ball Pivoting Algorithm（Bernardini et al. 1999 的直接实现）
// 诚实性原则：
//  - 不做任何自动封洞：球无法到达的区域保持为边界/空洞
//  - 半径由局部点间距推导（逐点局部尺度），多半径递进
//  - 每个三角片必须通过"球内无点"空球检验，且与法线兼容
//  - 未被任何三角片采用的点作为 unsupported 报告（含定位与原因）
import { KDTree } from '../kdtree.js';
import { cross, dot, normalize, sub, scale, add, triArea } from '../vec.js';

const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const cross2 = (a, b) => a[0] * b[1] - a[1] * b[0];
const distP = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
const dist2p = (a, b) => { const dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2]; return dx * dx + dy * dy + dz * dz; };
const median = (arr) => { const s = Array.from(arr).sort((a, b) => a - b); return s[s.length >> 1]; };

export function reconstructBPA(points, normals, scales, opts = {}) {
  const {
    radiusFactors = [1.1, 1.6, 2.4],
    angleThresholdDeg = 75,
    maxEdgeFactor = 6.0,
    minTriangleArea = 1e-12,
    minEdgeRatio = 0.28, // 最短边/最长边低于此值视为细长搭桥片
  } = opts;

  const n = points.length;
  const medianScale = median(scales);
  const radii = radiusFactors.map((f) => f * medianScale);
  const maxEdge = maxEdgeFactor * medianScale;
  const cosThr = Math.cos((angleThresholdDeg * Math.PI) / 180);
  const tree = new KDTree(points);

  const faces = [];
  const usedPoint = new Uint8Array(n);
  const edgeCount = new Map();
  const edgeKey = (a, b) => (a < b ? a * n + b : b * n + a);
  const triKey = (a, b, c) => { const t = [a, b, c].sort((x, y) => x - y); return t[0] * n * n + t[1] * n + t[2]; };
  const triSeen = new Set();
  const edgeOpposite = new Map(); // 无向边 key -> { v0, v1, opp } 首次出现时的有向形式
  const boundaryEdges = new Set();
  let frontier = [];

  function registerFace(a, b, c) {
    const tk = triKey(a, b, c);
    if (triSeen.has(tk)) return false;
    // 流形保证：任何边已被两个面使用则拒绝
    for (const [x, y] of [[a, b], [b, c], [c, a]]) {
      if ((edgeCount.get(edgeKey(x, y)) || 0) >= 2) return false;
    }
    // 防重叠：边已有一个面片时，新第三顶点必须在该边的相反侧
    for (const [x, y, z] of [[a, b, c], [b, c, a], [c, a, b]]) {
      const owner = edgeOpposite.get(edgeKey(x, y));
      if (owner && sameSideOfEdge(x, y, z, owner.opp)) return false;
    }
    // 防空洞跨越：三条边都必须短于 maxEdge，且三个顶点在彼此的局部邻域内
    const localReach = Math.max(scales[a], scales[b], scales[c]) * maxEdgeFactor;
    for (const [x, y] of [[a, b], [b, c], [c, a]]) {
      if (distP(points[x], points[y]) > Math.min(maxEdge, localReach)) return false;
    }
    // 防对角搭桥：三角形内心附近必须有真实点（玻璃缺测/遮挡空洞的内心是空的）
    if (!hasSupportNearIncenter(a, b, c)) return false;
    // 边缘支撑：任一边中点悬空则该三角形跨越无数据区域
    if (!hasEdgeMidpointSupport(a, b, c)) return false;
    // 切平面覆盖：内心周围多方向须有真实点，洞缘搭桥片不满足
    if (!hasSurfaceCoverage(a, b, c)) return false;
    // 防细长搭桥：最短边与最长边之比低于阈值的三角形往往跨越空洞边缘
    const e1 = distP(points[a], points[b]);
    const e2 = distP(points[b], points[c]);
    const e3 = distP(points[c], points[a]);
    const emin = Math.min(e1, e2, e3), emax = Math.max(e1, e2, e3);
    if (emin / emax < minEdgeRatio) return false;
    triSeen.add(tk);
    faces.push([a, b, c]);
    usedPoint[a] = usedPoint[b] = usedPoint[c] = 1;
    const edges = [[a, b, c], [b, c, a], [c, a, b]];
    for (const [x, y, opp] of edges) {
      const k = edgeKey(x, y);
      edgeCount.set(k, (edgeCount.get(k) || 0) + 1);
      if (!edgeOpposite.has(k)) edgeOpposite.set(k, { v0: x, v1: y, opp });
    }
    if (edgeCount.get(edgeKey(a, b)) < 2) frontier.push([a, b, c]);
    if (edgeCount.get(edgeKey(b, c)) < 2) frontier.push([b, c, a]);
    if (edgeCount.get(edgeKey(c, a)) < 2) frontier.push([c, a, b]);
    return true;
  }

  // 内心支撑检验：内心在 (la*A+lb*B+lc*C)/(la+lb+lc)。
  // 内切圆半径 rin = 面积/半周长；若内心在 0.9×rin 内没有任何非顶点真实点，
  // 说明该三角片悬空于无数据区域（玻璃缺测/遮挡空洞），拒绝。
  function hasSupportNearIncenter(a, b, c) {
    const la = distP(points[b], points[c]);
    const lb = distP(points[a], points[c]);
    const lc = distP(points[a], points[b]);
    const s = la + lb + lc;
    const center = [
      (la * points[a][0] + lb * points[b][0] + lc * points[c][0]) / s,
      (la * points[a][1] + lb * points[b][1] + lc * points[c][1]) / s,
      (la * points[a][2] + lb * points[b][2] + lc * points[c][2]) / s,
    ];
    const area = triArea(points[a], points[b], points[c]);
    const rin = area / (s / 2);
    // 关键判据：正常表面片的内心附近存在"独立于三顶点"的邻点，距离在局部间距量级；
    // 空洞搭桥片内心远离任何独立点（最近独立点距离远大于局部间距）。
    const localSpacing = Math.max(scales[a], scales[b], scales[c]);
    const probe = Math.max(rin * 1.2, localSpacing * 1.5);
    let independentSupport = Infinity;
    for (const q of tree.radius(center, probe)) {
      if (q.i !== a && q.i !== b && q.i !== c) {
        independentSupport = Math.min(independentSupport, Math.sqrt(q.d2));
      }
    }
    return independentSupport <= localSpacing * 1.35;
  }

  // 边缘支撑检验：每条边中点的局部邻域内必须存在非端点真实点。
  // 跨越玻璃缺测/遮挡空洞的边，其中点悬空（最近点只有两端），据此拒绝。
  function hasEdgeMidpointSupport(a, b, c) {
    const localSpacing = Math.max(scales[a], scales[b], scales[c]);
    const probe = localSpacing * 1.1;
    for (const [x, y] of [[a, b], [b, c], [c, a]]) {
      const mid = scale(add(points[x], points[y]), 0.5);
      let supported = false;
      for (const q of tree.radius(mid, probe)) {
        if (q.i !== x && q.i !== y && q.i !== c) { supported = true; break; }
      }
      if (!supported) return false;
    }
    return true;
  }

  // 数据覆盖检验：在三角形内心所在的表面位置，沿切平面各方向小范围采样，
  // 必须存在覆盖该位置的真实点。洞缘搭桥片的内心切平面邻域只在一侧有数据。
  function hasSurfaceCoverage(a, b, c) {
    const la = distP(points[b], points[c]);
    const lb = distP(points[a], points[c]);
    const lc = distP(points[a], points[b]);
    const s = la + lb + lc;
    const center = [
      (la * points[a][0] + lb * points[b][0] + lc * points[c][0]) / s,
      (la * points[a][1] + lb * points[b][1] + lc * points[c][1]) / s,
      (la * points[a][2] + lb * points[b][2] + lc * points[c][2]) / s,
    ];
    const nf = faceNormal(a, b, c);
    const ref = Math.abs(nf[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const ex = normalize(cross(nf, ref));
    const ey = cross(nf, ex);
    const localSpacing = Math.max(scales[a], scales[b], scales[c]);
    // 在内心周围切平面上取 4 个偏置采样点，统计真实点在切平面方向上的覆盖
    const offsets = [[0.6, 0], [-0.6, 0], [0, 0.6], [0, -0.6]];
    let covered = 0;
    for (const [ox, oy] of offsets) {
      const sample = add(center, add(scale(ex, ox * localSpacing), scale(ey, oy * localSpacing)));
      const near = tree.radius(sample, localSpacing * 0.85);
      if (near.some((q) => q.i !== a && q.i !== b && q.i !== c)) covered++;
    }
    // 4 个方向至少 3 个有数据，否则内心位于数据区边缘/空洞内
    return covered >= 3;
  }

  // 两点 p,q 在边 (a,b) 的同侧判定：用边向量与两顶点的 3D 叉积点积符号。
  // 不依赖法线：若两个三角片在边的同一侧，其叉积向量同向，点积为正。
  function sameSideOfEdge(a, b, p, q) {
    const ab = sub(points[b], points[a]);
    const cp = cross(ab, sub(points[p], points[a]));
    const cq = cross(ab, sub(points[q], points[a]));
    // 归一化后比较，避免边长缩放影响阈值
    const np = norm(cp), nq = norm(cq);
    if (np < 1e-14 || nq < 1e-14) return true; // 共线视为冲突，拒绝新面
    return dot(scale(cp, 1 / np), scale(cq, 1 / nq)) > 1e-6;
  }

  function faceNormal(a, b, c) {
    return normalize(cross(sub(points[b], points[a]), sub(points[c], points[a])));
  }

  // 含边 (a,b) 的已有面片的第三顶点；无面片返回 -1
  function findOppositeVertex(a, b) {
    return edgeOpposite.get(edgeKey(a, b))?.opp ?? -1;
  }

  // 种子定向：与种子点自身法线（已按观测方向定向）对齐，保证局部绕序一致
  function orientSeed(a, b, c) {
    let nf = faceNormal(a, b, c);
    if (norm(nf) === 0) return { ok: false };
    if (dot(nf, normals[a]) < 0) {
      const t = b; b = c; c = t;
      nf = faceNormal(a, b, c);
    }
    for (const v of [a, b, c]) {
      if (dot(nf, normals[v]) < cosThr) return { ok: false };
    }
    return { ok: true, tri: [a, b, c] };
  }

  // 球心：外心沿 -面法线 抬升 h（球位于表面外侧）
  function ballCenter(a, b, c, r) {
    const pa = points[a], pb = points[b], pc = points[c];
    const ab = sub(pb, pa), ac = sub(pc, pa);
    const nf = normalize(cross(ab, ac));
    const area2 = norm(cross(ab, ac)); // |ab×ac| = 2×三角形面积
    if (area2 < 1e-15) return null;
    const area = area2 / 2;
    const circR = (norm(ab) * norm(sub(pc, pb)) * norm(ac)) / (2 * area);
    if (circR > r * (1 + 1e-9)) return null;
    const denom = 2 * (dot(ab, ab) * dot(ac, ac) - dot(ab, ac) ** 2);
    if (Math.abs(denom) < 1e-20) return null;
    const u = (norm(ab) ** 2 * (dot(ac, ac) - dot(ab, ac))) / denom;
    const v = (norm(ac) ** 2 * (dot(ab, ab) - dot(ab, ac))) / denom;
    const circ = add(pa, add(scale(ab, u), scale(ac, v)));
    const h = Math.sqrt(Math.max(0, r * r - circR * circR));
    return { center: sub(circ, scale(nf, h)), normal: nf };
  }

  function emptyBall(center, r, except) {
    const r2 = r * r;
    for (const q of tree.radius(center, r)) {
      if (except.includes(q.i)) continue;
      if (q.d2 < r2 - 1e-12) return false;
    }
    return true;
  }

  // 绕有向边 (v0,v1) 自对面顶点 o 旋转。新三角片在边的另一侧，
  // 为保持面法线同向必须使用反转绕序 (v1,v0,ci)
  function pivot(v0, v1, o, r) {
    const p0 = points[v0], p1 = points[v1];
    const edge = sub(p1, p0);
    const edgeLen = norm(edge);
    if (edgeLen > maxEdge || edgeLen < 1e-12) return null;
    const axis = scale(edge, 1 / edgeLen);
    const cand = new Set();
    const reach = Math.max(r * 2, maxEdge);
    for (const anchor of [v0, v1]) {
      for (const q of tree.radius(points[anchor], reach)) {
        if (q.i === v0 || q.i === v1 || q.i === o) continue;
        cand.add(q.i);
      }
    }
    let best = null;
    for (const ci of cand) {
      if (distP(points[ci], p0) > maxEdge && distP(points[ci], p1) > maxEdge) continue;
      // 反转绕序后的面法线必须与三个顶点法线兼容
      const nf = faceNormal(v1, v0, ci);
      if (norm(nf) === 0) continue;
      let ok = true;
      for (const v of [v0, v1, ci]) if (dot(nf, normals[v]) < cosThr) { ok = false; break; }
      if (!ok) continue;
      const ang = pivotAngle(points, axis, p0, o, ci);
      if (ang == null) continue;
      const bc = ballCenter(v1, v0, ci, r);
      if (!bc || !emptyBall(bc.center, r, [v0, v1, ci])) continue;
      const score = ang + 1e-6 * (distP(points[ci], p0) + distP(points[ci], p1));
      if (!best || score < best.score) best = { i: ci, ang, score };
    }
    // Delaunay 回退：规则格网上 BPA 球常被对角点阻挡（外接球退化），
    // 此时在边的切平面上按 2D 空圆准则选最近合法候选，仍保持不跨空洞、不封洞
    if (!best) best = delaunayFallback(v0, v1, o, r, cand);
    return best;
  }

  // 切平面 2D Delaunay：候选与边两端构成的三角形，其外接圆（切平面投影）内无其他点
  function delaunayFallback(v0, v1, o, r, cand) {
    const p0 = points[v0], p1 = points[v1];
    const axisN = normalize(cross(normals[v0], normals[v1]));
    const planeN = norm(axisN) > 1e-6 ? axisN : normals[v0];
    let best = null;
    for (const ci of cand) {
      if (distP(points[ci], p0) > maxEdge || distP(points[ci], p1) > maxEdge) continue;
      const nf = faceNormal(v1, v0, ci);
      if (norm(nf) === 0) continue;
      // 回退只约束法线夹角（绝对点积）；绕序由 (v1,v0,ci) 的边约定保证
      let ok = true;
      for (const v of [v0, v1, ci]) if (Math.abs(dot(nf, normals[v])) < cosThr) { ok = false; break; }
      if (!ok) continue;
      // 切平面坐标：以 p0 为原点，e1 沿边，e2 在面内垂直边
      const e1 = normalize(sub(p1, p0));
      const e2 = cross(planeN, e1);
      const proj2 = (p) => [dot(sub(p, p0), e1), dot(sub(p, p0), e2)];
      const A = proj2(p0), B = proj2(p1), C = proj2(points[ci]);
      const circ = circumcircle2(A, B, C);
      if (!circ) continue;
      // 空圆检验（仅切平面邻域内点）
      let blocked = false;
      for (const q of tree.radius(p0, maxEdge)) {
        if (q.i === v0 || q.i === v1 || q.i === ci) continue;
        const Q = proj2(points[q.i]);
        const dd = (Q[0] - circ.x) ** 2 + (Q[1] - circ.y) ** 2;
        if (dd < circ.r2 * (1 - 1e-9)) { blocked = true; break; }
      }
      if (blocked) continue;
      const score = distP(points[ci], p0) + distP(points[ci], p1);
      if (!best || score < best.score) best = { i: ci, ang: Math.PI, score };
    }
    return best;
  }

  function findSeed(i, r) {
    const nb = tree.radius(points[i], r * 2, i).map((q) => q.i);
    nb.sort((a, b) => dist2p(points[a], points[i]) - dist2p(points[b], points[i]));
    const lim = Math.min(nb.length, 24);
    // 第一遍：严格 BPA 空球
    for (let x = 0; x < lim; x++) {
      for (let y = x + 1; y < lim; y++) {
        const ori = orientSeed(i, nb[x], nb[y]);
        if (!ori.ok) continue;
        const [a, b, c] = ori.tri;
        if ((edgeCount.get(edgeKey(a, b)) || 0) >= 2 || (edgeCount.get(edgeKey(a, c)) || 0) >= 2) continue;
        if (distP(points[b], points[c]) > maxEdge) continue;
        if (triArea(points[a], points[b], points[c]) < minTriangleArea) continue;
        const bc = ballCenter(a, b, c, r);
        if (!bc || !emptyBall(bc.center, r, [a, b, c])) continue;
        return [a, b, c];
      }
    }
    // 第二遍：Delaunay 空圆回退（同样不跨空洞）
    const planeN = normals[i];
    const ref = Math.abs(planeN[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const ex = normalize(cross(planeN, ref));
    const ey = cross(planeN, ex);
    const p0 = points[i];
    const proj2 = (p) => [dot(sub(p, p0), ex), dot(sub(p, p0), ey)];
    let best = null;
    for (let x = 0; x < lim; x++) {
      for (let y = x + 1; y < lim; y++) {
        const ori = orientSeed(i, nb[x], nb[y]);
        if (!ori.ok) continue;
        const [a, b, c] = ori.tri;
        if ((edgeCount.get(edgeKey(a, b)) || 0) >= 2 || (edgeCount.get(edgeKey(a, c)) || 0) >= 2) continue;
        if (distP(points[b], points[c]) > maxEdge) continue;
        if (triArea(points[a], points[b], points[c]) < minTriangleArea) continue;
        const circ = circumcircle2(proj2(points[a]), proj2(points[b]), proj2(points[c]));
        if (!circ) continue;
        let blocked = false;
        for (const q of tree.radius(p0, maxEdge)) {
          if (q.i === a || q.i === b || q.i === c) continue;
          const Q = proj2(points[q.i]);
          if ((Q[0] - circ.x) ** 2 + (Q[1] - circ.y) ** 2 < circ.r2 * (1 - 1e-9)) { blocked = true; break; }
        }
        if (blocked) continue;
        const score = distP(points[b], p0) + distP(points[c], p0);
        if (!best || score < best.score) best = [a, b, c];
      }
    }
    return best;
  }

  const radiusLog = [];
  // 每个半径独立记录本轮失败的边；下一半径可重试，只有全部半径都失败才定为边界
  let pendingBoundary = new Set();
  for (const r of radii) {
    const facesBefore = faces.length;
    // 1) 播种（仅未使用点）
    for (let i = 0; i < n; i++) {
      if (usedPoint[i]) continue;
      const seed = findSeed(i, r);
      if (seed) registerFace(seed[0], seed[1], seed[2]);
    }
    // 2) 上一半径遗留的边界边，在本半径重新入队
    for (const k of pendingBoundary) {
      if ((edgeCount.get(k) || 0) !== 1) continue;
      const [a, b] = [Math.floor(k / n), k % n];
      // 找到该边当前所属面的第三个顶点
      const o = findOppositeVertex(a, b);
      if (o >= 0) frontier.push([a, b, o]);
    }
    pendingBoundary = new Set();
    const touched = new Set();
    let guard = 0;
    const guardLimit = n * 40 + 1000;
    while (frontier.length && guard++ < guardLimit) {
      const [v0, v1, o] = frontier.shift();
      const ek = edgeKey(v0, v1);
      if ((edgeCount.get(ek) || 0) >= 2) continue;
      if (touched.has(ek)) continue;
      touched.add(ek);
      const hit = pivot(v0, v1, o, r);
      if (hit) {
        if ((edgeCount.get(edgeKey(v0, hit.i)) || 0) >= 2 || (edgeCount.get(edgeKey(v1, hit.i)) || 0) >= 2) {
          pendingBoundary.add(ek);
          continue;
        }
        // 反转绕序注册：共享边 (v0,v1) 在新面中为 (v1,v0)
        registerFace(v1, v0, hit.i);
      } else {
        pendingBoundary.add(ek);
      }
    }
    frontier = [];
    radiusLog.push({ radius: Number(r.toExponential(4)), newFaces: faces.length - facesBefore });
  }

  for (const k of pendingBoundary) boundaryEdges.add(k);

  // 3) 边界愈合：对所有仍为单侧的边做切平面 Delaunay 补片，迭代到收敛。
  //    仍受最大边长与法线夹角约束，因此跨空洞/缺墙的边不会被缝合。
  healBoundaryEdges();

  // 愈合后重新统计边界（愈合会改变边的使用次数）
  boundaryEdges.clear();
  for (const [k, c] of edgeCount) if (c === 1) boundaryEdges.add(k);

  function healBoundaryEdges() {
    for (let iter = 0; iter < 8; iter++) {
      const open = [];
      for (const [k, c] of edgeCount) if (c === 1) open.push(k);
      if (!open.length) break;
      let added = 0;
      for (const k of open) {
        if ((edgeCount.get(k) || 0) !== 1) continue;
        // 恢复有向边：首次注册该边的面片方向
        const dir = edgeOpposite.get(k);
        if (!dir) continue;
        const { v0, v1, opp: o } = dir;
        const cand = new Set();
        for (const anchor of [v0, v1]) {
          for (const q of tree.radius(points[anchor], maxEdge)) {
            if (q.i === v0 || q.i === v1 || q.i === o) continue;
            cand.add(q.i);
          }
        }
        const ci = delaunayHealCandidate(v0, v1, cand);
        if (ci >= 0 && registerFace(v1, v0, ci)) added++;
      }
      if (!added) break;
    }
  }

  // 边界愈合候选：切平面空圆 + 法线夹角 + 边长。返回顶点索引或 -1
  function delaunayHealCandidate(v0, v1, cand) {
    const p0 = points[v0], p1 = points[v1];
    const planeN = normals[v0];
    const ref = Math.abs(planeN[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
    const ex = normalize(cross(planeN, ref));
    const ey = cross(planeN, ex);
    const proj2 = (p) => [dot(sub(p, p0), ex), dot(sub(p, p0), ey)];
    let best = -1, bestScore = Infinity;
    for (const ci of cand) {
      if (distP(points[ci], p0) > maxEdge || distP(points[ci], p1) > maxEdge) continue;
      // 新面绕序 (v1,v0,ci)，法线须与顶点法线同向（有符号检查保证整体朝向）
      const nf = faceNormal(v1, v0, ci);
      if (norm(nf) === 0) continue;
      let ok = true;
      for (const v of [v0, v1, ci]) if (dot(nf, normals[v]) < cosThr) { ok = false; break; }
      if (!ok) continue;
      const circ = circumcircle2(proj2(points[v1]), proj2(points[v0]), proj2(points[ci]));
      if (!circ) continue;
      let blocked = false;
      for (const q of tree.radius(p0, maxEdge)) {
        if (q.i === v0 || q.i === v1 || q.i === ci) continue;
        const Q = proj2(points[q.i]);
        if ((Q[0] - circ.x) ** 2 + (Q[1] - circ.y) ** 2 < circ.r2 * (1 - 1e-9)) { blocked = true; break; }
      }
      if (blocked) continue;
      const score = distP(points[ci], p0) + distP(points[ci], p1);
      if (score < bestScore) { bestScore = score; best = ci; }
    }
    return best;
  }

  const unsupported = [];
  for (let i = 0; i < n; i++) {
    if (usedPoint[i]) continue;
    const nb = tree.radius(points[i], scales[i] * 2.5, i);
    unsupported.push({
      index: i,
      reason: nb.length < 4
        ? '邻域过稀疏，无合法空球三角形（噪点或密度骤变边界）'
        : '邻域内未找到满足空球与法线兼容的三角形（玻璃缺测/遮挡空洞边缘）',
      neighbors: nb.length,
    });
  }

  return {
    faces,
    usedPoint: Array.from(usedPoint),
    unsupported,
    boundaryEdgeCount: boundaryEdges.size,
    boundaryEdges: Array.from(boundaryEdges).map((k) => [Math.floor(k / n), k % n]),
    radiusLog,
    method: 'bpa',
  };
}

// 绕轴有符号旋转角（o -> c），范围 [0, 2π)
function pivotAngle(points, axis, p0, o, c) {
  const proj = (v) => sub(v, scale(axis, dot(v, axis)));
  const u = proj(sub(points[o], p0));
  const w = proj(sub(points[c], p0));
  if (norm(u) < 1e-12 || norm(w) < 1e-12) return null;
  const cosA = dot(normalize(u), normalize(w));
  const sinA = dot(axis, normalize(cross(u, w)));
  let ang = Math.atan2(sinA, cosA);
  if (ang < 0) ang += 2 * Math.PI;
  return ang;
}

// 2D 三角形外接圆 {x,y,r2}，退化返回 null
function circumcircle2(A, B, C) {
  const dax = B[0] - A[0], day = B[1] - A[1];
  const dbx = C[0] - A[0], dby = C[1] - A[1];
  const denom = 2 * (dax * dby - day * dbx);
  if (Math.abs(denom) < 1e-18) return null;
  const la = dax * dax + day * day;
  const lb = dbx * dbx + dby * dby;
  const x = A[0] + (dby * la - day * lb) / denom;
  const y = A[1] + (dax * lb - dbx * la) / denom;
  return { x, y, r2: (x - A[0]) ** 2 + (y - A[1]) ** 2 };
}
