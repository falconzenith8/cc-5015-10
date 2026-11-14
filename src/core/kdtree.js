// 自写平衡 KD-tree：kNN 与半径查询，零依赖
// 点为普通数组 [x,y,z]，查询返回 { i: 原始索引, d2: 平方距离 }
export class KDTree {
  constructor(points, leafSize = 16) {
    this.points = points;
    this.leafSize = leafSize;
    this.order = new Int32Array(points.length);
    for (let i = 0; i < this.order.length; i++) this.order[i] = i;
    this.nodes = [];
    this._build(0, this.order.length, 0);
  }

  _build(lo, hi, depth) {
    const id = this.nodes.length;
    this.nodes.push(null);
    const axis = depth % 3;
    const n = hi - lo;
    if (n <= this.leafSize) {
      this.nodes[id] = { leaf: true, lo, hi };
      return id;
    }
    const mid = (lo + hi) >> 1;
    selectNth(this.points, this.order, lo, hi, mid, axis);
    const splitVal = this.points[this.order[mid]][axis];
    const left = this._build(lo, mid, depth + 1);
    const right = this._build(mid + 1, hi, depth + 1);
    this.nodes[id] = { leaf: false, axis, splitVal, left, right };
    return id;
  }

  // k 近邻，按距离升序
  knn(point, k, exclude = -1) {
    const heap = new MaxHeap(k);
    this._knn(0, point, heap, exclude);
    return heap.sorted();
  }

  _knn(nodeId, p, heap, exclude) {
    const node = this.nodes[nodeId];
    if (node.leaf) {
      const pts = this.points;
      for (let i = node.lo; i < node.hi; i++) {
        const pi = this.order[i];
        if (pi === exclude) continue;
        const q = pts[pi];
        const dx = q[0] - p[0], dy = q[1] - p[1], dz = q[2] - p[2];
        heap.push(pi, dx * dx + dy * dy + dz * dz);
      }
      return;
    }
    const diff = p[node.axis] - node.splitVal;
    const first = diff <= 0 ? node.left : node.right;
    const second = diff <= 0 ? node.right : node.left;
    this._knn(first, p, heap, exclude);
    if (!heap.full || diff * diff < heap.topD2()) {
      this._knn(second, p, heap, exclude);
    }
  }

  // 半径 r（距离）内全部点
  radius(point, r, exclude = -1) {
    const r2 = r * r;
    const out = [];
    this._radius(0, point, r2, exclude, out);
    return out;
  }

  _radius(nodeId, p, r2, exclude, out) {
    const node = this.nodes[nodeId];
    if (node.leaf) {
      const pts = this.points;
      for (let i = node.lo; i < node.hi; i++) {
        const pi = this.order[i];
        if (pi === exclude) continue;
        const q = pts[pi];
        const dx = q[0] - p[0], dy = q[1] - p[1], dz = q[2] - p[2];
        const d2 = dx * dx + dy * dy + dz * dz;
        if (d2 <= r2) out.push({ i: pi, d2 });
      }
      return;
    }
    const diff = p[node.axis] - node.splitVal;
    const first = diff <= 0 ? node.left : node.right;
    const second = diff <= 0 ? node.right : node.left;
    this._radius(first, p, r2, exclude, out);
    if (diff * diff <= r2) this._radius(second, p, r2, exclude, out);
  }
}

// Quickselect（三路分区）
function selectNth(points, idx, lo, hi, k, axis) {
  while (lo < hi - 1) {
    const pivot = points[idx[(lo + hi) >> 1]][axis];
    let i = lo, j = lo, e = hi;
    while (j < e) {
      const v = points[idx[j]][axis];
      if (v < pivot) { const t = idx[i]; idx[i] = idx[j]; idx[j] = t; i++; j++; }
      else if (v > pivot) { e--; const t = idx[j]; idx[j] = idx[e]; idx[e] = t; }
      else j++;
    }
    if (k < i) hi = i;
    else if (k >= e) lo = e;
    else return;
  }
}

// 固定容量最大堆，堆顶为当前最大距离（第 k 近）
class MaxHeap {
  constructor(k) { this.k = k; this.arr = []; }
  get full() { return this.arr.length >= this.k; }
  topD2() { return this.arr[0].d2; }
  push(i, d2) {
    if (this.arr.length < this.k) {
      this.arr.push({ i, d2 });
      this._up(this.arr.length - 1);
    } else if (d2 < this.arr[0].d2) {
      this.arr[0] = { i, d2 };
      this._down(0);
    }
  }
  _up(c) {
    const a = this.arr;
    while (c > 0) {
      const p = (c - 1) >> 1;
      if (a[p].d2 >= a[c].d2) break;
      const t = a[p]; a[p] = a[c]; a[c] = t;
      c = p;
    }
  }
  _down(p) {
    const a = this.arr, n = a.length;
    for (;;) {
      const l = 2 * p + 1, r = l + 1;
      let m = p;
      if (l < n && a[l].d2 > a[m].d2) m = l;
      if (r < n && a[r].d2 > a[m].d2) m = r;
      if (m === p) break;
      const t = a[p]; a[p] = a[m]; a[m] = t;
      p = m;
    }
  }
  sorted() { return this.arr.slice().sort((a, b) => a.d2 - b.d2); }
}
