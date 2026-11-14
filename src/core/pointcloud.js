// 点云数据结构。原始点一旦载入即冻结：任何"修订"都产生带版本号的新对象，
// 原数组只读保留，满足"原点不可覆盖"。
import { createHash } from 'node:crypto';

let cloudSeq = 0;

export class PointCloud {
  constructor({ points, views = null, normals = null, name = 'cloud', sourceFormat = 'unknown', originVersionId = null }) {
    this.id = `cloud_${++cloudSeq}_${Date.now().toString(36)}`;
    this.name = name;
    this.sourceFormat = sourceFormat;
    // points: Float64Array-like? 用普通数组的数组，便于 KD-tree
    this.points = points; // [[x,y,z], ...]
    this.views = views;   // [[vx,vy,vz], ...] 每点观测方向（传感器->点的单位向量，或给 null）
    this.normals = normals; // 估计后填入 [[nx,ny,nz], ...]
    this.createdAt = new Date().toISOString();
    this.immutable = true;
    this.originVersionId = originVersionId; // 若为修订版，指向被修订的版本 id
    this.revisions = [];
    this.hash = this._hash();
  }

  get n() { return this.points.length; }

  _hash() {
    const h = createHash('sha256');
    h.update(this.name);
    for (const p of this.points) h.update(`${p[0].toFixed(6)},${p[1].toFixed(6)},${p[2].toFixed(6)};`);
    return h.digest('hex').slice(0, 16);
  }

  // 修订（删除/恢复/编辑点）：返回新 PointCloud，原对象不动
  revise({ keepMask = null, keptIndices = null, label = 'revision' }) {
    let indices;
    if (keptIndices) indices = keptIndices.slice();
    else if (keepMask) {
      indices = [];
      for (let i = 0; i < keepMask.length; i++) if (keepMask[i]) indices.push(i);
    } else throw new Error('revise 需要 keepMask 或 keptIndices');
    const child = new PointCloud({
      points: indices.map((i) => this.points[i].slice()),
      views: this.views ? indices.map((i) => (this.views[i] ? this.views[i].slice() : null)) : null,
      name: `${this.name}@${label}`,
      sourceFormat: this.sourceFormat,
      originVersionId: this.originVersionId ?? this.id,
    });
    child.lineage = [...(this.lineage || [this.id]), this.id];
    return { cloud: child, keptIndices: indices, removedIndices: this._complement(indices) };
  }

  _complement(kept) {
    const set = new Set(kept);
    const out = [];
    for (let i = 0; i < this.n; i++) if (!set.has(i)) out.push(i);
    return out;
  }
}
