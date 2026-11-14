// 点云/网格 IO：解析 PLY(ascii/binary-le)、XYZ、JSON；导出 PLY/OBJ/JSON
import { PointCloud } from './pointcloud.js';

// ---------------- 点云解析 ----------------
export function parsePointCloud(textOrBuf, filename) {
  const ext = filename.split('.').pop().toLowerCase();
  if (ext === 'json') return parseJSON(textOrBuf.toString('utf8'), filename);
  if (ext === 'ply') return parsePLY(Buffer.isBuffer(textOrBuf) ? textOrBuf : Buffer.from(textOrBuf), filename);
  if (ext === 'xyz' || ext === 'txt' || ext === 'pts') return parseXYZ(textOrBuf.toString('utf8'), filename);
  throw new Error(`不支持的点云格式: .${ext}（支持 .ply/.xyz/.txt/.pts/.json）`);
}

function parseJSON(text, filename) {
  const data = JSON.parse(text);
  if (!Array.isArray(data.points)) throw new Error('JSON 点云需要 points 数组');
  const points = data.points.map((p) => [Number(p[0]), Number(p[1]), Number(p[2])]);
  const views = Array.isArray(data.views)
    ? data.views.map((v) => (v == null ? null : [Number(v[0]), Number(v[1]), Number(v[2])]))
    : null;
  return new PointCloud({ points, views, name: data.name || filename, sourceFormat: 'json' });
}

function parseXYZ(text, filename) {
  const points = [], views = [];
  let hasViews = false;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line || line.startsWith('#')) continue;
    const t = line.split(/[\s,]+/).map(Number);
    if (t.length < 3 || t.some(Number.isNaN)) continue;
    points.push([t[0], t[1], t[2]]);
    if (t.length >= 6) { views.push([t[3], t[4], t[5]]); hasViews = true; }
    else views.push(null);
  }
  if (!points.length) throw new Error('XYZ 文件中未解析到任何点');
  return new PointCloud({ points, views: hasViews ? views : null, name: filename, sourceFormat: 'xyz' });
}

function parsePLY(buf, filename) {
  // 解析 header
  let pos = 0;
  const headerEnd = buf.indexOf(Buffer.from('end_header'));
  if (headerEnd < 0) throw new Error('PLY 缺少 end_header');
  const header = buf.slice(0, headerEnd).toString('ascii');
  const lines = header.split(/\r?\n/);
  let format = 'ascii';
  const elements = [];
  let cur = null;
  for (const line of lines) {
    const t = line.trim().split(/\s+/);
    if (t[0] === 'format') format = t[1];
    else if (t[0] === 'element') { cur = { name: t[1], count: Number(t[2]), props: [] }; elements.push(cur); }
    else if (t[0] === 'property' && cur) cur.props.push({ type: t[1], name: t[2] });
  }
  pos = headerEnd + 'end_header'.length;
  while (buf[pos] === 0x0d || buf[pos] === 0x0a) pos++;

  const vertEl = elements.find((e) => e.name === 'vertex');
  if (!vertEl) throw new Error('PLY 中没有 vertex 元素');
  const idx = (n) => vertEl.props.findIndex((p) => p.name === n);
  const ix = idx('x'), iy = idx('y'), iz = idx('z');
  const inv = idx('nx'), inv2 = idx('ny'), inv3 = idx('nz');
  const ivx = idx('sdx'), ivy = idx('sdy'), ivz = idx('sdz');
  const typeSize = { char: 1, int8: 1, uchar: 1, uint8: 1, short: 2, int16: 2, ushort: 2, uint16: 2, int: 4, int32: 4, uint: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 };

  const points = [], views = [], normals = [];
  if (format === 'ascii') {
    const text = buf.slice(pos).toString('ascii');
    const rows = text.split(/\r?\n/).filter((l) => l.trim().length);
    for (let r = 0; r < vertEl.count; r++) {
      const vals = rows[r].trim().split(/\s+/).map(Number);
      points.push([vals[ix], vals[iy], vals[iz]]);
      if (ivx >= 0 && vals[ivx] !== 0 && vals[ivy] !== 0 && vals[ivz] !== 0) views.push([vals[ivx], vals[ivy], vals[ivz]]); else views.push(null);
      if (inv >= 0) normals.push([vals[inv], vals[inv2], vals[inv3]]);
    }
  } else {
    const le = format.includes('little');
    let p = pos;
    for (let r = 0; r < vertEl.count; r++) {
      const vals = new Array(vertEl.props.length);
      vertEl.props.forEach((pr, k) => {
        const sz = typeSize[pr.type];
        if (pr.type === 'float' || pr.type === 'float32') vals[k] = buf.readFloatLE(p);
        else if (pr.type === 'double' || pr.type === 'float64') vals[k] = buf.readDoubleLE(p);
        else if (pr.type.startsWith('uint') || pr.type === 'uchar' || pr.type === 'ushort') vals[k] = sz === 1 ? buf.readUInt8(p) : sz === 2 ? buf.readUInt16LE(p) : buf.readUInt32LE(p);
        else vals[k] = sz === 1 ? buf.readInt8(p) : sz === 2 ? buf.readInt16LE(p) : buf.readInt32LE(p);
        p += sz;
      });
      points.push([vals[ix], vals[iy], vals[iz]]);
      if (ivx >= 0 && (vals[ivx] !== 0 || vals[ivy] !== 0 || vals[ivz] !== 0)) views.push([vals[ivx], vals[ivy], vals[ivz]]); else views.push(null);
      if (inv >= 0) normals.push([vals[inv], vals[inv2], vals[inv3]]);
    }
    if (!le) throw new Error('仅支持 little-endian 二进制 PLY');
  }
  const hasViews = views.some((v) => v !== null);
  const hasNormals = normals.length === points.length;
  return new PointCloud({
    points,
    views: hasViews ? views : null,
    normals: hasNormals ? normals : null,
    name: filename,
    sourceFormat: 'ply',
  });
}

// ---------------- 网格导出 ----------------
// mesh: { vertices:[[x,y,z]...], faces:[[a,b,c]...], faceMeta? }
export function meshToPLY(mesh, binary = false) {
  const n = mesh.vertices.length, f = mesh.faces.length;
  const header = `ply\nformat ${binary ? 'binary_little_endian_1.0' : 'ascii_1.0'}\nelement vertex ${n}\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face ${f}\nproperty list uchar int vertex_indices\nend_header\n`;
  if (!binary) {
    const lines = [header];
    mesh.vertices.forEach((v, i) => {
      const nm = mesh.vertexNormals?.[i] || [0, 0, 0];
      lines.push(`${v[0]} ${v[1]} ${v[2]} ${nm[0]} ${nm[1]} ${nm[2]}`);
    });
    mesh.faces.forEach((t) => lines.push(`3 ${t[0]} ${t[1]} ${t[2]}`));
    return Buffer.from(lines.join('\n') + '\n');
  }
  const head = Buffer.from(header);
  const body = Buffer.alloc(n * 24 + f * 13);
  let o = 0;
  mesh.vertices.forEach((v, i) => {
    const nm = mesh.vertexNormals?.[i] || [0, 0, 0];
    body.writeFloatLE(v[0], o); body.writeFloatLE(v[1], o + 4); body.writeFloatLE(v[2], o + 8);
    body.writeFloatLE(nm[0], o + 12); body.writeFloatLE(nm[1], o + 16); body.writeFloatLE(nm[2], o + 20);
    o += 24;
  });
  mesh.faces.forEach((t) => { body.writeUInt8(3, o); body.writeInt32LE(t[0], o + 1); body.writeInt32LE(t[1], o + 5); body.writeInt32LE(t[2], o + 9); o += 13; });
  return Buffer.concat([head, body]);
}

export function meshToOBJ(mesh) {
  const lines = ['# Indoor PC Mesh Studio export'];
  for (const v of mesh.vertices) lines.push(`v ${v[0]} ${v[1]} ${v[2]}`);
  const hasN = Array.isArray(mesh.vertexNormals) && mesh.vertexNormals.length === mesh.vertices.length;
  if (hasN) for (const n of mesh.vertexNormals) lines.push(`vn ${n[0]} ${n[1]} ${n[2]}`);
  for (const t of mesh.faces) {
    if (hasN) lines.push(`f ${t[0] + 1}//${t[0] + 1} ${t[1] + 1}//${t[1] + 1} ${t[2] + 1}//${t[2] + 1}`);
    else lines.push(`f ${t[0] + 1} ${t[1] + 1} ${t[2] + 1}`);
  }
  return Buffer.from(lines.join('\n') + '\n');
}

export function meshToJSON(mesh, report) {
  return Buffer.from(JSON.stringify({ mesh, report }, null, 2));
}
