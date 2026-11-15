// 零依赖 HTTP 服务：点云载入/修订、重建流水线、网格导出、内核 schema 下发。
// 不使用任何第三方框架；上传的不可信数据单独解析为 PointCloud，与源码隔离。
import { createServer } from 'node:http';
import { readFileSync, readdirSync, existsSync } from 'node:fs';
import { resolve, dirname, extname, join, normalize } from 'node:path';
import { fileURLToPath } from 'node:url';
import { PointCloud } from '../core/pointcloud.js';
import { runPipeline, PipelineError } from '../core/pipeline.js';
import { PARAM_SCHEMA, defaultParams } from '../core/schema.js';
import { parsePointCloud, meshToPLY, meshToOBJ, meshToJSON } from '../core/io.js';
import { pointToAnalyticDistance } from '../core/analytic.js';
import {
  nearestNeighborSpacing, pointSetToMeshDistances, faceSamplesToAnalytic,
  meshToAnalyticDistances, closureCoverage, meshTopology,
} from '../core/metrics.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const WEB_ROOT = resolve(__dirname, '../../web');
const DATA_DIR = resolve(__dirname, '../../data');
const FIXTURE_DIR = resolve(__dirname, '../../test/acceptance/fixtures');
const PORT = Number(process.env.PORT || 8080);

// 会话状态：原始冻结点云 + 修订版本 + 最近一次运行结果
const state = {
  originals: new Map(),   // cloudId -> PointCloud（永不改写）
  current: null,          // 当前活动 PointCloud（可能是修订版）
  currentId: null,
  currentFixture: null,   // 若载入的是验收集样例，记住其解析真值（仅用于距离检查展示）
  runs: new Map(),        // runId -> 完整流水线结果
  lastRunId: null,
};

const json = (res, code, obj) => {
  const body = JSON.stringify(obj);
  res.writeHead(code, { 'content-type': 'application/json; charset=utf-8' });
  res.end(body);
};

function readBody(req) {
  return new Promise((resolveBody, rejectBody) => {
    const chunks = [];
    let size = 0;
    req.on('data', (c) => {
      size += c.length;
      if (size > 200 * 1024 * 1024) { rejectBody(new Error('上传体积超过 200MB 限制')); req.destroy(); return; }
      chunks.push(c);
    });
    req.on('end', () => resolveBody(Buffer.concat(chunks)));
    req.on('error', rejectBody);
  });
}

function registerCloud(cloud) {
  state.originals.set(cloud.id, cloud);
  state.current = cloud;
  state.currentId = cloud.id;
  return cloudSummary(cloud);
}

// 验收集 fixture：真实解析真值 + 构造的未观测区域，供“手动载入 + 设置尺度”时做距离检查。
function loadFixtureDoc(name) {
  if (!/^[\w.-]+$/.test(name || '')) return null;
  const full = resolve(FIXTURE_DIR, `${name}.json`);
  if (!full.startsWith(FIXTURE_DIR) || !existsSync(full)) return null;
  return JSON.parse(readFileSync(full, 'utf8'));
}

function cloudSummary(cloud) {
  return {
    cloudId: cloud.id,
    name: cloud.name,
    sourceFormat: cloud.sourceFormat,
    pointCount: cloud.n,
    hash: cloud.hash,
    immutable: cloud.immutable,
    originVersionId: cloud.originVersionId ?? null,
    hasViews: Array.isArray(cloud.views) && cloud.views.some((v) => v !== null),
    lineage: cloud.lineage || [cloud.id],
    // 仅下发可视化所需几何，避免重复大对象
    points: cloud.points,
    views: cloud.views,
  };
}

//  multipart/form-data 极简解析（仅本服务上传用）
function parseMultipart(buf, contentType) {
  const m = /boundary=(?:"([^"]+)"|([^;]+))/.exec(contentType);
  if (!m) throw new Error('multipart 缺少 boundary');
  const boundary = '--' + (m[1] || m[2]).trim();
  const parts = [];
  let start = buf.indexOf(boundary);
  while (start >= 0) {
    const headerEnd = buf.indexOf('\r\n\r\n', start);
    if (headerEnd < 0) break;
    const headers = buf.slice(start, headerEnd).toString('utf8');
    const dataStart = headerEnd + 4;
    const next = buf.indexOf(boundary, dataStart);
    if (next < 0) break;
    let dataEnd = next - 2; // 去掉结尾 \r\n
    const nameMatch = /name="([^"]+)"/.exec(headers);
    const fileMatch = /filename="([^"]+)"/.exec(headers);
    if (nameMatch) {
      parts.push({
        name: nameMatch[1],
        filename: fileMatch ? fileMatch[1] : null,
        data: buf.slice(dataStart, Math.max(dataStart, dataEnd)),
      });
    }
    start = next;
  }
  return parts;
}

async function handleApi(req, res, url) {
  const route = url.pathname;

  if (req.method === 'GET' && route === '/api/schema') {
    return json(res, 200, { schema: PARAM_SCHEMA, defaults: defaultParams() });
  }

  if (req.method === 'GET' && route === '/api/scenarios') {
    const files = existsSync(DATA_DIR)
      ? readdirSync(DATA_DIR).filter((f) => f.endsWith('.json')).map((f) => f.replace(/\.json$/, ''))
      : [];
    return json(res, 200, { scenarios: files });
  }

  // 点云重建回归样例（test/acceptance/fixtures）：可从模型操作视窗直接打开。
  // 与数据场景分开列出，因为每个样例都带解析真值，可用于“重建表面与观测距离”检查。
  if (req.method === 'GET' && route === '/api/regression-samples') {
    const samples = existsSync(FIXTURE_DIR)
      ? readdirSync(FIXTURE_DIR)
        .filter((f) => f.endsWith('.json'))
        .map((f) => {
          try {
            const doc = JSON.parse(readFileSync(resolve(FIXTURE_DIR, f), 'utf8'));
            return {
              name: doc.name, description: doc.description,
              pointCount: doc.pointCount, hash: doc.hash,
              analyticKind: doc.analytic?.kind,
              unobserved: (doc.unobserved || []).map((r) => r.id),
            };
          } catch { return null; }
        })
        .filter(Boolean)
      : [];
    return json(res, 200, { samples });
  }

  if (req.method === 'POST' && route === '/api/load-regression-sample') {
    const { name } = JSON.parse((await readBody(req)).toString('utf8') || '{}');
    const doc = loadFixtureDoc(name);
    if (!doc) return json(res, 404, { error: `回归样例不存在: ${name}` });
    const exclude = new Set(doc.metadata?.cleanExcludeTags || []);
    const idx = [];
    for (let i = 0; i < doc.points.length; i++) if (!exclude.has(doc.tags?.[i])) idx.push(i);
    const cloud = new PointCloud({
      points: idx.map((i) => doc.points[i]), views: idx.map((i) => doc.views?.[i] ?? null),
      name: doc.name, sourceFormat: 'acceptance-fixture',
    });
    state.currentFixture = {
      name: doc.name, hash: doc.hash, analytic: doc.analytic,
      unobserved: doc.unobserved || [],
      excludedTagCount: doc.points.length - idx.length,
      noiseAmplitude: doc.metadata?.noiseAmplitude ?? null,
      derivation: doc.metadata?.derivation || null,
    };

    return json(res, 200, {
      cloud: registerCloud(cloud),
      sample: {
        name: doc.name, description: doc.description, hash: doc.hash,
        analyticKind: doc.analytic?.kind,
        cleanPointCount: idx.length,
        excludedTagCount: doc.points.length - idx.length,
        unobserved: (doc.unobserved || []).map((r) => r.id),
        derivation: doc.metadata?.derivation || null,
      },
    });
  }

  if (req.method === 'POST' && route === '/api/load-scenario') {
    const { name } = JSON.parse((await readBody(req)).toString('utf8') || '{}');
    if (!/^[\w.-]+$/.test(name || '')) return json(res, 400, { error: '非法场景名' });
    const full = resolve(DATA_DIR, `${name}.json`);
    if (!full.startsWith(DATA_DIR) || !existsSync(full)) return json(res, 404, { error: `场景不存在: ${name}` });
    const d = JSON.parse(readFileSync(full, 'utf8'));
    const cloud = new PointCloud({ points: d.points, views: d.views, name: d.name || name, sourceFormat: 'json' });
    state.currentFixture = null;
    return json(res, 200, { cloud: registerCloud(cloud), scenario: { description: d.description, expectations: d.expectations } });
  }

  if (req.method === 'POST' && route === '/api/upload') {
    const ct = req.headers['content-type'] || '';
    const buf = await readBody(req);
    let filename, data;
    if (ct.includes('multipart/form-data')) {
      const part = parseMultipart(buf, ct).find((p) => p.filename);
      if (!part) return json(res, 400, { error: '未找到上传文件部件' });
      filename = part.filename; data = part.data;
    } else {
      filename = new URL(req.url, 'http://x').searchParams.get('name') || 'upload.xyz';
      data = buf;
    }
    try {
      const cloud = parsePointCloud(data, filename);
      state.currentFixture = null;
      return json(res, 200, { cloud: registerCloud(cloud) });
    } catch (err) {
      return json(res, 400, { error: `点云解析失败: ${err.message}` });
    }
  }

  if (req.method === 'GET' && route === '/api/cloud') {
    if (!state.current) return json(res, 404, { error: '尚未载入点云' });
    return json(res, 200, { cloud: cloudSummary(state.current) });
  }

  // 修订：按保留/删除/恢复的索引产生新版本，原冻结点云不动
  if (req.method === 'POST' && route === '/api/revise') {
    if (!state.current) return json(res, 400, { error: '尚未载入点云' });
    const body = JSON.parse((await readBody(req)).toString('utf8') || '{}');
    const base = state.originals.get(state.current.originVersionId ?? state.currentId) || state.current;
    const n = base.n;
    let keepMask;
    if (body.mode === 'remove') {
      const set = new Set((body.indices || []).map(Number));
      keepMask = Array.from({ length: n }, (_, i) => !set.has(i));
    } else if (body.mode === 'restore') {
      const set = new Set((body.indices || []).map(Number));
      keepMask = Array.from({ length: n }, (_, i) => set.has(i));
    } else if (Array.isArray(body.keepMask)) {
      keepMask = body.keepMask;
    } else {
      return json(res, 400, { error: '修订需要 mode=remove|restore 加 indices，或 keepMask' });
    }
    const { cloud, keptIndices, removedIndices } = base.revise({ keepMask, label: body.label || 'revision' });
    state.current = cloud;
    state.currentId = cloud.id;
    // 修订后的点集已不是样例声明的那批，解析真值检查不再适用（避免用样例真值给别的点集背书）
    state.currentFixture = null;
    return json(res, 200, { cloud: cloudSummary(cloud), keptIndices, removedIndices, baseHash: base.hash });
  }

  // 运行重建流水线（参数由内核 schema 驱动；失败带阶段与定位回传）
  if (req.method === 'POST' && route === '/api/run') {
    if (!state.current) return json(res, 400, { error: '尚未载入点云' });
    const params = JSON.parse((await readBody(req)).toString('utf8') || '{}');
    try {
      const result = runPipeline(state.current, params);
      state.runs.set(result.runId, result);
      state.lastRunId = result.runId;
      return json(res, 200, serializeRun(result));
    } catch (err) {
      if (err instanceof PipelineError) {
        return json(res, 422, { error: '重建失败', pipelineError: err.failure, timings: err.timings });
      }
      return json(res, 500, { error: err.message });
    }
  }

  if (req.method === 'GET' && route === '/api/run/latest') {
    if (!state.lastRunId) return json(res, 404, { error: '尚无运行结果' });
    return json(res, 200, serializeRun(state.runs.get(state.lastRunId)));
  }

  // 观测距离检查：真实运算（点->重建表面距离，以及解析真值距离 / 错误封闭覆盖）。
  // 前端只展示这里返回的数值，不做任何前端自算或默认占位。
  // - 任何网格都可检查“观测点 -> 重建表面”距离（覆盖/贴合）；
  // - 若当前点云来自回归样例，额外给出“重建表面 -> 解析真值”距离与孔洞覆盖面积占比。
  if (req.method === 'GET' && route === '/api/distance-check') {
    if (!state.lastRunId) return json(res, 400, { error: '尚无运行结果：请先运行重建' });
    if (!state.current) return json(res, 400, { error: '尚未载入点云' });
    const result = state.runs.get(state.lastRunId);
    if (result.frozen.cloudHash !== state.current.hash) {
      return json(res, 409, {
        error: '当前点云与最近一次运行不一致（修订后尚未重跑）：请重新运行重建后再检查观测距离。',
      });
    }
    const fixture = state.currentFixture;
    const pts = state.current.points;
    const p2m = pointSetToMeshDistances(pts, result.mesh);
    const nns = nearestNeighborSpacing(pts);
    const base = {
      runId: result.runId,
      cloudName: result.frozen.cloudName,
      cloudHash: result.frozen.cloudHash,
      method: result.mesh.method,
      pointCount: pts.length,
      nnMedian: nns.median,
      pointToMesh: {
        count: p2m.count, uncoveredCount: p2m.uncoveredCount,
        median: p2m.median, p95: p2m.p95, max: p2m.max,
      },
      topology: meshTopology(result.mesh),
    };
    if (!fixture) {
      return json(res, 200, {
        ...base,
        hasAnalyticTruth: false,
        note: '当前点云无解析真值；观测距离仅表示“原始观测点 -> 重建表面”的贴合程度，不代表真值误差。',
      });
    }
    const surface = faceSamplesToAnalytic(result.mesh, fixture.analytic, pointToAnalyticDistance);
    const closure = closureCoverage(result.mesh, fixture.unobserved);
    const meshToTruth = meshToAnalyticDistances(result.mesh, fixture.analytic, pointToAnalyticDistance);
    const noise = fixture.noiseAmplitude;
    if (noise == null) {
      return json(res, 422, { error: `回归样例 ${fixture.name} 缺少 noiseAmplitude，无法独立推导容差` });
    }
    const tol = Math.max(4 * noise, 0.5 * nns.median);
    return json(res, 200, {
      ...base,
      hasAnalyticTruth: true,
      truth: {
        sample: fixture.name, hash: fixture.hash, analyticKind: fixture.analytic.kind,
        unobserved: fixture.unobserved.map((r) => r.id),
        excludedTagCount: fixture.excludedTagCount,
        derivation: fixture.derivation,
      },
      surfaceToTruth: {
        sampleCount: surface.sampleCount, mean: surface.mean, max: surface.max, maxFace: surface.maxFace,
        perVertexMedian: meshToTruth.median, perVertexP95: meshToTruth.p95, perVertexMax: meshToTruth.max,
      },
      closure: {
        coveredArea: closure.coveredArea, referenceArea: closure.referenceArea, fraction: closure.fraction,
        faceCount: closure.faces.length,
      },
      tolerance: {
        value: tol,
        rule: 'max(4×注入噪声幅度, 0.5×最近邻中位数)',
        noiseAmplitude: noise,
        nnMedian: nns.median,
      },
    });
  }

  // 网格导出
  if (req.method === 'GET' && route === '/api/export') {
    if (!state.lastRunId) return json(res, 400, { error: '尚无运行结果可导出' });
    const result = state.runs.get(state.lastRunId);
    const fmt = (url.searchParams.get('format') || 'ply').toLowerCase();
    const mesh = { vertices: result.mesh.vertices, faces: result.mesh.faces, vertexNormals: result.mesh.vertexNormals };
    const stamp = `${result.frozen.cloudName}_${result.mesh.method}`;
    if (fmt === 'ply') {
      const buf = meshToPLY(mesh, false);
      res.writeHead(200, { 'content-type': 'application/octet-stream', 'content-disposition': `attachment; filename="${stamp}.ply"` });
      return res.end(buf);
    }
    if (fmt === 'plybin') {
      const buf = meshToPLY(mesh, true);
      res.writeHead(200, { 'content-type': 'application/octet-stream', 'content-disposition': `attachment; filename="${stamp}.ply"` });
      return res.end(buf);
    }
    if (fmt === 'obj') {
      const buf = meshToOBJ(mesh);
      res.writeHead(200, { 'content-type': 'text/plain; charset=utf-8', 'content-disposition': `attachment; filename="${stamp}.obj"` });
      return res.end(buf);
    }
    if (fmt === 'json') {
      const buf = meshToJSON(mesh, { frozen: result.frozen, qc: result.qc });
      res.writeHead(200, { 'content-type': 'application/json; charset=utf-8', 'content-disposition': `attachment; filename="${stamp}.json"` });
      return res.end(buf);
    }
    return json(res, 400, { error: `不支持的导出格式: ${fmt}` });
  }

  return json(res, 404, { error: `未知 API: ${route}` });
}

// 精简下发：几何与质检全量，但控制体积；逐点距离数组保留供前端着色
function serializeRun(r) {
  return {
    runId: r.runId,
    startedAt: r.startedAt,
    finishedAt: r.finishedAt,
    frozen: r.frozen,
    stageTimings: r.stageTimings,
    scales: r.scales,
    noise: {
      candidates: r.noise.candidates,
      stats: r.noise.stats,
    },
    normals: r.normals,
    mesh: {
      vertices: r.mesh.vertices,
      faces: r.mesh.faces,
      method: r.mesh.method,
      boundaryEdges: r.mesh.boundaryEdges,
      boundaryEdgeCount: r.mesh.boundaryEdgeCount,
      regionReports: r.mesh.regionReports,
      radiusLog: r.mesh.radiusLog,
      unsupported: r.mesh.unsupported,
    },
    qc: r.qc,
  };
}

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
};

function serveStatic(req, res, url) {
  let rel = decodeURIComponent(url.pathname);
  if (rel === '/') rel = '/index.html';
  const filePath = normalize(join(WEB_ROOT, rel));
  if (!filePath.startsWith(WEB_ROOT)) { res.writeHead(403); return res.end('forbidden'); }
  if (!existsSync(filePath)) { res.writeHead(404); return res.end('not found'); }
  const buf = readFileSync(filePath);
  res.writeHead(200, { 'content-type': MIME[extname(filePath)] || 'application/octet-stream' });
  res.end(buf);
}

const server = createServer((req, res) => {
  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
  if (url.pathname.startsWith('/api/')) {
    handleApi(req, res, url).catch((err) => json(res, 500, { error: err.message }));
  } else {
    serveStatic(req, res, url);
  }
});

server.listen(PORT, () => {
  console.log(`室内点云重建平台已启动: http://localhost:${PORT}`);
  console.log(`静态目录: ${WEB_ROOT}`);
  console.log(`数据目录: ${DATA_DIR}（若为空请先 npm run gen-data）`);
});
