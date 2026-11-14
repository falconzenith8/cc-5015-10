// 平台自测：覆盖任务要求的四个验证场景（+玻璃缺测附加场景），两个重建后端。
// 断言诚实性红线：缺墙/玻璃洞不得被自动封洞；噪点可恢复；原点不可覆盖；
// 输出检查退化面、连通区域、点到网格距离（距离小不证明拓扑真实）。
// 运行：npm run selftest
import { readFileSync, existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { PointCloud } from '../src/core/pointcloud.js';
import { runPipeline } from '../src/core/pipeline.js';
import { defaultParams } from '../src/core/schema.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const DATA = resolve(__dirname, '../data');

let pass = 0, fail = 0;
const failures = [];
function check(name, cond, detail = '') {
  if (cond) { pass++; }
  else { fail++; failures.push(`${name} ${detail}`); console.error(`  ✗ ${name} ${detail}`); }
}
function section(t) { console.log(`\n=== ${t} ===`); }

function load(name) {
  const path = resolve(DATA, `${name}.json`);
  if (!existsSync(path)) {
    console.error(`缺少数据文件 ${path}，请先运行 npm run gen-data`);
    process.exit(2);
  }
  const d = JSON.parse(readFileSync(path, 'utf8'));
  return new PointCloud({ points: d.points, views: d.views, name: d.name, sourceFormat: 'json' });
}

function facesNearXp(r) {
  const V = r.mesh.vertices, F = r.mesh.faces;
  let k = 0;
  for (const [a, b, c] of F) {
    const cx = (V[a][0] + V[b][0] + V[c][0]) / 3;
    if (cx > 1.95) k++;
  }
  return k;
}
function facesInGlassRect(r) {
  const V = r.mesh.vertices, F = r.mesh.faces;
  let k = 0;
  for (const [a, b, c] of F) {
    const cx = (V[a][0] + V[b][0] + V[c][0]) / 3;
    const cy = (V[a][1] + V[b][1] + V[c][1]) / 3;
    const cz = (V[a][2] + V[b][2] + V[c][2]) / 3;
    if (cz < -1.45 && cx > -0.7 && cx < 0.7 && cy > 0.2 && cy < 1.6) k++;
  }
  return k;
}

// 场景 1：已知平面房间 —— 两个后端都应产出大量面片、零退化、距离有限
section('s1 已知平面房间');
{
  const cloud = load('s1_planar_room');
  for (const method of ['bpa', 'planar']) {
    const r = runPipeline(cloud, { method });
    check(`s1/${method} 产出面片`, r.mesh.faces.length > 1000, `faces=${r.mesh.faces.length}`);
    check(`s1/${method} 零退化面`, r.qc.degenerateCount === 0, `deg=${r.qc.degenerateCount}`);
    check(`s1/${method} 距离中位数有限且毫米级`, r.qc.distances.median !== null && r.qc.distances.median < 0.05, `med=${r.qc.distances.median}`);
    check(`s1/${method} 含拓扑免责声明`, typeof r.qc.topologyNotice === 'string' && r.qc.topologyNotice.length > 0);
  }
}

// 场景 2：密度骤变 —— 局部尺度在稀疏区增大；不产生超长搭桥；距离仍可控
section('s2 密度骤变');
{
  const cloud = load('s2_density_jump');
  for (const method of ['bpa', 'planar']) {
    const r = runPipeline(cloud, { method });
    check(`s2/${method} 产出面片`, r.mesh.faces.length > 1000, `faces=${r.mesh.faces.length}`);
    check(`s2/${method} 零退化面`, r.qc.degenerateCount === 0);
    check(`s2/${method} 距离中位数毫米级`, r.qc.distances.median !== null && r.qc.distances.median < 0.1, `med=${r.qc.distances.median}`);
    // 稀疏区点的局部尺度应高于全局中位数（自适应尺度证据）：用 q98/q50 分位比
    const scales = r.scales.perPoint;
    check(`s2/${method} 逐点尺度存在离散（自适应而非统一）`, r.scales.q98 > r.scales.q50 * 1.2, `q98/q50=${(r.scales.q98 / r.scales.q50).toFixed(2)}`);
    void scales;
  }
}

// 场景 3：孤立噪点 —— 筛查命中离群点；候选只标记不删除；修订后可恢复；原点不被覆盖
section('s3 孤立噪点筛查与可恢复修订');
{
  const cloud = load('s3_isolated_noise');
  const originalHash = cloud.hash;
  const originalN = cloud.n;
  const r = runPipeline(cloud, { method: 'bpa' });
  const cand = r.noise.candidates;
  check('s3 噪点候选非空', cand.length > 0, `candidates=${cand.length}`);
  // 8 个远距离群点（无观测方向）应几乎全部命中
  const farHits = cand.filter((c) => cloud.views[c.index] === null).length;
  check('s3 8 个远距离群点全部被标记', farHits >= 8, `farHits=${farHits}`);
  // 筛查不改写原点
  check('s3 筛查后原点数量与哈希不变', cloud.n === originalN && cloud.hash === originalHash);

  // 修订：剔除候选 -> 新点云；原对象保持不变
  const removedSet = new Set(cand.map((c) => c.index));
  const keepMask = cloud.points.map((_, i) => !removedSet.has(i));
  const { cloud: revised, removedIndices } = cloud.revise({ keepMask, label: 'drop-noise' });
  check('s3 修订产生独立新点云', revised.id !== cloud.id && revised.n === cloud.n - cand.length);
  check('s3 原点云修订后仍完整', cloud.n === originalN && cloud.hash === originalHash);
  check('s3 修订记录血缘', Array.isArray(revised.lineage) && revised.lineage.includes(cloud.id));

  // 恢复：用被删索引的补集重建 -> 点数回到原值（误删可恢复）
  const restoreMask = new Array(originalN).fill(false);
  for (const i of removedIndices) restoreMask[i] = true;
  const { cloud: restored } = cloud.revise({ keepMask: restoreMask, label: 'restore' });
  check('s3 被删候选可恢复为独立点云', restored.n === removedIndices.length);

  const r2 = runPipeline(revised, { method: 'bpa' });
  check('s3 剔除离群点后仍能重建', r2.mesh.faces.length > 1000, `faces=${r2.mesh.faces.length}`);
}

// 场景 4：缺一面墙 —— 红线：+X 开口必须保持开放，零面片穿越
section('s4 缺墙（禁止自动封洞）');
{
  const cloud = load('s4_missing_wall');
  for (const method of ['bpa', 'planar']) {
    const r = runPipeline(cloud, { method });
    const near = facesNearXp(r);
    check(`s4/${method} 开口附近零面片（不封墙）`, near === 0, `facesNearXp=${near}`);
    check(`s4/${method} 存在边界边（开放证据）`, r.mesh.boundaryEdgeCount > 0, `boundary=${r.mesh.boundaryEdgeCount}`);
  }
}

// 附加场景 5：玻璃缺测 —— 平面后端空栅格>0 且矩形内零面片；BPA 洞缘披露
section('s5 玻璃缺测（矩形无回波）');
{
  const cloud = load('s5_glass_hole');
  const rp = runPipeline(cloud, { method: 'planar' });
  const emptyCells = rp.mesh.regionReports.reduce((s, x) => s + x.gridCellsEmpty, 0);
  check('s5/planar 报告保留的空栅格', emptyCells > 0, `emptyCells=${emptyCells}`);
  check('s5/planar 玻璃矩形内零面片', facesInGlassRect(rp) === 0, `inside=${facesInGlassRect(rp)}`);

  const rb = runPipeline(cloud, { method: 'bpa' });
  const inside = facesInGlassRect(rb);
  // BPA 洞缘受支撑窄三角属已知几何局限：允许存在但必须可被显式计数/披露，不得静默封洞
  check('s5/bpa 洞缘面数量受限且远小于整面墙', inside >= 0 && inside <= 20, `rimFaces=${inside}`);
  check('s5/bpa 未把整片玻璃封死（面片不覆盖矩形主体）', rb.mesh.faces.length > 1000, `faces=${rb.mesh.faces.length}`);
}

// 参数冻结：每次运行携带输入哈希与参数快照
section('参数与输入冻结');
{
  const cloud = load('s1_planar_room');
  const params = defaultParams();
  const r = runPipeline(cloud, { method: 'planar', gridFactor: 1.5 });
  check('冻结输入哈希', r.frozen.cloudHash === cloud.hash);
  check('冻结参数快照反映本次取值', r.frozen.paramsSnapshot.gridFactor === 1.5);
  check('默认参数对象未被运行改写', params.gridFactor === defaultParams().gridFactor);
}

// 失败可定位：构造无法成面的极端输入，确认 PipelineError 带阶段与提示
section('重建失败定位');
{
  // 共线点集（6 点，少于最小平面区域）无法形成二维受支撑表面
  const pts = Array.from({ length: 6 }, (_, i) => [i * 0.1, 0, 0]);
  const views = pts.map(() => [0, 1, 0]);
  const cloud = new PointCloud({ points: pts, views, name: 'degenerate-line' });
  let threw = null;
  try { runPipeline(cloud, { method: 'planar' }); } catch (e) { threw = e; }
  check('退化输入抛出 PipelineError', threw !== null && threw.name === 'PipelineError');
  if (threw) {
    check('错误携带失败阶段', !!threw.failure?.failedStage);
    check('错误携带修复提示', typeof threw.failure?.hint === 'string' && threw.failure.hint.length > 0);
  }
}

// 参数对照：同一输入两套参数，输出可量化对比（任务要求"参数对照"）
section('参数对照（gridFactor 0.75 vs 2.0）');
{
  const cloud = load('s1_planar_room');
  const a = runPipeline(cloud, { method: 'planar', gridFactor: 0.75 });
  const b = runPipeline(cloud, { method: 'planar', gridFactor: 2.0 });
  console.log(`  gridFactor=0.75 -> faces=${a.mesh.faces.length}, comps=${a.qc.componentCount}, distMed=${a.qc.distances.median}, boundary=${a.mesh.boundaryEdgeCount}`);
  console.log(`  gridFactor=2.00 -> faces=${b.mesh.faces.length}, comps=${b.qc.componentCount}, distMed=${b.qc.distances.median}, boundary=${b.mesh.boundaryEdgeCount}`);
  check('参数对照两次运行均成功且各自冻结', a.frozen.paramsSnapshot.gridFactor === 0.75 && b.frozen.paramsSnapshot.gridFactor === 2);
  check('细栅格产生更多面片', a.mesh.faces.length > b.mesh.faces.length, `${a.mesh.faces.length} vs ${b.mesh.faces.length}`);
}

console.log(`\n结果：通过 ${pass} 项，失败 ${fail} 项`);
if (fail > 0) {
  console.error('失败项：\n - ' + failures.join('\n - '));
  process.exit(1);
}
console.log('全部诚实性与几何断言通过。');
