// 3x3 对称矩阵特征分解（Jacobi 循环），输出升序特征值与对应特征向量
// 输入为完整 3x3 数组（只需对称部分）
export function symEig3(Ain) {
  const a = Ain.map((r) => r.slice());
  const v = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

  for (let sweep = 0; sweep < 50; sweep++) {
    const off = a[0][1] ** 2 + a[0][2] ** 2 + a[1][2] ** 2;
    if (off < 1e-26) break;
    jacobiRotate(a, v, 0, 1);
    jacobiRotate(a, v, 0, 2);
    jacobiRotate(a, v, 1, 2);
  }

  const vals = [a[0][0], a[1][1], a[2][2]];
  const order = [0, 1, 2].sort((x, y) => vals[x] - vals[y]);
  return {
    values: order.map((k) => vals[k]),
    vectors: order.map((k) => [v[0][k], v[1][k], v[2][k]]),
  };
}

function jacobiRotate(a, v, p, q) {
  if (Math.abs(a[p][q]) < 1e-18) return;
  const tau = (a[q][q] - a[p][p]) / (2 * a[p][q]);
  let t;
  if (tau >= 0) t = 1 / (tau + Math.sqrt(1 + tau * tau));
  else t = -1 / (-tau + Math.sqrt(1 + tau * tau));
  const c = 1 / Math.sqrt(1 + t * t);
  const s = t * c;

  const app = a[p][p], aqq = a[q][q], apq = a[p][q];
  a[p][p] = app - t * apq;
  a[q][q] = aqq + t * apq;
  a[p][q] = 0;
  a[q][p] = 0;
  for (let k = 0; k < 3; k++) {
    if (k === p || k === q) continue;
    const akp = a[k][p], akq = a[k][q];
    a[k][p] = c * akp - s * akq; a[p][k] = a[k][p];
    a[k][q] = s * akp + c * akq; a[q][k] = a[k][q];
  }
  for (let k = 0; k < 3; k++) {
    const vkp = v[k][p], vkq = v[k][q];
    v[k][p] = c * vkp - s * vkq;
    v[k][q] = s * vkp + c * vkq;
  }
}
