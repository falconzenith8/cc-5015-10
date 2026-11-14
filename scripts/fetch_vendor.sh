#!/usr/bin/env bash
# 下载 Three.js r160 ESM 模块到本地 web/static/vendor，使网页离线可用、无构建步骤。
# 下载内容属于第三方不可信数据，单独存放于 vendor 目录，不与项目源码混放。
set -euo pipefail

VERSION="0.160.0"
BASE="https://unpkg.com/three@${VERSION}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${HERE}/../web/static/vendor"

mkdir -p "${OUT}/addons/controls"

echo "下载 three.module.js (r${VERSION}) ..."
curl -fsSL "${BASE}/build/three.module.js" -o "${OUT}/three.module.js"

echo "下载 OrbitControls.js ..."
curl -fsSL "${BASE}/examples/jsm/controls/OrbitControls.js" -o "${OUT}/addons/controls/OrbitControls.js"

# OrbitControls 内部以裸说明符 'three' 导入；改写为相对路径，配合 importmap 也可直接引用
sed -i "s|from 'three'|from '../../three.module.js'|g" "${OUT}/addons/controls/OrbitControls.js"

# 校验
test -s "${OUT}/three.module.js"
test -s "${OUT}/addons/controls/OrbitControls.js"
grep -q "three.module.js" "${OUT}/addons/controls/OrbitControls.js"

echo "完成："
ls -1 "${OUT}/three.module.js" "${OUT}/addons/controls/OrbitControls.js"
