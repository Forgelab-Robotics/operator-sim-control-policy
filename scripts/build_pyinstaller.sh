#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_ENV="${ROOT}/.venv-build"
WORK_DIR="${ROOT}/build/pyinstaller"
DIST_DIR="${ROOT}/dist"
SPEC_FILE="${SCRIPT_DIR}/sim_control_policy.spec"
POLICY_BINARY="${DIST_DIR}/sim_control_policy"

cleanup() {
  rm -rf "${BUILD_ENV}"
}
trap cleanup EXIT

cd "${ROOT}"
rm -rf "${BUILD_ENV}" "${WORK_DIR}"
rm -f "${POLICY_BINARY}"
mkdir -p "${DIST_DIR}" "${WORK_DIR}"

echo "==> 创建隔离 Python 3.12 构建环境..."
uv venv --python 3.12 --clear "${BUILD_ENV}"
UV_PROJECT_ENVIRONMENT="${BUILD_ENV}" uv sync \
  --frozen \
  --no-dev \
  --group build

PYTHON_VERSION="$("${BUILD_ENV}/bin/python" -c \
  'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${PYTHON_VERSION}" != "3.12" ]]; then
  echo "ERROR: 构建环境不是 Python 3.12（实际为 ${PYTHON_VERSION}）" >&2
  exit 1
fi

echo "==> 构建 PyInstaller 单文件..."
PYINSTALLER_CONFIG_DIR="${WORK_DIR}/config" \
  "${BUILD_ENV}/bin/pyinstaller" \
  --noconfirm \
  --clean \
  --distpath "${DIST_DIR}" \
  --workpath "${WORK_DIR}" \
  "${SPEC_FILE}"

if [[ ! -x "${POLICY_BINARY}" ]]; then
  echo "ERROR: 未生成可执行文件：${POLICY_BINARY}" >&2
  exit 1
fi

echo "==> 验证参数解析与冻结依赖导入..."
timeout 30 "${POLICY_BINARY}" --help >/dev/null

echo "==> 构建及 smoke 验证成功"
printf '%s\n' "${POLICY_BINARY}"
