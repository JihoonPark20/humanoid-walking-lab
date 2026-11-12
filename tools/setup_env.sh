#!/usr/bin/env bash
# Install CPU dependencies and the robot assets used by the walking demos.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
MODEL_REVISION="370c43965323c0944b433b202aac5058b5f907a9"
MODEL_DIR="${REPO_ROOT}/third_party/mujoco_menagerie"
ARM_MODELS="${REPO_ROOT}/lab-2-Ur5e-robotics-lab/models"

if [[ "${1:-}" != "--models-only" && -n "${1:-}" ]]; then
  echo "Usage: bash tools/setup_env.sh [--models-only]" >&2
  exit 2
fi
if [[ "${1:-}" != "--models-only" ]]; then
  "${PYTHON_BIN}" -m pip install -r "${REPO_ROOT}/requirements.txt"
fi
if [[ ! -d "${MODEL_DIR}/.git" ]]; then
  if [[ -e "${MODEL_DIR}" ]]; then
    echo "Existing model directory is not a Git checkout: ${MODEL_DIR}" >&2
    exit 1
  fi
  mkdir -p "${MODEL_DIR}"
  git -C "${MODEL_DIR}" init -q
  git -C "${MODEL_DIR}" remote add origin https://github.com/google-deepmind/mujoco_menagerie.git
fi
if [[ -n "$(git -C "${MODEL_DIR}" status --porcelain)" ]]; then
  echo "Model checkout has local changes; preserve them before rerunning setup." >&2
  exit 1
fi
if ! git -C "${MODEL_DIR}" cat-file -e "${MODEL_REVISION}^{commit}" 2>/dev/null; then
  git -C "${MODEL_DIR}" fetch --filter=blob:none --depth 1 origin "${MODEL_REVISION}"
fi
git -C "${MODEL_DIR}" sparse-checkout init --cone
git -C "${MODEL_DIR}" sparse-checkout set unitree_g1 universal_robots_ur5e robotiq_2f85
git -C "${MODEL_DIR}" checkout --detach "${MODEL_REVISION}"

# Reuse one checkout. Do not replace an independent user-owned model directory.
if [[ ! -e "${ARM_MODELS}/mujoco_menagerie" && ! -L "${ARM_MODELS}/mujoco_menagerie" ]]; then
  ln -s ../../third_party/mujoco_menagerie "${ARM_MODELS}/mujoco_menagerie"
fi
if [[ "$(cd "${ARM_MODELS}/mujoco_menagerie" && pwd -P)" != "$(cd "${MODEL_DIR}" && pwd -P)" ]]; then
  echo "An independent UR5e model checkout exists; align it with ${MODEL_REVISION}." >&2
  exit 1
fi
for model in unitree_g1 universal_robots_ur5e robotiq_2f85; do
  test -f "${MODEL_DIR}/${model}/LICENSE"
done
for mapping in 'assets:mujoco_menagerie/universal_robots_ur5e/assets' 'scene_link.xml:lab_scene.xml'; do
  name="${mapping%%:*}"
  target="${mapping#*:}"
  if [[ ! -e "${ARM_MODELS}/${name}" && ! -L "${ARM_MODELS}/${name}" ]]; then
    ln -s "${target}" "${ARM_MODELS}/${name}"
  fi
done
echo "Models ready: ${MODEL_REVISION}"
echo "macOS passive viewer: mjpython; Linux headless rendering: MUJOCO_GL=egl"
