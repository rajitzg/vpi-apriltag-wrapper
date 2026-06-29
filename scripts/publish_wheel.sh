#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This script expects an aarch64 Jetson host with VPI 3.x installed." >&2
  exit 1
fi

python3.10 -m pip install --upgrade pip build twine scikit-build-core pybind11 numpy
python3.10 -m build --wheel

if [[ -n "${CI_JOB_TOKEN:-}" && -n "${CI_PROJECT_ID:-}" ]]; then
twine upload \
  --repository-url "https://gitlab.com/api/v4/groups/9205524/-/packages/pypi" \
  -u gitlab-ci-token \
  -p "${CI_JOB_TOKEN}" \
  dist/*.whl
else
  echo "Built wheel(s) in dist/. Set CI_JOB_TOKEN and CI_PROJECT_ID to publish automatically."
  ls -la dist/
fi
