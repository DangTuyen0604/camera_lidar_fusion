#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
release_tag=v1.0.0

[[ -z "$(git status --porcelain)" ]] || {
  echo 'Release refused: working tree is not clean.' >&2
  exit 1
}
git rev-parse "$release_tag" >/dev/null 2>&1 && {
  echo "Release refused: tag $release_tag already exists." >&2
  exit 1
}
[[ -n "${DEMO_VIDEO_URL:-}" ]] || {
  echo 'Release refused: DEMO_VIDEO_URL is not set.' >&2
  exit 1
}

required=(
  benchmark_results/raw_results.csv
  benchmark_results/summary.csv
  benchmark_results/latency.png
  benchmark_results/fusion_success_rate.png
  benchmark_results/calibration_quality.png
  benchmark_results/delay_sensitivity.png
  benchmark_results/noise_sensitivity.png
  benchmark_results/mission_duration.png
  benchmark_results/resource_usage.png
  docs/images/architecture.png
  docs/images/warehouse.png
  docs/images/fused_detection.png
  docs/images/costmap_obstacle.png
  docs/images/pallet_replan.png
  docs/images/person_stop.png
  docs/images/box_avoidance.png
  docs/images/docking.png
)
for artifact in "${required[@]}"; do
  [[ -s "$artifact" ]] || { echo "Release refused: missing $artifact" >&2; exit 1; }
done

./scripts/run_audit.sh --full
./docker/validate_runtime.sh

if command -v gh >/dev/null; then
  gh run list --branch "$(git branch --show-current)" --limit 1 \
    --json conclusion --jq '.[0].conclusion' | grep -Fxq success || {
      echo 'Release refused: latest GitHub Actions run is not successful.' >&2
      exit 1
    }
else
  echo 'Release refused: gh CLI is required to verify CI.' >&2
  exit 1
fi

if [[ "${1:-}" == '--tag' ]]; then
  git tag -a "$release_tag" \
    -m 'Complete warehouse camera-LiDAR fusion navigation system'
  echo "Created $release_tag locally. Review it before pushing branch and tag."
else
  echo "All release gates passed. Re-run with --tag to create $release_tag locally."
fi
