.PHONY: audit benchmark build calibration demo navigation perception projection test warehouse

audit:
	./scripts/run_audit.sh

build:
	./scripts/build_workspace.sh

test:
	./scripts/run_tests.sh

projection:
	./scripts/run_projection_demo.sh

perception:
	./scripts/run_perception_demo.sh

calibration:
	./scripts/run_calibration_demo.sh

navigation:
	./scripts/run_navigation_demo.sh

warehouse:
	./scripts/run_warehouse_demo.sh

benchmark:
	./scripts/run_benchmarks.sh

demo:
	./scripts/run_release_demo.sh
