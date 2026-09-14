.PHONY: build test projection perception calibration navigation

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
