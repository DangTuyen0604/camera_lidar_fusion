cmake_minimum_required(VERSION 3.8)
project(system_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_topics test/test_topics.py)
  ament_add_pytest_test(test_calibration_drift test/test_calibration_drift.py)
  ament_add_pytest_test(
    test_navigation_pipeline
    test/test_navigation_pipeline.py
  )
  ament_add_pytest_test(
    test_stage3_safety_config
    test/test_stage3_safety_config.py
  )
  ament_add_pytest_test(
    test_gate12_14_artifacts
    test/test_gate12_14_artifacts.py
  )
  ament_add_pytest_test(
    test_gate11_pipeline
    test/test_gate11_pipeline.py
  )
  ament_add_pytest_test(test_fault_injection test/test_fault_injection.py)
  ament_add_pytest_test(
    test_license_provenance
    test/test_license_provenance.py
  )
  add_launch_test(test/test_perception_pipeline.py TIMEOUT 90)
  add_launch_test(test/test_canonical_launch_smoke.py TIMEOUT 180)
endif()

ament_package()
