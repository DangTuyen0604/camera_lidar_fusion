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
  ament_add_pytest_test(test_topic_contract test/test_topic_contract.py)
  ament_add_pytest_test(test_tf_tree test/test_tf_tree.py)
  ament_add_pytest_test(test_calibration_drift test/test_calibration_drift.py)
  ament_add_pytest_test(test_fusion_output test/test_fusion_output.py)
  ament_add_pytest_test(
    test_navigation_pipeline
    test/test_navigation_pipeline.py
  )
  ament_add_pytest_test(test_warehouse_day5 test/test_warehouse_day5.py)
  ament_add_pytest_test(
    test_navigation_bridge_pipeline
    test/test_navigation_bridge_pipeline.py
  )
  ament_add_pytest_test(test_dynamic_replanning test/test_dynamic_replanning.py)
  ament_add_pytest_test(test_person_stop_resume test/test_person_stop_resume.py)
  ament_add_pytest_test(test_box_avoidance test/test_box_avoidance.py)
  ament_add_pytest_test(test_docking_pipeline test/test_docking_pipeline.py)
  ament_add_pytest_test(test_multi_station_mission test/test_multi_station_mission.py)
  ament_add_pytest_test(test_node_restart test/test_node_restart.py)
  ament_add_pytest_test(test_rosbag_replay test/test_rosbag_replay.py)
  ament_add_pytest_test(test_clean_shutdown test/test_clean_shutdown.py)
  ament_add_pytest_test(test_fault_injection test/test_fault_injection.py)
  add_launch_test(test/test_perception_pipeline.py TIMEOUT 90)
  add_launch_test(test/test_warehouse_end_to_end.py TIMEOUT 600)
endif()

ament_package()
