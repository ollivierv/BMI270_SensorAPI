
if(CONFIG_IMU_BMI270)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  #include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bmi270_DSD_examples/common)

  target_sources(app PRIVATE
    bmi2.c
    bmi270_dsd.c
    bmi270_DSD_examples/common/zephyr_common.c
    bmi270_DSD_examples/door_state_detector/door_state_detector.c
  )

endif()
