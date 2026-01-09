
if(CONFIG_IMU_BMI270)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  #include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bmi270_DSD_examples/common)

  # target_sources(app PRIVATE
  #   bmi2.c
  #   bmi270_dsd.c
  #   bmi270_DSD_examples/common/zephyr_common.c
  #   bmi270_DSD_examples/door_state_detector/door_state_detector.c
  #   bmi270_DSD_examples/door_state_detector_hw_int/door_state_detector_hw_int.c
  # )



  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bmi270_examples/accel_gyro)
  target_sources(app PRIVATE
    bmi2.c
    bmi270.c
    bmi270_examples/common/zephyr_common.c
    bmi270_examples/accel_gyro/accel_gyro_custom_door.c
  )


endif()
