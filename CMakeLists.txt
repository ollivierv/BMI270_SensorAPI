
if(CONFIG_IMU_BMI270)
  #FILE(GLOB app_sources *.c)
  #target_sources(app PRIVATE ${app_sources})

  target_sources(app PRIVATE
    bmi2.c
    bmi270_dsd.c
    bmi270_DSD_examples/common/zephyr_common.c
    bmi270_DSD_examples/door_state_detector/door_state_detector.c
  )

endif()
