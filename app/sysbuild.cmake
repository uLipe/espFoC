# Add external project
ExternalZephyrProject_Add(
    APPLICATION espfoc_motor_remote
    SOURCE_DIR ${APP_DIR}/espfoc_motor
    BOARD ${SB_CONFIG_ESP_FOC_REMOTE_BOARD}
  )

add_dependencies(app espfoc_motor_remote)
sysbuild_add_dependencies(CONFIGURE app espfoc_motor_remote)

if(SB_CONFIG_BOOTLOADER_MCUBOOT)
  # Make sure MCUboot is flashed first
  sysbuild_add_dependencies(FLASH espfoc_motor_remote mcuboot)
endif()
