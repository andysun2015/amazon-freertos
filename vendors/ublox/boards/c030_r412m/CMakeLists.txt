set(stm32f767_dir "${AFR_VENDORS_DIR}/st/stm32cube_fw_f7/stm32cube_fw_f7_v1.15.0")
set(afr_ports_dir "${CMAKE_CURRENT_LIST_DIR}/ports")
set(board_demos_dir "${CMAKE_CURRENT_LIST_DIR}/aws_demos")
set(board_tests_dir "${CMAKE_CURRENT_LIST_DIR}/aws_tests")
if(AFR_IS_TESTING)
    set(board_dir "${board_tests_dir}")
else()
    set(board_dir "${board_demos_dir}")
endif()

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS Console metadata
# -------------------------------------------------------------------------------------------------
afr_set_board_metadata(ID "STM32-NUCLEO-F767ZI")
afr_set_board_metadata(DISPLAY_NAME "STM32F7 Nucleo Board")
afr_set_board_metadata(DESCRIPTION "NUCLEO-BF767ZI Nucleo board with ethernet")
afr_set_board_metadata(VENDOR_NAME "STMicroelectronics")
afr_set_board_metadata(FAMILY_NAME "STM32F7")
afr_set_board_metadata(IS_ACTIVE "TRUE")
afr_set_board_metadata(DATA_RAM_MEMORY "512KB")
afr_set_board_metadata(PROGRAM_MEMORY "2MB")
afr_set_board_metadata(CODE_SIGNER "AmazonFreeRTOS-Default")
afr_set_board_metadata(SUPPORTED_IDE "STM32CubeIDE")
afr_set_board_metadata(RECOMMENDED_IDE "STM32CubeIDE")
afr_set_board_metadata(IDE_STM32CubeIDE_NAME "STM32CubeIDE")
afr_set_board_metadata(IDE_STM32CubeIDE_COMPILER "GCC")

afr_set_board_metadata(IDE_STM32CubeIDE_PROJECT_LOCATION "${AFR_ROOT_DIR}/projects/st/nucleo_f767zi/stm32cubeide/aws_demos")
afr_set_board_metadata(AWS_DEMOS_CONFIG_FILES_LOCATION "${CMAKE_CURRENT_LIST_DIR}/aws_demos/config_files")

# -------------------------------------------------------------------------------------------------
# Compiler settings
# -------------------------------------------------------------------------------------------------
afr_mcu_port(compiler)
set(
    defined_symbols
    USE_HAL_DRIVER
    STM32F767xx
    MQTTCLIENT_PLATFORM_HEADER=MQTTCMSIS.h
    ENABLE_IOT_INFO
    ENABLE_IOT_ERROR
)
target_compile_definitions(
    AFR::compiler::mcu_port
    INTERFACE $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:${defined_symbols}>
)

set(common_flags "-mcpu=cortex-m7" "-mthumb" "-mfloat-abi=hard" "-mfpu=fpv5-d16")
set(c_flags "-Wall" "-fmessage-length=0" "-ffunction-sections")
set(linker_flags "-specs=nosys.specs" "-Wl,-Map=output.map,--gc-sections,-zmuldefs" "-lm")

# Compiler flags
target_compile_options(
    AFR::compiler::mcu_port
    INTERFACE
        ${common_flags}
        $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:${c_flags}>
)

# Linker flags
target_link_options(
    AFR::compiler::mcu_port
    INTERFACE ${common_flags} ${linker_flags}
)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS portable layers
# -------------------------------------------------------------------------------------------------
# Kernel
afr_mcu_port(kernel)
afr_glob_src(driver_src DIRECTORY "${stm32f767_dir}" RECURSE)
target_sources(
    AFR::kernel::mcu_port
    INTERFACE
        ${driver_src}
        "${AFR_KERNEL_DIR}/portable/GCC/ARM_CM7/port.c"
        "${AFR_KERNEL_DIR}/portable/GCC/ARM_CM7/portmacro.h"
        "${AFR_KERNEL_DIR}/portable/MemMang/$<IF:${AFR_IS_TESTING},heap_4.c,heap_5.c>"
)
set(
    kernel_inc_dirs
    "${AFR_KERNEL_DIR}/portable/GCC/ARM_CM7"
    "${stm32f767_dir}/BSP/STM32F7xx_Nucleo_144"
    "${stm32f767_dir}/BSP/Components/Common"
    "${stm32f767_dir}/BSP/Components/lan8742"
    "${stm32f767_dir}/CMSIS/Include"
    "${stm32f767_dir}/CMSIS/Device/ST/STM32F7xx/Include"
    "${stm32f767_dir}/STM32F7xx_HAL_Driver/Inc"
    "${stm32f767_dir}/STM32F7xx_HAL_Driver/Inc/Legacy"
    "${board_dir}/application_code"
    "${board_dir}/application_code/st_code"
    "$<IF:${AFR_IS_TESTING},${AFR_TESTS_DIR},${AFR_DEMOS_DIR}>/include"
)

target_include_directories(
    AFR::kernel::mcu_port
    INTERFACE
        $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:${kernel_inc_dirs}>
        "${board_dir}/config_files"
)

# WiFi
#afr_mcu_port(wifi)
#target_sources(
#    AFR::wifi::mcu_port
#    INTERFACE "${afr_ports_dir}/wifi/aws_wifi.c"
#)

# PKCS11
afr_mcu_port(pkcs11)
target_sources(
    AFR::pkcs11::mcu_port
    INTERFACE "${afr_ports_dir}/pkcs11/aws_pkcs11_pal.c"
)

# Link to AFR::pkcs11_mbedtls if you want to use default implementation based on mbedtls.
target_link_libraries(
    AFR::pkcs11::mcu_port
    INTERFACE
        AFR::pkcs11_mbedtls
#        AFR::wifi
)

# Secure sockets
afr_mcu_port(secure_sockets)
target_link_libraries(
    AFR::secure_sockets::mcu_port
    INTERFACE
        AFR::tls
#        AFR::wifi
)
target_sources(
    AFR::secure_sockets::mcu_port
    INTERFACE "${afr_ports_dir}/secure_sockets/aws_secure_sockets.c"
)

# -------------------------------------------------------------------------------------------------
# Amazon FreeRTOS demos and tests
# -------------------------------------------------------------------------------------------------
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

if(AFR_IS_TESTING)
    set(exe_target aws_tests)
else()
    set(exe_target aws_demos)
endif()

# TODO, remove network manager src.
afr_glob_src(network_manager_src DIRECTORY "${AFR_DEMOS_DIR}/network_manager")
afr_glob_src(board_code_src DIRECTORY "${board_dir}/application_code/st_code")
afr_glob_src(config_files DIRECTORY "${board_dir}/config_files")

if(AFR_IS_TESTING)
add_executable(
    ${exe_target}
    ${board_code_src}
    ${config_files}
    "${board_dir}/application_code/main.c"
)
else()
add_executable(
    ${exe_target}
    ${board_code_src}
    ${config_files}
    ${network_manager_src}
    "${board_dir}/application_code/main.c"
)
endif()

target_link_libraries(
    ${exe_target}
    PRIVATE
#        AFR::wifi
        AFR::utils
        -T"${board_dir}/STM32F767ZITx_FLASH.ld"
)

if(AFR_METADATA_MODE)
    return()
endif()

# -------------------------------------------------------------------------------------------------
# Additional build configurations
# -------------------------------------------------------------------------------------------------

find_program(gcc_objcopy arm-none-eabi-objcopy)
find_program(gcc_size arm-none-eabi-size)
if(NOT gcc_objcopy)
    message(FATAL_ERROR "Cannot find arm-none-eabi-objcopy.")
endif()

set(output_file "$<TARGET_FILE_DIR:${exe_target}>/${exe_target}.hex")
add_custom_command(
    TARGET ${exe_target} POST_BUILD
    COMMAND "${gcc_objcopy}" -O ihex "$<TARGET_FILE:${exe_target}>" "${output_file}"
    COMMAND "${gcc_size}" "$<TARGET_FILE:${exe_target}>"
)
add_custom_command(
    TARGET ${exe_target} POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy "${output_file}" "${CMAKE_BINARY_DIR}"
)
