# SPDX-License-Identifier: Apache-2.0

# This is a general purpose project created from MotionSenseHRV to be used for data collection.

# Ohio State SENSE LAB

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(external_lib)


# this is a defined k-config
add_subdirectory_ifdef(CONFIG_CMSIS_DSP        src/DSP)
target_sources(app PRIVATE src/main.cc)
target_sources(app PRIVATE src/ppgSensor.cc)
target_sources(app PRIVATE src/batteryMonitor.cc)
target_sources(app PRIVATE src/imuSensor.cc)
target_sources(app PRIVATE src/orientation.cc)
include_directories(src/DSP/PrivateInclude)
target_sources(app PRIVATE src/constants.cc)
target_sources(app PRIVATE src/output_handler.cc)
target_sources(app PRIVATE src/assert.cc)

target_sources(app PRIVATE src/BLEService.cc)

# Other external build systems may be self-contained enough that they
# do not need any build information from zephyr. Or they may be
# incompatible with certain zephyr options and need them to be
# filtered out.

# set(TARGET ${BOARD})
# set(TARGET_ARCH cortex-m33)




set(extra_project_flags "-mcpu=${TARGET_ARCH} -mthumb -mno-thumb-interwork -mhard-float -mfpu=fpv5-sp-d16") #I had to remove -DTF_LITE_STATIC_MEMORY to make hello_world run. 

zephyr_get_include_directories_for_lang_as_string(       C C_includes)
zephyr_get_system_include_directories_for_lang_as_string(C C_system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       C C_definitions)
zephyr_get_compile_options_for_lang_as_string(           C C_options)

set(external_project_cflags
  "${C_includes} ${C_definitions} ${optC_optionsions} ${C_system_includes} ${extra_project_flags}"
)

zephyr_get_include_directories_for_lang_as_string(       CXX CXX_includes)
zephyr_get_system_include_directories_for_lang_as_string(CXX CXX_system_includes)
zephyr_get_compile_definitions_for_lang_as_string(       CXX CXX_definitions)
zephyr_get_compile_options_for_lang_as_string(           CXX CXX_options)










