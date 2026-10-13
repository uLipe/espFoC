cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# IDF 6 registers chip-only drivers that fail to resolve on the host target,
# so the host build pulls in only what main and the test components need.
if("${IDF_TARGET}" STREQUAL "linux")
    idf_build_set_property(MINIMAL_BUILD ON)
endif()
project(espfoc_unit_test_runner)
