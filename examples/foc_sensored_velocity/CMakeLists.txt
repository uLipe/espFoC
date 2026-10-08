cmake_minimum_required(VERSION 3.16)

# Builds against the espFoC checkout this example lives in. In your own
# project, add espFoC as a component (components/ or the component manager)
# and drop this line.
set(EXTRA_COMPONENT_DIRS "../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(foc_sensored_velocity)
