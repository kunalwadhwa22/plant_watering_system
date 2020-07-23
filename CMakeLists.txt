# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS $ENV{PROJECT_PATH}/../bwesp32/components $ENV{PROJECT_PATH}/../bwhal/src/bwcommon $ENV{PROJECT_PATH}/../bwhal/src/bwdriver $ENV{PROJECT_PATH}/../bwhal/src/bwlog $ENV{PROJECT_PATH}/../bwhal/module/bwgpio/gpioexpanders) $ENV{PROJECT_PATH}/../bwhal/module/bwled/bwled)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(plant-watering-system)
