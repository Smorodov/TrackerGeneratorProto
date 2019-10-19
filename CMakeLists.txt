# ----------------------------------------------------------------------------
# минимальная версия CMake, способная переварить этот файл
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}) 

set(project_name TrackerGeneratorProto)

project(${project_name})

find_package(OpenCV)
message("Opencv Version: ${OPENCV_VERSION}" )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std:c++17")
#----------------------------------------------------------------------------  
# ищем все .cpp и .h файлы и добавляем их в наш проект 
# ----------------------------------------------------------------------------
SET(SOURCE
src/main.cpp
)

SET(HEADERS

)

include(CmakeIncludes/Common.txt)
include(CmakeIncludes/MainThread.txt)
include(CmakeIncludes/InputCamera.txt)
include(CmakeIncludes/OutputWindow.txt)
include(CmakeIncludes/DetectorFace.txt)
include(CmakeIncludes/FilterKalman.txt)
include(CmakeIncludes/Hungarian.txt)
include(CmakeIncludes/FilterHungarianKalman.txt)

SOURCE_GROUP("Source Files" FILES ${SOURCE})
SOURCE_GROUP("Header Files" FILES ${HEADERS})

set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_STATIC_RUNTIME   ON)
find_package(Boost 1.54 REQUIRED COMPONENTS system thread filesystem regex)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
get_directory_property(_my_link_dirs LINK_DIRECTORIES)
message(STATUS "_my_link_dirs = ${_my_link_dirs}") 
# ----------------------------------------------------------------------------   
# добавляем include директории
# ----------------------------------------------------------------------------
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
# ----------------------------------------------------------------------------  
# создаем проект
# ----------------------------------------------------------------------------
  ADD_EXECUTABLE( ${project_name}
  				  ${SOURCE}
  				  ${HEADERS}
                  ${MAIN_THREAD_SOURCE}
  				  ${MAIN_THREAD_HEADERS}
                  ${COMMON_SOURCE}
                  ${COMMON_HEADER}
                  ${INPUT_SOURCE}
                  ${INPUT_HEADER}
                  ${DETECTOR_SOURCE}
                  ${DETECTOR_HEADER}
                  ${TRACKER_SOURCE}
                  ${TRACKER_HEADER}
                  ${OUTPUT_SOURCE}
                  ${OUTPUT_HEADER}
  				  )
# ----------------------------------------------------------------------------
# Lib-ы
# ----------------------------------------------------------------------------
TARGET_LINK_LIBRARIES(${project_name} ${OpenCV_LIBS} )
