# ----------------------------------------------------------------------------
# минимальная версия CMake, способная переварить этот файл
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.8)

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

file(GLOB ROOT_COMMON_HEADER src/Common/*.h*)
file(GLOB ROOT_COMMON_SOURCE src/Common/*.c*)
file(GLOB TRACK_HEADER src/Tracker/Common/Track/*.h*)
file(GLOB TRACK_SOURCE src/Tracker/Common/Track/*.c*)
file(GLOB TRACKER_COMMON_HEADER src/Tracker/Common/*.h*)
file(GLOB TRACKER_COMMON_SOURCE src/Tracker/Common/*.c*)
file(GLOB DETECTOR_COMMON_HEADER src/Detector/Common/*.h*)
file(GLOB DETECTOR_COMMON_SOURCE src/Detector/Common/*.c*)


set(COMMON_HEADER ${ROOT_COMMON_HEADER} 
                  ${TRACK_HEADER}
                  ${DETECTOR_COMMON_HEADER}
                  ${TRACKER_COMMON_HEADER}
                  )
set(COMMON_SOURCE ${ROOT_COMMON_SOURCE}
                  ${TRACK_SOURCE}
                  ${DETECTOR_COMMON_SOURCE}
                  ${TRACKER_COMMON_SOURCE}
                  )

SOURCE_GROUP("COMMON Source" FILES ${COMMON_SOURCE})
SOURCE_GROUP("COMMON Header" FILES ${COMMON_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Common)
include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common)
include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/Track)

file(GLOB MAIN_THREAD_HEADER src/MainThread/*.h*)
file(GLOB MAIN_THREAD_SOURCE src/MainThread/*.c*)
SOURCE_GROUP("MAIN_THREAD Source" FILES ${MAIN_THREAD_SOURCE})
SOURCE_GROUP("MAIN_THREAD Header" FILES ${MAIN_THREAD_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/MainThread)

file(GLOB INPUT_HEADER src/Input/CameraInput/*.h*)
file(GLOB INPUT_SOURCE src/Input/CameraInput/*.c*)
file(GLOB INPUT_COMMON_HEADER src/Input/Common/*.h*)
file(GLOB INPUT_COMMON_SOURCE src/Input/Common/*.c*)
set(INPUT_HEADER ${INPUT_HEADER} ${INPUT_COMMON_HEADER})
set(INPUT_SOURCE ${INPUT_SOURCE} ${INPUT_COMMON_SOURCE})
SOURCE_GROUP("INPUT Source" FILES ${INPUT_SOURCE})
SOURCE_GROUP("INPUT Header" FILES ${INPUT_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Input/Common)
include_directories(${CMAKE_SOURCE_DIR}/src/Input/CameraInput)

file(GLOB OUTPUT_HEADER src/Output/OutputWindow/*.h*)
file(GLOB OUTPUT_SOURCE src/Output/OutputWindow/*.c*)
file(GLOB OUTPUT_COMMON_HEADER src/Output/Common/*.h*)
file(GLOB OUTPUT_COMMON_SOURCE src/Output/Common/*.c*)
set(OUTPUT_HEADER ${OUTPUT_HEADER} ${OUTPUT_COMMON_HEADER})
set(OUTPUT_SOURCE ${OUTPUT_SOURCE} ${OUTPUT_COMMON_SOURCE})
SOURCE_GROUP("OUTPUT Source" FILES ${OUTPUT_SOURCE})
SOURCE_GROUP("OUTPUT Header" FILES ${OUTPUT_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Output/Common)
include_directories(${CMAKE_SOURCE_DIR}/src/Output/OutputWindow)




file(GLOB DETECTOR_HEADER src/Detector/FaceDetector/*.h*)
file(GLOB DETECTOR_SOURCE src/Detector/FaceDetector/*.c*)
SOURCE_GROUP("DETECTOR Source" FILES ${DETECTOR_SOURCE})
SOURCE_GROUP("DETECTOR Header" FILES ${DETECTOR_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Detector/Common)
include_directories(${CMAKE_SOURCE_DIR}/src/Detector/FaceDetector)


file(GLOB KALMAN_HEADER src/Tracker/Common/Filters/Kalman/*.h*)
file(GLOB KALMAN_SOURCE src/Tracker/Common/Filters/Kalman/*.c*)
SOURCE_GROUP("KALMAN Source" FILES ${KALMAN_SOURCE})
SOURCE_GROUP("KALMAN Header" FILES ${KALMAN_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/Filters/Kalman)

# file(GLOB GTL_HEADER src/Tracker/Common/GraphSolvers/GTL/include/GTL/*.h*)
# file(GLOB GTL_SOURCE src/Tracker/Common/GraphSolvers/GTL/src/*.c*)
# file(GLOB GRAPH_HEADER src/Tracker/Common/GraphSolvers/GTL/*.h*)
# file(GLOB GRAPH_SOURCE src/Tracker/Common/GraphSolvers/GTL/*.c*)
# file(GLOB BIPART_KALMAN_HEADER src/Tracker/BipartKalmanTracker/*.h*)
# file(GLOB BIPART_KALMAN_SOURCE src/Tracker/BipartKalmanTracker/*.c*)
# SOURCE_GROUP("BIPART Source" FILES ${GTL_SOURCE} ${GRAPH_SOURCE})
# SOURCE_GROUP("BIPART Header" FILES ${GTL_HEADER} ${GRAPH_HEADER})
# SOURCE_GROUP("BIPART_KALMAN Source" FILES ${BIPART_KALMAN_SOURCE})
# SOURCE_GROUP("BIPART_KALMAN Header" FILES ${BIPART_KALMAN_HEADER})
# include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/GraphSolvers/GTL/include)
# include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/GraphSolvers/GTL/include/GTL)
# include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/GraphSolvers/GTL)
# include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/BipartKalmanTracker)
# set(TRACKER_SOURCE ${KALMAN_SOURCE} 				 
#                    ${GTL_SOURCE}
#                    ${GRAPH_SOURCE}
#                    ${BIPART_KALMAN_SOURCE}
#                    )
# set(TRACKER_HEADER ${KALMAN_HEADER}  				                   
# 				     ${GTL_HEADER}
# 				     ${GRAPH_HEADER}
#                    ${BIPART_KALMAN_HEADER}
#                    )

file(GLOB HUNGARIAN_HEADER src/Tracker/Common/GraphSolvers/HungarianAlg/*.h*)
file(GLOB HUNGARIAN_SOURCE src/Tracker/Common/GraphSolvers/HungarianAlg/*.c*)
SOURCE_GROUP("HUNGARIAN Source" FILES ${HUNGARIAN_SOURCE})
SOURCE_GROUP("HUNGARIAN Header" FILES ${HUNGARIAN_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/Common/GraphSolvers/HungarianAlg)

file(GLOB HUNGARIAN_KALMAN_HEADER src/Tracker/HungarianKalmanTracker/*.h*)
file(GLOB HUNGARIAN_KALMAN_SOURCE src/Tracker/HungarianKalmanTracker/*.c*)
SOURCE_GROUP("HUNGARIAN_KALMAN Source" FILES ${HUNGARIAN_KALMAN_SOURCE})
SOURCE_GROUP("HUNGARIAN_KALMAN Header" FILES ${HUNGARIAN_KALMAN_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/src/Tracker/HungarianKalmanTracker)

set(TRACKER_SOURCE ${HUNGARIAN_SOURCE}
                   ${KALMAN_SOURCE}
                   ${HUNGARIAN_KALMAN_SOURCE}
                   )
set(TRACKER_HEADER ${HUNGARIAN_HEADER}                  
  				   ${KALMAN_HEADER}
                   ${HUNGARIAN_KALMAN_HEADER}
                   )

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
include_directories(${CMAKE_SOURCE_DIR}/src)

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
