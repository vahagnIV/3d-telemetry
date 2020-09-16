file(GLOB SOURCE_FILES *.cpp feature_extraction/*.cpp state/*.cpp)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${realsense2_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
message(${CMAKE_CURRENT_SOURCE_DIR}/third_party)
message(${realsense2_LIBRARY})
add_library(3dtelemetry ${SOURCE_FILES})

target_link_libraries(3dtelemetry ${realsense2_LIBRARY} ${OpenCV_LIBRARIES} )
