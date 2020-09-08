file(GLOB SOURCE_FILES *.cpp feature_extraction/*.cpp )
find_package(realsense2 REQUIRED)

include_directories(${realsense2_INCLUDE_DIRS})
message(${realsense2_LIBRARY})
add_library(3dtelemetry ${SOURCE_FILES} )

target_link_libraries(3dtelemetry ${realsense2_LIBRARY})
