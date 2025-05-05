cmake_minimum_required(VERSION 3.19)
project(spatial_partitioning)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
else()
endif()

add_subdirectory(thirdparty/glm)

include_directories(include)

aux_source_directory(./include/spatial_partitioning/ header_files)
aux_source_directory(./src/ source_files)
aux_source_directory(./src/bullet/ bt_source_files)

add_library(spatial_partitioning STATIC
	${bt_source_files}
	${header_files}
	${source_files}
)
target_link_libraries(spatial_partitioning PUBLIC glm)

add_executable(test1 tests/Test1.cpp)
target_link_libraries(test1 spatial_partitioning)
set_target_properties(test1 PROPERTIES ENABLE_EXPORTS ON)

add_executable(testAabb tests/TestAabb.cpp)
target_link_libraries(testAabb spatial_partitioning)
