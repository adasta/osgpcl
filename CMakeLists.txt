cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(osgpcl)
	
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib)

include_directories( include)

set(CMAKE_BUILD_TYPE "DEBUG")
######################### External Libraries ###########################
	
find_package(PCL 1.7 REQUIRED COMPONENTS 
			io octree filters search kdtree  outofcore visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenSceneGraph REQUIRED COMPONENTS osgGA osgDB osgViewer osgUtil )
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS} )
link_directories(/usr/lib/osgPlugins-3.0.1/)

FIND_PACKAGE( Boost 1.44 COMPONENTS 
					program_options filesystem thread
					serialization python 
					REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
include_directories( include)

######################### Project Variables ############################
	
set(OSGPCL_DEP_LIBRARIES  ${PCL_LIBRARIES}
${OPENSCENEGRAPH_LIBRARIES}
${Boost_LIBRARIES}
)

######################## Executables / Libraries ######################


add_library(osgpcl src/point_cloud.cpp 
				   src/common.cpp
				   src/surfel.cpp  
				   src/shapes.cpp
				   )
target_link_libraries(osgpcl debug ${OSGPCL_DEP_LIBRARIES} )

add_library(osgdb_oct_idx  src/outofcore_octree_reader.cpp)
target_link_libraries(osgdb_oct_idx osgpcl ${OSGPCL_DEP_LIBRARIES} osgDB)

add_library(osgdb_pcd src/point_cloud_reader.cpp)
target_link_libraries(osgdb_pcd osgpcl ${OSGPCL_DEP_LIBRARIES} osgDB)

add_executable(cloud_viewer apps/cloud_viewer.cpp)
target_link_libraries(cloud_viewer osgpcl ${OSGPCL_DEP_LIBRARIES} osgdb_pcd osgdb_oct_idx)


INSTALL(TARGETS osgpcl osgdb_oct_idx osgdb_pcd cloud_viewer
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

INSTALL(DIRECTORY include/osgpcl  DESTINATION include)

#add a directory for throw away programs for testing out code
if( IS_DIRECTORY "${CMAKE_SOURCE_DIR}/scratch")
message(STATUS "Adding scratch directory")
add_subdirectory(scratch)
endif()
