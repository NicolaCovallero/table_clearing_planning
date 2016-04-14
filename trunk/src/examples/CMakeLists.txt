# create an example application
ADD_EXECUTABLE(table_clearing_planning_test table_clearing_planning_test.cpp)

FIND_PACKAGE( PCL 1.8 REQUIRED )
find_package(tos_supervoxels REQUIRED)


IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND)
include_directories(${tos_supervoxels_INCLUDE_DIR})


# link necessary libraries

TARGET_LINK_LIBRARIES(table_clearing_planning_test table_clearing_planning  ${PCL_LIBRARIES} ${tos_supervoxels_LIBRARY})
