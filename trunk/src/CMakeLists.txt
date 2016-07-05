FIND_PACKAGE(PCL 1.8 REQUIRED)
FIND_PACKAGE(fcl REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)



# driver source files
SET(sources table_clearing_planning.cpp edge_processing.cpp utilities.cpp)
# application header files
SET(headers table_clearing_planning.h edge_processing.h conversions.h utilities.h)
# locate the necessary dependencies
# add the necessary include directories
INCLUDE_DIRECTORIES(.)

IF (PCL_FOUND)
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
  ADD_DEFINITIONS(${PCL_DEFINITIONS})
ENDIF (PCL_FOUND) 

include_directories(${fcl_INCLUDE_DIR} )

# create the shared library
ADD_LIBRARY(table_clearing_planning SHARED ${sources})
TARGET_LINK_LIBRARIES(table_clearing_planning ${PCL_LIBRARIES} ${fcl_LIBRARIES} ${OpenCV_LIBS}) 

# link necessary libraries
INSTALL(TARGETS table_clearing_planning
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
INSTALL(FILES ${headers} DESTINATION include/iridrivers)
INSTALL(FILES ../Findtable_clearing_planning.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
