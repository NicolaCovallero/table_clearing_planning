#edit the following line to add the librarie's header files
FIND_PATH(table_clearing_planning_INCLUDE_DIR table_clearing_planning.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(table_clearing_planning_LIBRARY
    NAMES table_clearing_planning
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (table_clearing_planning_INCLUDE_DIR AND table_clearing_planning_LIBRARY)
   SET(table_clearing_planning_FOUND TRUE)
ENDIF (table_clearing_planning_INCLUDE_DIR AND table_clearing_planning_LIBRARY)

IF (table_clearing_planning_FOUND)
   IF (NOT table_clearing_planning_FIND_QUIETLY)
      MESSAGE(STATUS "Found table_clearing_planning: ${table_clearing_planning_LIBRARY}")
   ENDIF (NOT table_clearing_planning_FIND_QUIETLY)
ELSE (table_clearing_planning_FOUND)
   IF (table_clearing_planning_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find table_clearing_planning")
   ENDIF (table_clearing_planning_FIND_REQUIRED)
ENDIF (table_clearing_planning_FOUND)

