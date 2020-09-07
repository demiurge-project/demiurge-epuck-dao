#
# Set minimum required version
#
cmake_minimum_required(VERSION 2.8)

#if(ARGOS_BUILD_FOR_SIMULATOR)
	# Set up ARGoS compilation information
	include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
	link_directories(${ARGOS_LIBRARY_DIRS})

	# Headers
        set(HEADERS
          EpuckDAO.h
          RabMessageBuffer.h
	  ReferenceModel1Dot1.h
	  ReferenceModel1Dot2.h
          ReferenceModel3Dot1.h
	)

	# Sources
	set(SOURCES
          EpuckDAO.cpp
          RabMessageBuffer.cpp
	  ReferenceModel1Dot1.cpp
	  ReferenceModel1Dot2.cpp
          ReferenceModel3Dot1.cpp
	)
#else()
#endif(ARGOS_BUILD_FOR_SIMULATOR)


add_library(argos3_demiurge_epuck_dao SHARED ${HEADERS} ${SOURCES} )
target_link_libraries(argos3_demiurge_epuck_dao argos3plugin_${ARGOS_BUILD_FOR}_epuck)

install(
  FILES ${HEADERS}
  DESTINATION include/argos3/demiurge/epuck-dao)

# Install location for the e-puck plugin
install(TARGETS argos3_demiurge_epuck_dao
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
