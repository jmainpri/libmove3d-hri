# The name of the project is "LIBHRI". CMakeLists files in this project can
# refer to the root source directory of the project as ${LIBHRI_SOURCE_DIR} and
# to the root binary directory of the project as ${LIBHRI_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(LIBHRI CXX)

set( PACKAGE_NAME libhri )
set( PACKAGE_VERSION 3.10.99 )

#Uncomment next line to get verbose makefiles.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Pick up our scripts - they are all in the conf subdirectory
SET(LIBHRI_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${LIBHRI_MODULE_PATH})

#--------------------- Move3D -------------------------------------------------
FIND_PACKAGE (Pkgconfig)
pkg_check_modules(MOVE3D REQUIRED "libmove3d")

#MESSAGE( "${MOVE3D_INCLUDE_DIRS}" )
#MESSAGE( "${MOVE3D_LIBRARY_DIRS}" )

MESSAGE( "${MOVE3D_CFLAGS}" )
MESSAGE( "${MOVE3D_LDFLAGS}" )

#INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
#LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
#set(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
#------------------------------------------------------------------------------

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

SET(BUILD_SHARED_LIBS true)

# Get the current libhri version
ADD_DEFINITIONS(${MOVE3D_CFLAGS} -DUNIX -DHRI_VERSION="${PACKAGE_VERSION}")

# La target ne link absolument pas contre libmove3d
# si tu compile sans HRI_PLANNER il y a plein d'erreures
# les lignes suivantes sont celles que j'utilise dans mon
# projet pour linker un pkg issue de pkgconfig, bonne chance et bon courage
# pour le linker avec ce projet lˆ (j'y ai passŽ une heure j'ai
# pas rŽussit, il y aurait une target par default qui a le mme nom )

#ADD_LIBRARY(hri SHARED ${PROJECT_SOURCE_DIR}/src)
#SET_TARGET_PROPERTIES(hri PROPERTIES COMPILE_FLAGS ${MOVE3D_CFLAGS})
#TARGET_LINK_LIBRARIES(hri ${MOVE3D_LDFLAGS})

##########################################
##       PKGCONFIG file                ###
##########################################

IF   (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libhri.pc.in")
    MESSAGE(STATUS "configured ${CMAKE_CURRENT_SOURCE_DIR}/libhri.pc.in --> ${CMAKE_CURRENT_BINARY_DIR}/libhri.pc")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/libhri.pc.in 
      ${CMAKE_CURRENT_BINARY_DIR}/libhri.pc
      @ONLY )

	INSTALL(FILES
		${CMAKE_CURRENT_BINARY_DIR}/libhri.pc
		DESTINATION lib/pkgconfig
	)
ENDIF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libhri.pc.in")

##########################################
##           Options                   ###
##########################################

OPTION (USE_MIGHTABILITY_MAPS "Enable Mightability Maps support" OFF)

##########################################
##       HRI library                 ###
##########################################


ADD_SUBDIRECTORY (src)


