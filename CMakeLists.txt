# The name of the project is "LIBORO". CMakeLists files in this project can
# refer to the root source directory of the project as ${LIBORO_SOURCE_DIR} and
# to the root binary directory of the project as ${LIBORO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(LIBHRI CXX)

set(PACKAGE_NAME libhri)
set(PACKAGE_VERSION 3.10.99)

#Uncomment next line to get verbose makefiles.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Pick up our scripts - they are all in the conf subdirectory
SET(LIBHRI_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${LIBHRI_MODULE_PATH})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

find_package(PkgConfig REQUIRED)

pkg_check_modules(MOVE3D REQUIRED libmove3d)
include_directories(${MOVE3D_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
set(LIBS ${LIBS} ${MOVE3D_LIBRARIES})

SET(BUILD_SHARED_LIBS true)

# Get the current libhri version
ADD_DEFINITIONS(${MOVE3D_CFLAGS} -DUNIX -DHRI_VERSION="${PACKAGE_VERSION}")

INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

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
##       HRI library                 ###
##########################################


ADD_SUBDIRECTORY(src)


