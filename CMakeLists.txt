# The name of the project is "LIBHRI". CMakeLists files in this project can
# refer to the root source directory of the project as ${LIBHRI_SOURCE_DIR} and
# to the root binary directory of the project as ${LIBHRI_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(LIBHRI C CXX)

set( PACKAGE_NAME libhri )
set( PACKAGE_VERSION 1.0 )

#Uncomment next line to get verbose makefiles.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Pick up our scripts - they are all in the conf subdirectory
SET(LIBHRI_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${LIBHRI_MODULE_PATH})

#--------------- Platform -------------
#--------------------------------------

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX)
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

########################################
##            Dependencies            ##
########################################

FIND_PACKAGE (PkgConfig REQUIRED)

### Move3D ###

SET(MOVE3D "")

IF(DEFINED LIBMOVE3D_PC)
        SET(ENV{PKG_CONFIG_PATH} ${LIBMOVE3D_PC}:$ENV{PKG_CONFIG_PATH})
ENDIF()

IF(NOT USE_QT_LIBMOVE3D_VIEWER)
    	MESSAGE(libmove3d)
    	PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d)
	INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
	LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
	SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
	ADD_DEFINITIONS(${MOVE3D_CFLAGS})
ELSE()
    	MESSAGE(libmove3d-viewer)
    	PKG_CHECK_MODULES(MOVE3DVIEWER REQUIRED libmove3d-viewer)
    	INCLUDE_DIRECTORIES(${MOVE3DVIEWER_INCLUDE_DIRS})
	LINK_DIRECTORIES( ${MOVE3DVIEWER_LIBRARY_DIRS} )
	SET(LIBS ${LIBS} ${MOVE3DVIEWER_LIBRARIES})
	ADD_DEFINITIONS(${MOVE3DVIEWER_CFLAGS})
ENDIF()

### GSL ###

PKG_CHECK_MODULES(GSL REQUIRED gsl)

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})
LINK_DIRECTORIES( ${GSL_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${GSL_LIBRARIES})

## GL ###

FIND_PACKAGE(OpenGL REQUIRED)
INCLUDE_directories(${OPENGL_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})

########################################
##          Documentation             ##
########################################

INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

##########################################
##       PKGCONFIG file                ###
##########################################
IF(USE_MIGHTABILITY_MAPS)
SET(LIBHRI_COMPILATION_FLAGS "-DUSE_MIGHTABILITY_MAPS")
ENDIF()

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
OPTION (USE_QT_LIBMOVE3D_VIEWER "Enable Linking with libmove3d-veiwer" OFF)

##########################################
##       HRI library                 ###
##########################################

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${LIBHRI_BINARY_DIR}/install" CACHE PATH "LibHri install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

ADD_DEFINITIONS(-DHRI_VERSION="${PACKAGE_VERSION}")

SET(BUILD_SHARED_LIBS true)

INSTALL(DIRECTORY
       "include/"
       DESTINATION include/hri
)

#MESSAGE("Linking with ${LIBS}")

ADD_SUBDIRECTORY (src)




