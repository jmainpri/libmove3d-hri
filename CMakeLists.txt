# The name of the project is "LIBMOVE3D_HRI". CMakeLists files in this project can
# refer to the root source directory of the project as ${LIBMOVE3D_HRI_SOURCE_DIR} and
# to the root binary directory of the project as ${LIBMOVE3D_HRI_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(LIBMOVE3D_HRI C CXX)

set( PACKAGE_NAME libmove3d-hri )
set( PACKAGE_VERSION 1.99 )

#Uncomment next line to get verbose makefiles.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Pick up our scripts - they are all in the conf subdirectory
SET(LIBMOVE3D_HRI_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${LIBMOVE3D_HRI_MODULE_PATH})

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

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

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



PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d)
INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D_CFLAGS})

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

#INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)

FIND_PACKAGE(Doxygen)

SET(DOXYFILE_FOUND false)

IF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})
    SET(DOXYFILE_FOUND true)
ENDIF(EXISTS ${PROJECT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE})

IF( DOXYGEN_FOUND )
          MESSAGE("Doxygen found")
          IF( DOXYFILE_FOUND )
	      MESSAGE("Project source dir : ${PROJECT_SOURCE_DIR}" )
	      MESSAGE("Doxyfile found")
	      MESSAGE("Doxygen config file : ${DOXYGEN_CONFIG_FILE}")
	      # Add target
	      ADD_CUSTOM_TARGET( doc ALL ${DOXYGEN_EXECUTABLE} "${PROJECT_SOURCE_DIR}/Doxyfile.in" )
    	 ELSE( DOXYFILE_FOUND )
    	  		 MESSAGE( STATUS "Doxygen configuration file not found - Documentation will not be generated" )
    	  ENDIF( DOXYFILE_FOUND )
    ELSE(DOXYGEN_FOUND)
          MESSAGE(STATUS "Doxygen not found - Documentation will not be generated")
ENDIF(DOXYGEN_FOUND)

##########################################
##       PKGCONFIG file                ###
##########################################
IF(USE_MIGHTABILITY_MAPS)
SET(LIBMOVE3D_HRI_COMPILATION_FLAGS "-DUSE_MIGHTABILITY_MAPS")
ENDIF()

IF   (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libmove3d-hri.pc.in")
    MESSAGE(STATUS "configured ${CMAKE_CURRENT_SOURCE_DIR}/libmove3d-hri.pc.in --> ${CMAKE_CURRENT_BINARY_DIR}/libmove3d-hri.pc")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/libmove3d-hri.pc.in 
      ${CMAKE_CURRENT_BINARY_DIR}/libmove3d-hri.pc
      @ONLY )

	INSTALL(FILES
		${CMAKE_CURRENT_BINARY_DIR}/libmove3d-hri.pc
		DESTINATION lib/pkgconfig
	)
ENDIF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libmove3d-hri.pc.in")

##########################################
##           Options                   ###
##########################################

OPTION (USE_MIGHTABILITY_MAPS "Enable Mightability Maps support" OFF)


##########################################
##       HRI library                 ###
##########################################

IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#  SET(CMAKE_INSTALL_PREFIX "${LIBMOVE3D_HRI_BINARY_DIR}/install" CACHE PATH "LibHri install prefix" FORCE)
  SET(CMAKE_INSTALL_PREFIX "$ENV{ROBOTPKG_BASE}" CACHE PATH "LibHri install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

ADD_DEFINITIONS(-DMOVE3D_HRI_VERSION="${PACKAGE_VERSION}")

SET(BUILD_SHARED_LIBS true)

INSTALL(DIRECTORY
       "include/"
       DESTINATION include/libmove3d/hri
)

#MESSAGE("Linking with ${LIBS}")

ADD_SUBDIRECTORY (src)




