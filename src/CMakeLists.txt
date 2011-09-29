
FOREACH (srcfile 
	hri_agent.cpp 
	hri_gik.cpp 
	hri_visibility.cpp
	hri_graphic.cpp
	hri_distance.cpp
	hri_bitmap.cpp
	hri_manip.cpp
	hri_knowledge.cpp
	hri_kinect.cpp
        )
	LIST(APPEND HRI_SRC ${srcfile})
ENDFOREACH(srcfile)

IF(USE_MIGHTABILITY_MAPS)
    MESSAGE("Mightability Maps enabled")
    LIST (APPEND HRI_SRC 
                 Mightability_Analysis.cpp 
			     HRI_tasks.cpp
		  Agent_State_Analysis.cpp
    )
  ADD_DEFINITIONS("-DUSE_MIGHTABILITY_MAPS")
ENDIF(USE_MIGHTABILITY_MAPS)

FILE(GLOB HRIBITMAP_SRC "hri_bitmap/*.cpp")
FOREACH (srcfile ${HRIBITMAP_SRC})
	LIST(APPEND HRI_SRC ${srcfile})
ENDFOREACH(srcfile)

IF(USE_HRP2_GIK)
	MESSAGE("HRP2_gik.cpp can not be compiled for now since it badly \
			mix algorithmic part with GUI parts. Please untangle them, and move \
			the GUI parts in the move3d-studio project.")
	#LIST (APPEND HRI_SRC 
	#HRP2_gik.cpp
	#)
ENDIF(USE_HRP2_GIK)

########################################
##       HRI library                  ##
########################################

ADD_LIBRARY (move3d-hri ${HRI_SRC})

TARGET_LINK_LIBRARIES(move3d-hri ${LIBS})

INSTALL (TARGETS move3d-hri
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)


