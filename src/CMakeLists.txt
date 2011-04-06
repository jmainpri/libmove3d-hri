
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
    )
	ADD_DEFINITIONS("-DUSE_MIGHTABILITY_MAPS")
ENDIF(USE_MIGHTABILITY_MAPS)

FILE(GLOB HRIBITMAP_SRC "hri_bitmap/*.cpp")
FOREACH (srcfile ${HRIBITMAP_SRC})
	LIST(APPEND HRI_SRC ${srcfile})
ENDFOREACH(srcfile)

IF(HRI_PLANNER_GUI)

    IF(USE_MIGHTABILITY_MAPS)
        LIST (APPEND HRI_SRC 
        FORM_HRI_affordance.c
        )
    ENDIF(USE_MIGHTABILITY_MAPS)
    
    IF(USE_HRP2_GIK)
        LIST (APPEND HRI_SRC 
        HRP2_gik.cpp
        )
    ENDIF(USE_HRP2_GIK)
    

ENDIF(HRI_PLANNER_GUI)

########################################
##       HRI library                  ##
########################################

ADD_LIBRARY (hri ${HRI_SRC})

TARGET_LINK_LIBRARIES(hri ${LIBS})

INSTALL (TARGETS hri
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)


