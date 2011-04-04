
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

	ADD_DEFINITIONS(-DUSE_MIGHTABILITY_MAPS)

ENDIF(USE_MIGHTABILITY_MAPS)

FILE(GLOB HRIBITMAP_SRC "hri_bitmap/*.cpp")
FOREACH (srcfile ${HRIBITMAP_SRC})
	LIST(APPEND HRI_SRC ${srcfile})
ENDFOREACH(srcfile)

IF(HRI_PLANNER_GUI)

    LIST (APPEND HRI_SRC 
    FORMgikjointselection.c 
    FORMhri_planner.c 
    FORMpsp_parameters.c 
    hri_wave_exp.c 
    p3d_perspective.c
    )

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
    
	ADD_SUBDIRECTORY(graphic)

ENDIF(HRI_PLANNER_GUI)


FILE(GLOB_RECURSE HRI_HEADERS "include/*.h")

ADD_LIBRARY (hri ${HRI_SRC})
#set_target_properties(hri
#                      PROPERTIES PUBLIC_HEADER "${HRI_HEADERS}"
#)


INSTALL (TARGETS hri
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
#	PUBLIC_HEADER DESTINATION include/hri
)

INSTALL(FILES
	${PUBLIC_HEADERS}
	DESTINATION include/hri
)


