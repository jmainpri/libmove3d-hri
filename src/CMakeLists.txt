
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
	LIST(APPEND HRI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${srcfile})
ENDFOREACH(srcfile)

IF(USE_MIGHTABILITY_MAPS)
    LIST (INSERT SPARK_SRC 
    Mightability_Analysis.cpp 
    HRI_tasks.cpp
    )
ENDIF(USE_MIGHTABILITY_MAPS)

#ADD_SUBDIRECTORY(hri_bitmap)

IF(HRI_PLANNER_GUI)

    LIST (INSERT SPARK_SRC 
    FORMgikjointselection.c 
    FORMhri_planner.c 
    FORMpsp_parameters.c 
    hri_wave_exp.c 
    p3d_perspective.c
    )

    IF(USE_MIGHTABILITY_MAPS)
        LIST (INSERT SPARK_SRC 
        FORM_HRI_affordance.c
        )
    ENDIF(USE_MIGHTABILITY_MAPS)
    
    IF(USE_HRP2_GIK)
        LIST (INSERT SPARK_SRC 
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


