FILE(GLOB HRI_HEADERS "src/include/*.h")

LIST (INSERT HRI_SRC 
src/hri_agent.cpp 
src/hri_gik.cpp 
src/hri_visibility.cpp
src/hri_graphic.cpp
src/hri_distance.cpp
src/hri_bitmap.cpp
src/hri_manip.cpp
src/hri_knowledge.cpp
src/hri_kinect.cpp
)

IF(USE_MIGHTABILITY_MAPS)
    LIST (INSERT SPARK_SRC 
    Mightability_Analysis.cpp 
    HRI_tasks.cpp
    )
ENDIF(USE_MIGHTABILITY_MAPS)

include(src/hri_bitmap/SourceList.cmake)

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
    
    include(src/graphic/SourceList.cmake)

ENDIF(HRI_PLANNER_GUI)
