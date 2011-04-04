FOREACH (srcfile 
             g3d_draw_camera.c 
			 g3d_draw_navigation.c 
			 g3d_perspective_window.c 
			 g3d_position_area.c)
		 LIST(APPEND HRI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${srcfile})
ENDFOREACH(srcfile)

