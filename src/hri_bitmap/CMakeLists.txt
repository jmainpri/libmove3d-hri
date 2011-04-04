FOREACH (srcfile 
             hri_bitmap_bin_heap.c 
			 hri_bitmap_cost.c 
			 hri_bitmap_draw.c 
			 hri_bitmap_util.c)
		 LIST(APPEND HRI_SRC ${CMAKE_CURRENT_SOURCE_DIR}/${srcfile})
ENDFOREACH(srcfile)
