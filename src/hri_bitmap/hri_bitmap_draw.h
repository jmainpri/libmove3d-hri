/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#ifndef HRI_BTMAP_DRAW
#define HRI_BTMAP_DRAW


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "hri.h"
#include "math.h"

int hri_bt_insert_obs(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_obj* obj, p3d_env* env, double expand, double value, int manip);
int hri_bt_insert_obsrobot(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_rob* obj, p3d_env* env, double expand, double value, int manip);
int  hri_bt_fill_bitmap_zone(hri_bitmapset * btset, hri_bitmap* bitmap, double xmin, double xmax, double ymin,
    double ymax, double zmin, double zmax, double expand, int val, int manip);
void hri_bt_show_path(hri_bitmapset * btset, hri_bitmap* bitmap);
void hri_bt_clearCorridorMarks(hri_bitmapset * btset, hri_bitmap* bitmap);

#endif

