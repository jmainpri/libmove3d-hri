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
/*
 * hri_bitmap_cost.h
 *
 *  Created on: Jun 29, 2009
 *      Author: kruset
 */

#ifndef HRI_BITMAP_COST_H_
#define HRI_BITMAP_COST_H_
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "hri.h"
#include "math.h"
#include "hri_bitmap_util.h"

double getPathCost(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap_cell* robot_position);
double hri_bt_A_CalculateCellG(hri_bitmapset * btset, hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell, double step_distance);
double hri_bt_dist_heuristic(hri_bitmapset * btset, hri_bitmap* bitmap, int x_s, int y_s, int z_s);
int CalculateCellValue(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell );
int getClosestPoseOnProjection(hri_bitmapset * btset, hri_human* cur_human, double realx, double realy, double projectionTime, double * projectionx, double  * projectiony, double  * projectionth);
double hri_bt_calc_hz_value_human ( hri_bitmapset * btset, hri_human* cur_human, int x, int y );
double hri_bt_calc_dist_value_human ( hri_bitmapset * btset, hri_human* cur_human, int x, int y, int z );
double hri_bt_calc_vis_value_human ( hri_bitmapset * btset, hri_human* cur_human, int x, int y, int z );
double hri_bt_calc_hz_value ( hri_bitmapset * btset, int x, int y, int z );
double hri_bt_calc_dist_value ( hri_bitmapset * btset, int x, int y, int z );
double hri_bt_calc_vel_value ( hri_bitmapset * btset, int x, int y, int z );
double hri_bt_calc_vis_value ( hri_bitmapset * btset, int x, int y, int z );
double hri_bt_calc_combined_value ( hri_bitmapset * btset, int x, int y, int z );
int hri_bt_keep_old_path(hri_bitmapset* bitmapset, hri_bitmap* bitmap_oldpath, hri_bitmap* bitmap_newpath, double newcosts, hri_bitmap_cell* new_search_start);
#endif /* HRI_BITMAP_COST_H_ */
