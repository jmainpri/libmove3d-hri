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
#ifndef HRI_BTMAP_UTIL
#define HRI_BTMAP_UTIL


#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "hri.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "math.h"

int on_map(int x, int y, int z, hri_bitmap* bitmap);

//int get_direction(hri_bitmap_cell *satellite_cell, hri_bitmap_cell *center_cell);

int isHardEdge(hri_bitmap_cell *last_cell, hri_bitmap_cell *middle_cell);

hri_bitmap* hri_bt_get_bitmap(int type, hri_bitmapset* bitmapset);

hri_bitmap_cell* hri_bt_get_cell(hri_bitmap* bitmap, int x, int y, int z);

hri_bitmap_cell* hri_bt_get_closest_cell(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z);

hri_bitmap_cell* hri_bt_get_closest_free_cell(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z, double orientation, double max_square_tolerance);

int hri_bt_isRobotOnCellInCollision(hri_bitmapset * btset, hri_bitmap* bitmap, hri_bitmap_cell* cell, double orientation, int checkHumanCollision);

hri_bitmap_cell* hri_bt_getCellOnPath(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z);

void hri_bt_copy_bitmap_values(hri_bitmap* bitmap_source, hri_bitmap* bitmap_target);

hri_bitmap* hri_bt_create_copy(hri_bitmap* bitmap);

hri_bitmapset*  hri_bt_create_empty_bitmapset();

hri_bitmap*  hri_bt_create_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int));

int hri_bt_create_data(hri_bitmap* bitmap);

hri_bitmap*  hri_bt_create_empty_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int));

int hri_bt_change_bitmap_position(hri_bitmapset * btset, double x, double y, double z);

int hri_bt_destroy_bitmap(hri_bitmap* bitmap);

int hri_bt_destroy_bitmap_data(hri_bitmap* bitmap);

int hri_bt_equalPath(hri_bitmap* bitmap1, hri_bitmap* bitmap2);

int localPathCollides (hri_bitmapset * btset, hri_bitmap_cell* cell, hri_bitmap_cell* fromcell);

double getCellDistance (hri_bitmap_cell* cell1, hri_bitmap_cell* cell2 );

double get3CellAngle(hri_bitmap_cell* cell1, hri_bitmap_cell* cell2, hri_bitmap_cell* cell3);

hri_bitmap_cell* hri_bt_nth_from_start(hri_bitmap_cell* path_start, hri_bitmap_cell* path_end, int n);

double getPathGridLength(hri_bitmap_cell* path_end);

double normalizeAngleDeviation(double angle_deviation);

double getAngleDeviation(double angle1, double angle2);

void distanceFromLine(double cx, double cy, double ax, double ay ,
                                          double bx, double by,
                                          double * distanceSegment, double * distanceLine,
                                          double * proPointx, double * proPointy);

double getRotationBoundingCircleRadius(p3d_rob *robot);

#endif

