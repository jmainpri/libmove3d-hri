/*
 *  hri_distance.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRI_DISTANCE_H
#define HRI_DISTANCE_H

#include "hri_agent.h"

#include <limits>

double hri_robot_min_distance( HRI_AGENT* robot, HRI_AGENT* human, double minDistPrev = std::numeric_limits<double>::max() ); 
double hri_robot_min_distance( HRI_AGENTS* agents );

double hri_distance_cost( HRI_AGENTS* agents, double& distance );
double hri_distance_cost( HRI_AGENT* robot, HRI_AGENT* human, double& distance );

void hri_set_mindist_display(bool draw);
void hri_draw_mindist();

#endif
