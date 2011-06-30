/*
 *  hri_distance.c
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "hri.h"

#include <limits>

std::string hri_text_to_display;
double				hri_cost_to_display=0;
double                          hri_mindist=0;
bool				hri_draw_distance=false;
std::vector<double>		hri_disp_dist; // to draw the distance between the human and the robot
std::vector<double>		hri_histo_dist; // to save all distances that have been computed
static const double		hri_safe_radius = 2.0; // in meters

//! Enables colision detection
//! between the robot and the human
void hri_set_human_robot_dist( p3d_rob* rob, HRI_AGENTS* agents )
{
	for(int i=0; i<agents->humans_no; i++)
	{
//		std::cout << "Human is : " << GLOBAL_AGENTS->humans[i]->robotPt->name << std::endl;
		p3d_col_activate_rob_rob(rob,
														 agents->humans[i]->robotPt);
	}
}

//! Sets the collision checker in a normal
//! detection mode
void hri_set_normal_dist(p3d_rob* rob)
{

}

//! set the display mode
void hri_set_mindist_display(bool draw)
{
  hri_draw_distance = draw;
}

//! Comptue the distance between the
//! robot and the closest human agent
double hri_robot_min_distance( HRI_AGENTS* agents )
{
	p3d_rob* rob = agents->robots[agents->source_agent_idx]->robotPt;

	std::cout << "Compute dist for : " << rob->name << std::endl;

//	printf("hri_robot_min_distance\n");
	hri_set_human_robot_dist(rob,agents);

	int nof_bodies = rob->no;
	double* distances = new double[nof_bodies];
	p3d_vector3 *body = new p3d_vector3[nof_bodies];
	p3d_vector3 *other = new p3d_vector3[nof_bodies];

	int k=0;
	double minDist = std::numeric_limits<double>::max();

	// Checkout the collision checker version
	switch (p3d_col_get_mode())
	{
		case p3d_col_mode_kcd:
		{
			int settings = get_kcd_which_test();
			set_kcd_which_test((p3d_type_col_choice)(20+3));
			// 40 = KCD_ROB_ENV
			// 3 = DISTANCE_EXACT

			p3d_col_test_choice();
			// Collision detection with other robots only

			p3d_kcd_closest_points_between_bodies(rob,body,other,distances);

			// Get Min indice of distance
			for(int i=0;i<nof_bodies;i++)
			{
				std::cout << "distances[" << i << "] = " << distances[i] << std::endl;
				if( minDist > distances[i] )
				{
					minDist = distances[i];
					k = i;
				}
			}

			set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL

			break;
		}
		case p3d_col_mode_pqp:
		{
		        minDist =  pqp_robot_robot_distance(rob,GLOBAL_AGENTS->humans[0]->robotPt, body[k], other[k]);

			// Only handles two humans
			if( GLOBAL_AGENTS->humans_no > 1 )
			  {
			    p3d_vector3 body2,other2;
			    double minDist2 = pqp_robot_robot_distance(rob,GLOBAL_AGENTS->humans[1]->robotPt, body2, other2);

			    if(minDist2 < minDist)
			      {
				minDist = minDist2;

				body[k][0] = body2[0];
				body[k][1] = body2[1];
				body[k][2] = body2[2];

				other[k][0] = other2[0];
				other[k][1] = other2[1];
				other[k][2] = other2[2];
			      }
			  }
			break;
		}
	}

	if(hri_draw_distance)
	{
		hri_disp_dist.clear();
		hri_disp_dist.push_back(body[k][0]);
		hri_disp_dist.push_back(body[k][1]);
		hri_disp_dist.push_back(body[k][2]);
		hri_disp_dist.push_back(other[k][0]);
		hri_disp_dist.push_back(other[k][1]);
		hri_disp_dist.push_back(other[k][2]);

//		std::cout	<< "vect_jim[0] = " << hri_disp_dist[0]
//							<< " vect_jim[1] = " << hri_disp_dist[1]
//					    << " vect_jim[2] = " << hri_disp_dist[2] << std::endl;
//
//		std::cout	<< "vect_jim[3] = " << hri_disp_dist[3]
//							<< " vect_jim[4] = " << hri_disp_dist[4]
//					    << " vect_jim[5] = " << hri_disp_dist[5] << std::endl;
	}

//	hri_set_normal_dist(rob);

	delete[] distances;
	delete[] other;
	delete[] body;

	return minDist;
}

//! Cost between 0 and 1 for the distance to the robot
//! the minimal distance is computed and a squaling is done to have
//! a cost in [0 1]
double hri_distance_cost(HRI_AGENTS* agents, double& distance)
{
	distance = hri_robot_min_distance(agents);

	double penetrationRatio = (hri_safe_radius - distance)/hri_safe_radius;

	double Cost=0.0;
	// double Cost = 0.00001;

	// Compute of the hri cost function
	if ( penetrationRatio > 0 )
	{
	  const double k = 2.0;
	  Cost = pow( penetrationRatio , k );
	  //Cost += (exp(penetrationDist-1) - exp(-1) ) / ( 1 - exp(-1) );
	  // Cost += _PenetrationDist[k];
	}


	// Set the Hri Cost To Display
	hri_mindist = hri_cost_to_display = distance;

	return Cost;
}

void hri_draw_mindist()
{
  if( (!hri_draw_distance) || (hri_mindist > hri_safe_radius) )
    return;

  double color[4];

  color[0]= 1.0; 
  color[1]= 0.0; 
  color[2]= 0.0; 
  color[3]= 1.0;

  glLineWidth(3.);

  g3d_drawOneLine(hri_disp_dist[0],hri_disp_dist[1],hri_disp_dist[2],
		  hri_disp_dist[3],hri_disp_dist[4],hri_disp_dist[5],Any,color);
}
