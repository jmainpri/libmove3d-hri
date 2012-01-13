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
void hri_set_human_robot_dist( p3d_rob* rob, HRI_AGENT* agent )
{
  //	for(int i=0; i<agents->humans_no; i++)
  //	{
  p3d_col_activate_rob_rob( rob, agent->robotPt );
  //	}
}

//! Sets the collision checker in a normal
//! detection mode
void hri_set_normal_dist(p3d_rob* rob)
{

}

bool hri_activate_coll_between_robot_and_one_human_arms( HRI_AGENT* robot, HRI_AGENT* human, bool enable )
{  
  if( human->type != HRI_HERAKLES )
    {
      return false;
    }

  std::vector<p3d_jnt*> ArmJoints;
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rShoulderX" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rShoulderY" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rShoulderZ" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rArmTrans" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rElbowZ" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lPoint" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rWristX" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rWristY" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "rWristZ" ) );

  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lShoulderX" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lShoulderY" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lShoulderZ" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lArmTrans" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lElbowZ" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lPoint" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lWristX" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lWristY" ) );
  ArmJoints.push_back( p3d_get_robot_jnt_by_name( human->robotPt, "lWristZ" ) );

  for(int i=0;i<robot->robotPt->no;i++)
    {
      p3d_obj* obj1 =  robot->robotPt->o[i];
      if( obj1 == NULL )
	continue;

      for(int j=0;j<ArmJoints.size();j++)
	{ 
	  p3d_obj* obj2 =  ArmJoints[j]->o;
	  if( obj2 == NULL )
	    continue;

	  if( !enable )
	    p3d_col_deactivate_pair_of_objects( obj1, obj2 );
	  else
	    p3d_col_activate_pair_of_objects( obj1, obj2 );
	}
    }

  return true;
}

//! 
//!
bool hri_activate_coll_robot_and_all_humans_arms( HRI_AGENT* robot, HRI_AGENTS* agents, bool enable ) 
{
  for(int i=0;i<agents->humans_no;i++)
    {
      if( agents->humans[i]->type != HRI_HERAKLES )
	{
	  return false;
	}

      hri_activate_coll_between_robot_and_one_human_arms( robot, agents->humans[i], enable );
    }
  
  return true;
}


//! set the display mode
void hri_set_mindist_display(bool draw)
{
  hri_draw_distance = draw;
}

double hri_robot_min_distance( HRI_AGENT* robot, HRI_AGENT* human, double minDistPrev )
{
  p3d_rob* rob = robot->robotPt;
  
  hri_set_human_robot_dist( rob, human );
  
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
	    if( minDist > distances[i] )
	      {
		minDist = distances[i];
		k = i;
	      }
	  }
      
	set_kcd_which_test((p3d_type_col_choice)settings); // ROB_ALL + BOOL
      
	break;
      }
    case p3d_col_mode_pqp:
      {
	minDist =  pqp_robot_robot_distance(rob,human->robotPt, body[k], other[k]);
	break;
      }
    }
  
  if(hri_draw_distance)
    {
      if( minDist < minDistPrev )
	{
	  hri_disp_dist.clear();
	  hri_disp_dist.push_back(body[k][0]);
	  hri_disp_dist.push_back(body[k][1]);
	  hri_disp_dist.push_back(body[k][2]);
	  hri_disp_dist.push_back(other[k][0]);
	  hri_disp_dist.push_back(other[k][1]);
	  hri_disp_dist.push_back(other[k][2]);
	}
    }
  
  delete[] distances;
  delete[] other;
  delete[] body;
  
  return minDist;
}

//! Comptue the distance between the
//! robot and the closest human agent
double hri_robot_min_distance( HRI_AGENTS* agents )
{
  double minDist1 = std::numeric_limits<double>::max();
  double minDist2 = std::numeric_limits<double>::max();
  
  HRI_AGENT* robot = agents->robots[ agents->source_agent_idx ];
  
  for (int i=0; i<agents->humans_no; i++) 
    {
      HRI_AGENT* human = agents->humans[i];
    
      minDist2 = hri_robot_min_distance( robot, human, minDist1 );
    
      if( human->is_present == TRUE )
	{
	  if( minDist2 < minDist1 )
	    {
	      minDist1 = minDist2;
	    }
	}
    }
  
  return minDist1;
}

//!
double hri_compute_cost_from_dist( double distance )
{
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

//! Cost between 0 and 1 for the distance to the robot
//! the minimal distance is computed and a squaling is done to have
//! a cost in [0 1]
double hri_distance_cost( HRI_AGENTS* agents, double& distance )
{
  distance = hri_robot_min_distance(agents);
  return hri_compute_cost_from_dist( distance );
}

//!
double hri_distance_cost( HRI_AGENT* robot, HRI_AGENT* human, double& distance )
{
  distance = hri_robot_min_distance( robot, human );
  return hri_compute_cost_from_dist( distance );
}

//!
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
