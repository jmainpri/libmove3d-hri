//
// C++ Interface: HRI_tasks
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@verdier.laas.fr>, (C) 2010
//
// Copyright: See COPYING file that comes with this distribution
//
//
//High Level action IDs
#ifndef _HRI_tasks_H
#define _HRI_tasks_H

#include "Mightability_Analysis.h"
////#define SECOND_HUMAN_EXISTS
extern void* (*XFORM_update_func)();
extern int (*default_drawtraj_fct_ptr)(p3d_rob* robot, p3d_localpath* curLp);

typedef enum HRI_TASK_TYPE_ENUM
{
MAKE_OBJECT_ACCESSIBLE=0,
SHOW_OBJECT,
GIVE_OBJECT,
HIDE_OBJECT,
TAKE_OBJECT,
PUT_AWAY_OBJECT,
HIDE_AWAY_OBJECT,
MAKE_SPACE_FREE_OF_OBJECT,
PUT_INTO_OBJECT,
REACH_TO_POINT,
//NOTE: Don't forget to add any new task in init_HRI_task_name_ID_map() also
//Add new tasks here before the last line

MAXI_NUM_OF_HRI_TASKS
}HRI_TASK_TYPE;


typedef enum HRI_SUB_TASK_TYPE
{
REACH_TO_TAKE=0,
REACH_TO_GRASP,
GRASP,
LIFT_OBJECT,
CARRY_OBJECT,
PUT_DOWN_OBJECT,
RELEASE_OBJECT,
RETREAT_HAND,
//NOTE: Don't forget to add any new task in init_HRI_task_name_ID_map() also
//Add new sub tasks here before the last line

MAXI_NUM_SUB_TASKS
}HRI_SUB_TASK_TYPE;


typedef struct HRI_task_desc
{
 HRI_TASK_TYPE task_type;
 
 std::string for_object;

 HRI_TASK_AGENT by_agent;
 HRI_TASK_AGENT for_agent;

}HRI_task_desc;

typedef struct world_state_configs
{
 //int no_robots;
 std::vector <configPt> robot_config;// robot_config[50]; //To store configurations of all the robots, it should be synchronized with the index of the robots
   
 
}world_state_configs;

typedef struct traj_for_HRI_sub_task
{
  HRI_TASK_AGENT traj_for_agent;
int armID;
HRI_SUB_TASK_TYPE sub_task_type;
p3d_traj* traj;
world_state_configs config_after_sub_task;
}traj_for_HRI_sub_task;

typedef struct traj_for_HRI_task
{
HRI_TASK_TYPE task_type;
std::vector<traj_for_HRI_sub_task> sub_task_traj;

}traj_for_HRI_task;

// typedef struct HRI_task_desc
// {
//  HRI_TASK_TYPE task_type;
//  
//  std::string for_object;
// 
//  HRI_TASK_AGENT_ENUM by_agent;
//  HRI_TASK_AGENT_ENUM for_agent;
// 
//  
// 
// }HRI_task_desc;


typedef struct HRI_task_node
{
 HRI_task_desc hri_task;
 int task_plan_id;
 world_state_configs before_task;
 world_state_configs after_task;
 
 traj_for_HRI_task traj;

}HRI_task_node;

typedef struct symbolic_HRI_task_desc
{
 char task_name[50];//See HRI_tasks.h for various types
 char for_object[50];

 char by_agent[50]; //see enum HRI_TASK_AGENT_ENUM in Mightability_Analysis.h
 char for_agent[50];//see enum HRI_TASK_AGENT_ENUM in Mightability_Analysis.h

}symbolic_HRI_task_desc;

typedef struct grasp_lift_info
{
  ////std::vector <p3d_traj*> take_trajs;
  p3d_traj* reach_traj;
  p3d_traj* pre_grasp_traj;
  p3d_traj* grasp_traj;
  
  configPt graspConf;
  configPt liftConf;
  
}grasp_lift_info;

typedef struct taskability_node
{
  int performing_agent;
  int target_agent;
  int target_object;//index of target obj in envPt
  int task;
  int performing_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  int target_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  candidate_poins_for_task *candidate_points;
  
  int node_id;// It should be unique
  char desc[200];
}taskability_node;




#endif