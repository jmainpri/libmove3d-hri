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

typedef enum HRI_TASK_TYPE_ENUM
{
MAKE_OBJECT_ACCESSIBLE=0,
SHOW_OBJECT,
GIVE_OBJECT,
HIDE_OBJECT,
PUT_AWAY_OBJECT,
HIDE_AWAY_OBJECT,
MAKE_SPACE_FREE_OF_OBJECT_OBJ,
PUT_INTO_OBJECT,
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

#endif