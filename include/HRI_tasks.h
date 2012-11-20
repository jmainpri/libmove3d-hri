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
extern bool (*ext_hrics_init_otp)(std::string humanName);
extern bool (*ext_hrics_compute_otp)(std::string humanName, std::vector<std::vector<double> >& traj, configPt& handConf,bool isStanding, double objectNessecity);

typedef enum HRI_TASK_TYPE_ENUM
{
MAKE_OBJECT_ACCESSIBLE=0,
SHOW_OBJECT,
GIVE_OBJECT,
HIDE_OBJECT,
TAKE_OBJECT,//for hand over task
GRASP_PICK_OBJECT,//for picking from a support
SEE_OBJECT,//To see an object
PUT_AWAY_OBJECT,
HIDE_AWAY_OBJECT,
MAKE_SPACE_FREE_OF_OBJECT,
PUT_INTO_OBJECT,
REACH_TO_POINT,
PUT_ONTO_OBJECT,
REACH_OBJECT,
PICK_MAKE_OBJECT_ACCESSIBLE,
PICK_SHOW_OBJECT,
PICK_GIVE_OBJECT,
PICK_HIDE_OBJECT,
PUT_VISIBLE_OBJECT,
PICK_PUT_VISIBLE_OBJECT,
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
 std::string for_container;//Used for put into tasks

 HRI_TASK_AGENT by_agent;
 HRI_TASK_AGENT for_agent;
 
 std::vector<agent_ability_effort_tuple> ag_ab_effort;//to store all the desired constraints for this task wrt all the agent, not only the performing or target agent.
 //TODO: use above as the actual set of constraints for planning a task
 
}HRI_task_desc;

typedef struct world_state_configs
{
 //int no_robots;
 int world_state_id;
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
  int performing_agent;//from HRI_TASK_AGENT_ENUM
  int target_agent;//from HRI_TASK_AGENT_ENUM
  int target_object;//index of target obj in envPt
  int task;
  int performing_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  int target_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  candidate_poins_for_task *candidate_points;
  
  int node_id;// It should be unique
  char desc[200];
}taskability_node;

typedef struct ability_graph
{
  int graph_id;
std::vector<taskability_node> graph;
  
}ability_graph;

typedef struct graph_vertex
{
 int Ag_or_obj_index; //if Ag index, it will be from HRI_TASK_AGENT_ENUM, if obj_index, it will be index of target obj in envPt. In case of this node represents a space, the id will be assigned through a counter IMPORTANT NOTE:It is important to use the vert_type in couple with this index to get the correct vertex because for different vertex type same value of index can be assigned 
 int vert_type;//1 for agent, 2 for object, 3 for space// NOTE: Any vertex of type 3 will have unique property that they will have only one in vertex and only one out vertex
 
 //To store the position of the vertex for visualization
 double x;
 double y;
 double z;
 
}graph_vertex;

typedef struct graph_edge
{
 double performing_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
 double target_ag_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
 int no_candidate; //to store no. of points or grasps depending upon Taskability, Manipulability graph or Put Into Ability graph
 int edge_task_type;
 int agent_role_for_edge; //1 for performing agent, 2 for target agent
 
 double weight_for_graph_search;
 double weight_for_graph_search_to_restore;
}graph_edge;

typedef struct agent_temporal_occupancy
{
  int time_slot_id;
  std::vector<int> agent_id;
}agent_temporal_occupancy;

typedef struct World_state_change_explanation
{
 int object_id;
 std::string object_name;
 std::string action_sequence;
 
}World_state_change_explanation;

typedef struct World_State_Changes
{
  int WS1_id;
  int WS2_id;
  std::vector<std::string> physical_changes;
  std::vector<std::string> ability_changes;
  std::vector<World_state_change_explanation> explanations;
  
}World_State_Changes;

typedef boost::adjacency_list< boost::vecS, boost::vecS, boost::bidirectionalS, graph_vertex, graph_edge > MY_GRAPH;
typedef boost::graph_traits<MY_GRAPH>::vertex_descriptor MY_VERTEX_DESC;
typedef boost::graph_traits<MY_GRAPH>::edge_descriptor MY_EDGE_DESC;



#endif
