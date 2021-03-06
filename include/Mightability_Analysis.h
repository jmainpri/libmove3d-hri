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
#ifndef _MIGHTABILITY_MAP_H
#define _MIGHTABILITY_MAP_H
//////////#include <map>

////#include "HRI_tasks.h"
////#include "hri_agent.h"

// **** AKP : Structures for storing different surfaces in the environment
//typedef struct bitmap_set hri_bitmapset;


#define MM_SHOW_DEBUG_MODE_BUTTONS

#define HUMAN1_EXISTS_FOR_MA
//#define HUMAN2_EXISTS_FOR_MA
////#define JIDO_EXISTS_FOR_MA
#define PR2_EXISTS_FOR_MA
////#define BERT_EXISTS_FOR_MA
//#define USE_HH_LEARNING
////#define USE_SYM_GEO_PLAN

//#define STORE_STATE_CONFIGS//Important NOTE: use it only to debug, comment this define for real robot run because of excessive memory consumption which might eventually lead to memory corruption. //Used to store different configs of the agent corresponding to the Mightability computation. NOTE: IMPORTANT:Set this flag as 0 also to save memory and speedup the Mightability updation and calculation process;

#define MAXI_NUM_OF_ALLOWED_OBJECTS_IN_ENV 100
#define COMMENT_TMP

#define UNKNOWN_OBJ_POS_Z -15 //This will be used to set the z value of some object which is not perceivable by the robot. 

//Set operators on Mightability Maps
#define MM_SET_OPR_NONE 0
#define MM_SET_OPR_OR 1
#define MM_SET_OPR_AND 2
#include <boost/concept_check.hpp>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>


extern p3d_vector3 to_reach_target;
extern struct grid_3D grid_around_HRP2;
extern int HRP2_GIK_MANIP;// Just to set the type of the bitmap
extern int HRP2_GIK_path_calculated;
extern int Affordances_Found; 

extern int grid_3d_affordance_calculated;
extern int HRP2_CURRENT_STATE;//1 for sitting, 2 for half sitting
extern int SHOW_OBSTACLE_CELLS;
extern int CANDIDATE_POINTS_FOR_TASK_FOUND;

extern p3d_vector3 point_to_look;


#define MAX_CANDIDATE_POINT_FOR_TASK 15000
#define MAX_POS_STATE_OF_AGENT_MM 15

// AKP : structure to store x-y co ordinates of point
typedef struct point_co_ordi
{
 double x;
 double y;
 double z;
 double theta;
 
 bool operator<(const point_co_ordi &a) const 
 { 
   if(x<a.x) 
     return true;
   
   if(x>a.x) 
     return false;
   
   if(y<a.y) 
     return true;
   
   if(y>a.y) 
     return false;
   
   if(z<a.z) 
     return true;
   
   return false;
   
 }
 
 //overload operator== for the std:set to work, i.e. to be able to find correctly, as it need to compare the values to sort while inserting
 bool operator==(const point_co_ordi &a) const 
 { 
   if(x==a.x && y==a.y && z==a.z) 
     return true;
   
   return false;
   
 }
 
}point_co_ordi;

//To store the indices to identify a particular cell in the 3D grid
typedef struct cell_X_Y_Z
{
 int x;
 int y;
 int z;
 
 bool valid;
 
 cell_X_Y_Z()
 {
   valid=false;
 }
 //overload operator< for the std:set to work, i.e. to be able to insert, as it need to compare the values to sort while inserting
 bool operator<(const cell_X_Y_Z &a) const 
 { 
   if(x<a.x) 
     return true;
   
   if(x>a.x) 
     return false;
   
   if(y<a.y) 
     return true;
   
   if(y>a.y) 
     return false;
   
   if(z<a.z) 
     return true;
   
   return false;
   
 }
 
 //overload operator== for the std:set to work, i.e. to be able to find correctly, as it need to compare the values to sort while inserting
 bool operator==(const cell_X_Y_Z &a) const 
 { 
   if(x==a.x && y==a.y && z==a.z) 
     return true;
   
   return false;
   
 }
}cell_X_Y_Z;

typedef enum HRI_TASK_AGENT_ENUM
{
  #ifdef HUMAN1_EXISTS_FOR_MA
  HUMAN1_MA=0,
#endif
  
#ifdef HUMAN2_EXISTS_FOR_MA
  HUMAN2_MA,
#endif
  
#ifdef JIDO_EXISTS_FOR_MA
  JIDO_MA,
#endif
  
#ifdef HRP2_EXISTS_FOR_MA
  HRP2_MA,
#endif
  
#ifdef PR2_EXISTS_FOR_MA  
  PR2_MA,
#endif
  
#ifdef BERT_EXISTS_FOR_MA  
  BERT_MA,
#endif
  //NOTE: DO NOT FORGET to add new agents in the function init_HRI_agent_name_ID_map() also
  //Add new performers here before the last line// TODO: Locate init_HRI_agent_name_ID_map(), disappeared mysteriously 

  MAXI_NUM_OF_AGENT_FOR_HRI_TASK
}HRI_TASK_AGENT;

////extern std::map<std::string,int> HriTaskAgent;


typedef enum HRP2_STATES_HRI
{
  HRP2_STANDING=0,
  HRP2_SITTING,
  HRP2_HALF_SITTING,
  
}HRP_STATES_HRI;

typedef enum PR2_POSTURE_MA
{
 PR2_LOW_MA=0,
 PR2_ARBITRARY_MA,
 PR2_HIGH_MA
}PR2_POSTURE_MA;

typedef enum MM_STATES_FOR_REACH_HUMAN
{
 MM_CURRENT_STATE_HUM_REACH=0,
 MM_SITTING_STRAIGHT_STATE_HUM_REACH,
 MM_SITTING_LEAN_FORWARD_STATE_HUM_REACH,
 MM_SITTING_TURN_AROUND_STATE_HUM_REACH,
 MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_REACH,
 MM_STANDING_STRAIGHT_STATE_HUM_REACH,
 MM_STANDING_LEAN_FORWARD_STATE_HUM_REACH,
 MM_STANDING_TURN_AROUND_STATE_HUM_REACH,
 MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_REACH,
 MM_ARBITRARY_STATE_HUM_REACH,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN
}MM_STATES_FOR_REACH_HUMAN;


typedef enum MM_STATES_FOR_REACH_HRP2
{
 MM_CURRENT_STATE_HRP2_REACH=0,
 MM_SITTING_LEAN_FORWARD_STATE_HRP2_REACH,
 MM_SITTING_TURN_AROUND_STATE_HRP2_REACH,
 MM_SITTING_TURN_AROUND_LEAN_STATE_HRP2_REACH,
 MM_STANDING_LEAN_FORWARD_STATE_HRP2_REACH,
 MM_STANDING_TURN_AROUND_STATE_HRP2_REACH,
 MM_STANDING_TURN_AROUND_LEAN_STATE_HRP2_REACH,
 MM_ARBITRARY_STATE_HRP2_REACH,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_REACH_HRP2
}MM_STATES_FOR_REACH_HRP2;

typedef enum MM_STATES_FOR_REACH_JIDO
{
 MM_CURRENT_STATE_JIDO_REACH=0,
 MM_TURN_AROUND_STATE_JIDO_REACH,
 MM_ARBITRARY_STATE_JIDO_REACH,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_REACH_JIDO
}MM_STATES_FOR_REACH_JIDO;

typedef enum MM_STATES_FOR_REACH_PR2
{
 MM_CURRENT_STATE_PR2_REACH=0,
 MM_LOW_REACH_STATE_PR2,
 MM_LOW_TURN_AROUND_STATE_PR2_REACH,
 MM_HIGH_REACH_STATE_PR2,
 MM_HIGH_TURN_AROUND_STATE_PR2_REACH,
 
 MM_ARBITRARY_STATE_PR2_REACH,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_REACH_PR2
}MM_STATES_FOR_REACH_PR2;

typedef enum MM_STATES_FOR_REACH_ARBITRARY_ROBOT
{
 MM_CURRENT_STATE_AR_REACH=0,
 MM_STRAIGHT_STATE_AR_REACH,
 MM_LEAN_FORWARD_STATE_AR_REACH,
 MM_TURN_AROUND_STATE_AR_REACH,
 MM_TURN_AROUND_LEAN_STATE_AR_REACH,
 MM_ARBITRARY_STATE_AR_REACH,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_REACH_AR

}MM_STATES_FOR_REACH_ARBITRARY_ROBOT;

typedef enum MM_STATES_FOR_VIS_ARBITRARY_ROBOT
{
 MM_CURRENT_STATE_AR_VIS=0,
 MM_STRAIGHT_HEAD_STATE_AR_VIS,
 MM_LOOK_AROUND_HEAD_STATE_AR_VIS,
 MM_LEAN_FORWARD_STATE_AR_VIS,
 
 
 MM_ARBITRARY_STATE_AR_VIS,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_VIS_AR
}MM_STATES_FOR_VIS_ARBITRARY_ROBOT;

typedef enum MM_STATES_FOR_VIS_HUMAN
{
 MM_CURRENT_STATE_HUM_VIS=0,
 MM_SITTING_STRAIGHT_HEAD_STATE_HUM_VIS,
 MM_SITTING_LOOK_AROUND_HEAD_STATE_HUM_VIS,
 MM_SITTING_LEAN_FORWARD_STATE_HUM_VIS,
 ////MM_SITTING_TURN_AROUND_STATE_HUM_VIS,
 ////MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_VIS,
 MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS,
 MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS,
 MM_STANDING_LEAN_FORWARD_STATE_HUM_VIS,
 ////MM_STANDING_TURN_AROUND_STATE_HUM_VIS,
 ////MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_VIS,
 
 MM_ARBITRARY_STATE_HUM_VIS,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN
}MM_STATES_FOR_VIS_HUMAN;

typedef enum MM_STATES_FOR_VIS_HRP2
{
 MM_CURRENT_STATE_HRP2_VIS=0,
 MM_SITTING_STRAIGHT_HEAD_STATE_HRP2_VIS,
 MM_SITTING_LOOK_AROUND_HEAD_STATE_HRP2_VIS,
 MM_SITTING_LEAN_FORWARD_STATE_HRP2_VIS,
 ////MM_SITTING_TURN_AROUND_STATE_HUM_VIS,
 ////MM_SITTING_TURN_AROUND_LEAN_STATE_HUM_VIS,
 //////MM_STANDING_STRAIGHT_HEAD_STATE_HUM_VIS,
 //////MM_STANDING_LOOK_AROUND_HEAD_STATE_HUM_VIS,
 //////MM_STANDING_LEAN_FORWARD_STATE_HUM_VIS,
 ////MM_STANDING_TURN_AROUND_STATE_HUM_VIS,
 ////MM_STANDING_TURN_AROUND_LEAN_STATE_HUM_VIS,
 
 MM_ARBITRARY_STATE_HRP2_VIS,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_VIS_HRP2
}MM_STATES_FOR_VIS_HRP2;


typedef enum MM_STATES_FOR_VIS_JIDO
{
 MM_CURRENT_STATE_JIDO_VIS=0,
 MM_STRAIGHT_HEAD_STATE_JIDO_VIS,
 MM_LOOK_AROUND_HEAD_STATE_JIDO_VIS,
  
 MM_ARBITRARY_STATE_JIDO_VIS,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_VIS_JIDO
}MM_STATES_FOR_VIS_JIDO;

typedef enum MM_STATES_FOR_VIS_PR2
{
 MM_CURRENT_STATE_PR2_VIS=0,
 MM_CURRENT_POST_STRAIGHT_HEAD_STATE_PR2_VIS,
 MM_CURRENT_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS,
 MM_LOW_POST_STRAIGHT_HEAD_STATE_PR2_VIS,
 MM_LOW_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS,
 MM_HIGH_POST_STRAIGHT_HEAD_STATE_PR2_VIS,
 MM_HIGH_POST_LOOK_AROUND_HEAD_STATE_PR2_VIS,
 MM_ARBITRARY_STATE_PR2_VIS,
 //Add new states before the last line 

 MAXI_NUM_POSSIBLE_STATES_VIS_PR2
}MM_STATES_FOR_VIS_PR2;

typedef enum MA_agent_hand_name
{ 
  MA_RIGHT_HAND=0,
  MA_LEFT_HAND,
  
  MAXI_NUM_OF_HANDS_OF_AGENT
}MA_agent_hand_name;

typedef enum human_head_joints_MA
{
  HUMAN_J_NECK_PAN=5,
  HUMAN_J_NECK_TILT=6,
  HUMAN_J_NECK_ROLL=7
}human_head_joints_MA;

typedef enum human_head_Qs_MA
{
  HUMAN_Q_NECK_PAN=15,
  HUMAN_Q_NECK_TILT=16,
  HUMAN_Q_NECK_ROLL=17
}human_head_Qs_MA;

typedef enum JIDO_head_joints_MA
{
  JIDO_J_NECK_PAN=2,
  JIDO_J_NECK_TILT=3
}JIDO_head_joints_MA;

typedef enum JIDO_head_Qs_MA
{
  JIDO_Q_NECK_PAN=12,
  JIDO_Q_NECK_TILT=13
}JIDO_head_Qs_MA;


typedef enum PR2_head_joints_MA
{
  PR2_J_NECK_PAN=3,
  PR2_J_NECK_TILT=4
}PR2_head_joints_MA;

typedef enum PR2_head_Qs_MA
{
  PR2_Q_NECK_PAN=13,
  PR2_Q_NECK_TILT=14
}PR2_head_Qs_MA;



typedef enum MA_agent_neck_joint_Q_names
{
  PAN=0,
  TILT,
  ROLL,
  MAX_NUM_NECK_J_Q_NAMES
}MA_agent_neck_joint_names;

typedef struct MA_agent_head_info
{
  int no_joints_neck;
  int no_Qs_neck;
  int joint_indices[MAX_NUM_NECK_J_Q_NAMES];
  int Q_indices[MAX_NUM_NECK_J_Q_NAMES];
}MA_agent_head_info;

typedef enum JIDO_hand_joints_MA
{
  JIDO_J_RSHOULDER=4
}JIDO_hand_joints_MA;


typedef enum human_hand_joints_MA
{
  HUMAN_J_RSHOULDER=8,
  HUMAN_J_LSHOULDER=15
  
}human_hand_joints_MA;

typedef enum PR2_hand_joints_MA
{
  PR2_J_RSHOULDER=6,
  PR2_J_LSHOULDER=16
  
}PR2_hand_joints_MA;

typedef enum MA_agents_hand_joints_names
{
 RSHOULDER=0,
 LSHOULDER,
 
 MAXI_NUM_HAND_JOINTS_MA
}MA_agents_hand_joints_names;

typedef struct MA_agent_hand_info
{
  
  int joint_indices[MAXI_NUM_HAND_JOINTS_MA];
  ////int Q_indices[MAX_NUM_NECK_J_Q_NAMES];
}MA_agent_hand_info;

typedef struct agent_prop_for_MA
{
  HRI_TASK_AGENT agent_type;
  MA_agent_head_info head_params;
  MA_agent_hand_info hand_params;
  
  int no_of_arms;//if it has only one hand it will be MA_RIGHT_HAND
  int maxi_num_vis_states;
  int maxi_num_reach_states;
  
}agent_prop_for_MA;

typedef struct agents_for_MA
{
  agent_prop_for_MA for_agent[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
  
  agents_for_MA()
  {
    for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     for_agent[i].agent_type=HRI_TASK_AGENT(i);
     switch(i)
     {
       case HUMAN1_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN;
       for_agent[i].no_of_arms=2;
       
       for_agent[i].head_params.no_joints_neck=3;
       /*
      for_agent[i].head_params.joint_indices[PAN]=HUMAN_J_NECK_PAN;
      for_agent[i].head_params.joint_indices[TILT]=HUMAN_J_NECK_TILT;
      for_agent[i].head_params.joint_indices[ROLL]=HUMAN_J_NECK_ROLL;
      */
      for_agent[i].head_params.no_Qs_neck=3;
      /*
      for_agent[i].head_params.Q_indices[PAN]=HUMAN_Q_NECK_PAN;
      for_agent[i].head_params.Q_indices[TILT]=HUMAN_Q_NECK_TILT;
      for_agent[i].head_params.Q_indices[ROLL]=HUMAN_Q_NECK_ROLL;
          
      for_agent[i].hand_params.joint_indices[RSHOULDER]=HUMAN_J_RSHOULDER;
      for_agent[i].hand_params.joint_indices[LSHOULDER]=HUMAN_J_LSHOULDER;
       */
       break;
#ifdef HUMAN2_EXISTS_FOR_MA
       case HUMAN2_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN;
       for_agent[i].no_of_arms=2;
       
       for_agent[i].head_params.no_joints_neck=3;
       /*
      for_agent[i].head_params.joint_indices[PAN]=HUMAN_J_NECK_PAN;
      for_agent[i].head_params.joint_indices[TILT]=HUMAN_J_NECK_TILT;
      for_agent[i].head_params.joint_indices[ROLL]=HUMAN_J_NECK_ROLL;
      */
      for_agent[i].head_params.no_Qs_neck=3;
      /*
      for_agent[i].head_params.Q_indices[PAN]=HUMAN_Q_NECK_PAN;
      for_agent[i].head_params.Q_indices[TILT]=HUMAN_Q_NECK_TILT;
      for_agent[i].head_params.Q_indices[ROLL]=HUMAN_Q_NECK_ROLL;

      for_agent[i].hand_params.joint_indices[RSHOULDER]=HUMAN_J_RSHOULDER;
      for_agent[i].hand_params.joint_indices[LSHOULDER]=HUMAN_J_LSHOULDER;
      */
       break;
#endif

#ifdef JIDO_EXISTS_FOR_MA
       case JIDO_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_JIDO;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_JIDO;
       for_agent[i].no_of_arms=1;
       
       for_agent[i].head_params.no_joints_neck=2;
       for_agent[i].head_params.joint_indices[PAN]=JIDO_J_NECK_PAN;
       for_agent[i].head_params.joint_indices[TILT]=JIDO_J_NECK_TILT;
      
       for_agent[i].head_params.no_Qs_neck=2;
       for_agent[i].head_params.Q_indices[PAN]=JIDO_Q_NECK_PAN;
       for_agent[i].head_params.Q_indices[TILT]=JIDO_Q_NECK_TILT;

       for_agent[i].hand_params.joint_indices[RSHOULDER]=JIDO_J_RSHOULDER;

       break;
#endif
#ifdef HRP2_EXISTS_FOR_MA 
       case HRP2_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_HRP2;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_HRP2;
       for_agent[i].no_of_arms=2;
       
       //TODO populate the Qs and joint indices of HRP2 for head and hand
       
       break;
#endif
       
#ifdef PR2_EXISTS_FOR_MA
        case PR2_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_PR2;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_PR2;
       for_agent[i].no_of_arms=2;
       
       for_agent[i].head_params.no_joints_neck=2;
      for_agent[i].head_params.joint_indices[PAN]=PR2_J_NECK_PAN;
      for_agent[i].head_params.joint_indices[TILT]=PR2_J_NECK_TILT;
      
      
      for_agent[i].head_params.no_Qs_neck=2;
      for_agent[i].head_params.Q_indices[PAN]=PR2_Q_NECK_PAN;
      for_agent[i].head_params.Q_indices[TILT]=PR2_Q_NECK_TILT;
     

      for_agent[i].hand_params.joint_indices[RSHOULDER]=PR2_J_RSHOULDER;
      for_agent[i].hand_params.joint_indices[LSHOULDER]=PR2_J_LSHOULDER;

       break;
#endif
       
#ifdef BERT_EXISTS_FOR_MA
        case BERT_MA:
       for_agent[i].maxi_num_reach_states=MAXI_NUM_POSSIBLE_STATES_REACH_AR;
       for_agent[i].maxi_num_vis_states=MAXI_NUM_POSSIBLE_STATES_VIS_AR;
       for_agent[i].no_of_arms=2;
       
       for_agent[i].head_params.no_joints_neck=2;
       
      for_agent[i].head_params.no_Qs_neck=2;
    
       break;
#endif
     }
     
   }
  }
  
}agents_for_MA;

static agents_for_MA agents_for_MA_obj;

typedef struct configPt_vect{

  ////std::vector<configPt> configPts;
  configPt configPts[15];
  int no_configs;
}configPt_vect;

////#define USE_VECTOR_FORM_FOR_CONFIGS

typedef struct Mightability_Map_set{

  int *visible[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j [i][j]
  int **reachable[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j by hand k, [i][j][k]
  
  //std::vector<configPt> *conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//To store configs for visibility
  ////std::vector<std::vector<std::vector<configPt> > > conf;//To store configs for visibility
#ifdef STORE_STATE_CONFIGS

#ifdef USE_VECTOR_FORM_FOR_CONFIGS
  std::vector<configPt> conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAX_POS_STATE_OF_AGENT_MM];//To store configs for visibility
#else
  configPt_vect conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAX_POS_STATE_OF_AGENT_MM];//To store configs for visibility
#endif

#ifdef USE_VECTOR_FORM_FOR_CONFIGS
  std::vector<configPt> **reach_conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//To store config for reachability
#else
 configPt_vect reach_conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAX_POS_STATE_OF_AGENT_MM][MAXI_NUM_OF_HANDS_OF_AGENT];//To store configs for reachability
#endif

  //std::vector<configPt_PTR_vect> conf[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
#endif
  
  ////MA_for_agents for_agent[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
/*  
// For Human

int reachable_by_human_LHand[MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN]; 
int reachable_by_human_RHand[MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN]; 
// int reachable_by_human_LHand_by_bending; 
// int reachable_by_human_RHand_by_bending; 
// int reachable_by_human_LHand_by_turning_around; 
// int reachable_by_human_RHand_by_turning_around;
// int reachable_by_human_LHand_by_turning_around_bending; 
// int reachable_by_human_RHand_by_turning_around_bending;  
// int reachable_by_human_LHand_by_standing; 
// int reachable_by_human_RHand_by_standing;
// int reachable_by_human_LHand_by_standing_bending; 
// int reachable_by_human_RHand_by_standing_bending;
// int reachable_by_human_LHand_by_standing_turning_around; 
// int reachable_by_human_RHand_by_standing_turning_around;
// int reachable_by_human_LHand_by_standing_turning_around_bending; 
// int reachable_by_human_RHand_by_standing_turning_around_bending;  
int visible_by_human[MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN];
// int visible_by_human_straight_head_orientation;
// int visible_by_human;
// int visible_by_human_neck_turn;
// int visible_by_human_torso_neack_turn;
// int visible_by_standing_human;
// int visible_by_standing_human_neck_turn;
// int visible_by_standing_human_torso_neack_turn;
// int visible_by_human_straight_head_orientation_standing;

#ifdef SECOND_HUMAN_EXISTS
// For Human2
int reachable_by_human2_LHand[MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN]; 
int reachable_by_human2_RHand[MAXI_NUM_POSSIBLE_STATES_REACH_HUMAN]; 
// int reachable_by_human2_LHand; 
// int reachable_by_human2_RHand; 
// int reachable_by_human2_LHand_by_bending; 
// int reachable_by_human2_RHand_by_bending; 
// int reachable_by_human2_LHand_by_turning_around; 
// int reachable_by_human2_RHand_by_turning_around;
// int reachable_by_human2_LHand_by_turning_around_bending; 
// int reachable_by_human2_RHand_by_turning_around_bending;  
// int reachable_by_human2_LHand_by_standing; 
// int reachable_by_human2_RHand_by_standing;
// int reachable_by_human2_LHand_by_standing_bending; 
// int reachable_by_human2_RHand_by_standing_bending;
// int reachable_by_human2_LHand_by_standing_turning_around; 
// int reachable_by_human2_RHand_by_standing_turning_around;
// int reachable_by_human2_LHand_by_standing_turning_around_bending; 
// int reachable_by_human2_RHand_by_standing_turning_around_bending;  
int visible_by_human2[MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN];
// int visible_by_human2_straight_head_orientation;
// int visible_by_human2;
// int visible_by_human2_neck_turn;
// int visible_by_human2_torso_neack_turn;
// int visible_by_standing_human2;
// int visible_by_standing_human2_neck_turn;
// int visible_by_standing_human2_torso_neack_turn;
// int visible_by_human2_straight_head_orientation_standing;
#endif




//For HRP2
int reachable_by_HRP2_LHand[MAXI_NUM_POSSIBLE_STATES_REACH_HRP2]; 
int reachable_by_HRP2_RHand[MAXI_NUM_POSSIBLE_STATES_REACH_HRP2]; 
int visible_by_HRP2[MAXI_NUM_POSSIBLE_STATES_VIS_HRP2];
// int visible_by_HRP2_straight_head_orientation;
// int visible_by_HRP2;
// int visible_by_HRP2_neck_turn;
// int visible_by_HRP2_torso_neck_turn;
// int visible_by_standing_HRP2;
// int visible_by_standing_HRP2_neck_turn;
// int visible_by_standing_HRP2_torso_neck_turn;
// int visible_by_HRP2_straight_head_orientation_standing;

//For JIDO
int reachable_by_JIDO_Hand[MAXI_NUM_POSSIBLE_STATES_REACH_JIDO]; 

int visible_by_JIDO[MAXI_NUM_POSSIBLE_STATES_VIS_JIDO];

// int visible_by_JIDO_straight_head_orientation;
// int visible_by_JIDO_neck_turn;
*/
/*
 Mightability_Map_set()//NOTE: It is not sufficient to allocate memory here. Allocate properly in create_Mightability_data_fields
 {
   ////static int mem_allo_ctr=0;
   conf.resize(MAXI_NUM_OF_AGENT_FOR_HRI_TASK);
   
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     
     
     visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     
     ////conf[i]=MY_ALLOC(std::vector<configPt>, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     
     conf[i].resize(agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     
     for(int k=0; k<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;k++)
     {
     visible[i][k]=-1;
     ////conf[i][k]=new std::vector<configPt>;
     //conf[i][k].clear();
     //conf[i][k]=NULL;
     conf[i][k].resize(0);
     }
     reachable[i]=MY_ALLOC(int*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
     reach_conf[i]=MY_ALLOC(std::vector<configPt>*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
     
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
     {
     reachable[i][j]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].no_of_arms);
     reach_conf[i][j]=MY_ALLOC(std::vector<configPt>, agents_for_MA_obj.for_agent[i].no_of_arms);
     
      for(int k=0;k< agents_for_MA_obj.for_agent[i].no_of_arms;k++)
      {
      reachable[i][j][k]=-1;
      ////reach_conf[i][j][k]=NULL;
      ////conf[i][k]=new std::vector<configPt>;
      }
     }
    /// printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
   }
  //// mem_allo_ctr++;
   ////printf("mem_allo_ctr=%d\n",mem_allo_ctr);
 }
 
*/

}Mightability_Map_set;



typedef struct surface_grid_cell{

// For Human
int reachable_by_human_LHand; 
int reachable_by_human_RHand; 
int visible;
int inside; // -1 means grid cell is outside the surface boundary, 1 means inside the surface boundary, because grid will be constructed based on the bounding box so even it may be inside box but not necessarily inside the actual boundary

int reachable_by_human_LHand_by_bending; 
int reachable_by_human_RHand_by_bending; 

int reachable_by_human_LHand_by_turning_around; 
int reachable_by_human_RHand_by_turning_around;

int reachable_by_human_LHand_by_turning_around_bending; 
int reachable_by_human_RHand_by_turning_around_bending;  

int reachable_by_human_LHand_by_standing; 
int reachable_by_human_RHand_by_standing;

int reachable_by_human_LHand_by_standing_bending; 
int reachable_by_human_RHand_by_standing_bending;


//For HRP2
int reachable_by_HRP2_LHand; 
int reachable_by_HRP2_RHand; 
int visible_by_HRP2;
 

}surface_grid_cell;

typedef struct flat_surface{
int surface_ID;
int no_vertices;
int surface_shape; //Polygon=1, circle=2, rectangle=3 etc
point_co_ordi vertices[10]; // for the case of polygon or rectangle or non circle NOTE : The vertices should be stored either in clockwise or in anticlockwise order for the purpose of detecting a point is inside or not
double BR_x_min; // min and max values of x and y of the bounding rectangle
double BR_x_max;
double BR_y_min;
double BR_y_max;
double BR_z;
point_co_ordi Bounding_Rectangle[4];  // It should be populated in the sequence of (x_min, y_min), (x_max,y_min), (x_max, y_max), (x_min, y_max)

double circle_centre; // for the case of circle
double radius; // for the case of circle

surface_grid_cell surf_grid[100][100]; // Max 100x100 cells
int grid_i_max;//The maximum valid 1st index of surf_grid[i][]
int grid_j_max;// The maximum valid 2nd index of surf_grid[][j]

}flat_surface;



typedef struct env_surfaces{
flat_surface flat_surf[100]; // Total no. of surfaces in the environment 
int total_no_of_surfaces; 

}env_surfaces;

typedef struct grid_Bounding_Box{
double min_x;
double max_x;
double min_y;
double max_y;
double min_z;
double max_z;
}grid_BB;

typedef struct grid_3D{
grid_Bounding_Box grid_BB;
////hri_bitmapset *GRID_SET;
struct bitmap_set *GRID_SET;
grid_3D()
 {
 GRID_SET = NULL;
 }
}grid_3D;

typedef struct candidate_poins_for_task{
point_co_ordi point[MAX_CANDIDATE_POINT_FOR_TASK];
double weight[MAX_CANDIDATE_POINT_FOR_TASK];
int status[MAX_CANDIDATE_POINT_FOR_TASK]; //0 is not tested for validity, 1 is accepted, 2 is rejected, 3 is found a valid solution for that individual task, but rejected when next task fails, i.e. rejected for the case of backtracking, 5 is tested and failed for the same task may be with lower effort level 
int curr_solution_point_index;//It will store the index by which the current solution for the task can be accessed directly from the array point.
int horizontal_surface_of[MAX_CANDIDATE_POINT_FOR_TASK];//To store the index of the object, horizontal surface of which belongs to the candidate point. 
int no_points; 
}candidate_poins_for_task;

typedef struct robots_status{
char rob_name[50];//This index should be synchronized with the indices of robots in env
//int is_robot;
////int rob_index;
configPt rob_prev_config;
////p3d_rob *rob_prev_config;
////p3d_rob *rob_prev_pos;

/*
double prev_x;
double prev_y;
double prev_z;

double curr_x;
double curr_y;
double curr_z;
*/
////p3d_BB prev_BB;
int has_moved;
int turned_head;

}object_status;

typedef struct Mightability_map_cell_object_info{

int belongs_to_objects_indx[50];//This index should be synchronized with the indices of robots in env, as it gives object having index i in env belongs to the cell or not
int objects_belonging_to[50];//It stores the indices of the objects belonging to this cell. This is redundant information, but required for fast access depending upon the case.//NOTE AKP Warning:: Don't use it. It is not fully implemented yet.

int no_belongs_to_objects;

int close_to_objects_indx[50];//This index should be synchronized with the indices of robots in env
int no_close_to_objects;

int belongs_to_object_BB_centre_indx[50];//This index should be synchronized with the indices of robots in env
int no_belongs_to_object_BB_centre;

//For belonging to flat surfaces
int is_horizontal_surface;
int near_horizontal_surface;
int surface_id;
int horizontal_surface_of;
double exact_z_val;

int is_table;

//For belonging to the first non visible cell 
  int *first_non_visible[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j [i][j]
  ////int *last_visible[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j [i][j]
  
  //NOTE: Below will hold valid values only in the case  if(STORE_STATE_CONFIGS==1), which is used to find the corresponding configuration for OOM. to speed up the process otherwise	   
  int *prev_visible_x[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
  int *prev_visible_y[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
  int *prev_visible_z[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
   
/*  
int first_non_visible_by_human[MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN];
// int first_non_visible_by_human_straight_head_orientation;
// int first_non_visible_by_human;
// int first_non_visible_by_human_neck_turn;
// int first_non_visible_by_human_torso_neack_turn;
// int first_non_visible_by_standing_human;
// int first_non_visible_by_standing_human_neck_turn;
// int first_non_visible_by_standing_human_torso_neack_turn;
// int first_non_visible_by_human_straight_head_orientation_standing; 

#ifdef SECOND_HUMAN_EXISTS
//For belonging to the first non visible cell of different type for human2
int first_non_visible_by_human2[MAXI_NUM_POSSIBLE_STATES_VIS_HUMAN];
// int first_non_visible_by_human2_straight_head_orientation;
// int first_non_visible_by_human2;
// int first_non_visible_by_human2_neck_turn;
// int first_non_visible_by_human2_torso_neack_turn;
// int first_non_visible_by_standing_human2;
// int first_non_visible_by_standing_human2_neck_turn;
// int first_non_visible_by_standing_human2_torso_neack_turn;
// int first_non_visible_by_human2_straight_head_orientation_standing; 

#endif

//For belonging to the first non visible cell of different type for HRP2
int first_non_visible_by_HRP2[MAXI_NUM_POSSIBLE_STATES_VIS_HRP2];
// int first_non_visible_by_HRP2_straight_head_orientation;
// int first_non_visible_by_HRP2;
// int first_non_visible_by_HRP2_neck_turn;
// int first_non_visible_by_HRP2_torso_neck_turn;
// int first_non_visible_by_standing_HRP2;
// int first_non_visible_by_standing_HRP2_neck_turn;
// int first_non_visible_by_standing_HRP2_torso_neck_turn;
// int first_non_visible_by_HRP2_straight_head_orientation_standing;

//For belonging to the first non visible cell of different type for JIDO
int first_non_visible_by_JIDO[MAXI_NUM_POSSIBLE_STATES_VIS_JIDO];
// int first_non_visible_by_JIDO;
// int first_non_visible_by_JIDO_straight_head_orientation;
// int first_non_visible_by_JIDO_neck_turn;
*/

Mightability_map_cell_object_info()//NOTE: It is not sufficient to allocate memory here. Allocate properly in create_Mightability_data_fields
 {
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     
     first_non_visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //last_visible[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
      prev_visible_x[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
      prev_visible_y[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
      prev_visible_z[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //prev_visible_x[i].resize(agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //prev_visible_y[i].resize(agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
     //prev_visible_z[i].resize(agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
   }
 }
}Mightability_map_cell_object_info;

typedef struct MM_based_task
{
 //Mightability_Map_set for_task;
 int is_true;
 int no_points;
 point_co_ordi point[200];

}MM_based_task;

typedef struct Symbolic_Mightability_Maps_Relations
{
 
 int reachable_for_grasping_to_carry_by_human;//For able to grasp, hold and carry
 #ifdef HUMAN2_EXISTS_FOR_MA
 int reachable_for_grasping_to_carry_by_human2;//For able to grasp, hold and carry
 #endif
 int reachable_for_grasping_to_carry_by_HRP2;//For able to grasp, hold and carry
 int reachable_for_grasping_to_carry_by_JIDO;//For able to grasp, hold and carry

//If at least one cell at the z_max of the bounding box of the object is reachable by agent, it is assumed that the agent could put somting inside that object if it is a container. Of course it should be filtered at higher level based on the size of the object to be put
 ////int reachable_for_putting_inside_by_human;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 ////int reachable_for_putting_inside_by_JIDO;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 ////int reachable_for_putting_inside_by_HRP2;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside

//If at least one cell at the z_max of the bounding box of the object is reachable by agent, it is assumed that the agent could put somting inside that object if it is a container. Of course it should be filtered at higher level based on the size of the object to be put
 MM_based_task reachable_for_putting_inside_by_human;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 #ifdef HUMAN2_EXISTS_FOR_MA
 MM_based_task reachable_for_putting_inside_by_human2;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 #endif
 MM_based_task reachable_for_putting_inside_by_JIDO;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 MM_based_task reachable_for_putting_inside_by_HRP2;//For able to reach sufficient area at the top of the object, so that something could be drop/put inside
 
//If Any cell which is occupied AND which belongs to that object is reachable by agent, then we assume agent can at least touch the object
 int reachable_for_touching_by_human;//For able to just touch the bounding box of the object
 #ifdef HUMAN2_EXISTS_FOR_MA
 int reachable_for_touching_by_human2;//For able to just touch the bounding box of the object
 #endif
 int reachable_for_touching_by_HRP2;//For able to just touch the bounding box of the object
 int reachable_for_touching_by_JIDO;//For able to just touch the bounding box of the object

}Symbolic_Mightability_Maps_Relations;
 

typedef struct object_mightability_set
{
 Mightability_Map_set geo_MM;
 Symbolic_Mightability_Maps_Relations sym_MM;
}object_mightability_set;

typedef struct object_Symbolic_Mightability_Maps_Relation
{
 //int object_indx[50]; //This index should be synchronized with the indices of robots in env
 //////////char object_name[50];
 ////Symbolic_Mightability_Maps_Relations MM_Relations;
 ////////Mightability_Map_set object[50];//This index should be synchronized with the indices of robots in env
 object_mightability_set object[50];//This index should be synchronized with the indices of robots in env

 
 
} object_Symbolic_Mightability_Maps_Relation;

typedef struct robots_indices
{
int HRP2_ROBOT;
int JIDO_ROBOT;
int YELLOW_BOTTLE;
int HUMAN;
int HUMAN2;
int VISBALL_MIGHTABILITY;
} robots_indices;



////**** structures for exporting the object level mightability information to the external modules
typedef struct object_mightabilities
{
  int is_reachable_by[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
  int is_visible_by[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
  /*
int is_reachable_by_human;
int is_visible_by_human;
#ifdef SECOND_HUMAN_EXISTS
int is_reachable_by_human2;
int is_visible_by_human2;
#endif
#ifdef HRI_JIDO
int is_reachable_by_JIDO;
int is_visible_by_JIDO;
#endif
#ifdef HRI_HRP2
int is_reachable_by_HRP2;
int is_visible_by_HRP2;
#endif
*/
}object_mightabilities;

typedef struct object_mightabilities_info
{
char object_name[50];//Name of robot(movable objects)
int object_index;//Index of robot (movable objects) in env
object_mightabilities obj_mightability;
int total_no_obj;// Total number of robot(movable objects)

}object_mightabilities_info;

typedef struct object_mightabilities_info_set
{
object_mightabilities_info object[100];//all the robot(movable objects)
int total_no_obj;// Total number of robot(movable objects)

}object_mightabilities_info_set;

typedef struct occluding_obj
{
 //////int occluding_obj_index[50];
 int occluding_ctr[50];//To keep track of no. of cells which are occluded, NOTE the index should be synchronized with the robot index
}occluding_obj;

typedef struct occluding_obj_info
{
 int occluding_obj_index;
 char occluding_obj_name[50];
 int occluded_cell_ctr;//To keep track of no. of cells which are occluded, NOTE the index should be synchronized with the robot index
}occluding_obj_info;

typedef struct agent_state_task_constraint
{
  int accepted_reach[MAX_POS_STATE_OF_AGENT_MM];//It stores id decleared in various the enum
  int no_accepted_reach_states;
  int accepted_visibility[MAX_POS_STATE_OF_AGENT_MM];//It stores id decleared in various the enum
  int no_accepted_vis_states;
  
  int non_accepted_reach[MAX_POS_STATE_OF_AGENT_MM];//It stores id decleared in various the enum
  int no_non_accepted_reach_states;
  int non_accepted_visibility[MAX_POS_STATE_OF_AGENT_MM];//It stores id decleared in various the enum
  int no_non_accepted_vis_states;
}agent_state_task_constraint;

typedef struct flags_show_Mightability_Maps
{
  int *show_visibility[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j [i][j], value will be 1-5 for different colors
  int **show_reachability[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];//for agent i for state j by hand k, [i][j][k], value will be 1-5 for different colors
  

 flags_show_Mightability_Maps()
  {
  //// static int mem_allo_ctr=0;
   for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
   {
     
     
     show_visibility[i]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
    
     show_reachability[i]=MY_ALLOC(int*, agents_for_MA_obj.for_agent[i].maxi_num_reach_states);
     
     for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
     {
     show_reachability[i][j]=MY_ALLOC(int, agents_for_MA_obj.for_agent[i].no_of_arms);
      
     }
     ///printf(" allocated memory for agent %d Mightability, agents_for_MA_obj.for_agent.maxi_num_vis_states=%d \n",i,agents_for_MA_obj.for_agent[i].maxi_num_vis_states);
   }
   ///mem_allo_ctr++;
   ///printf("mem_allo_ctr=%d\n",mem_allo_ctr);
 }
}flags_show_Mightability_Maps;
/*
typedef struct agent_agent_task_constraint
{
    HRI_TASK_TYPE_ENUM task_type; //It should be from the enum HRI_TASK_TYPE_ENUM
    int performer_agent_type; //it should be from the enum HRI_AGENT_TYPE_ENUM
    int performer_agent_index;//should be synchronized with robot index in the environment
    int target_agent_type; //it should be from the enum HRI_AGENT_TYPE_ENUM
    int target_agent_index;//should be synchronized with robot index in the environment
    
}
*/



// typedef struct world_state_configs
// {
//  //int no_robots;
//  std::vector <configPt> robot_config;// robot_config[50]; //To store configurations of all the robots, it should be synchronized with the index of the robots
//    
//  
// }world_state_configs;
// 
// typedef struct traj_for_HRI_sub_task
// {
// int armID;
// HRI_SUB_TASK_TYPE sub_task_type;
// p3d_traj* traj;
// world_state_configs config_after_sub_task;
// }traj_for_HRI_sub_task;
// 
// typedef struct traj_for_HRI_task
// {
// HRI_TASK_TYPE task_type;
// std::vector<traj_for_HRI_sub_task> sub_task_traj;
// 
// }traj_for_HRI_task;
// 
// // typedef struct HRI_task_desc
// // {
// //  HRI_TASK_TYPE task_type;
// //  
// //  std::string for_object;
// // 
// //  HRI_TASK_AGENT_ENUM by_agent;
// //  HRI_TASK_AGENT_ENUM for_agent;
// // 
// //  
// // 
// // }HRI_task_desc;
// 
// 
// typedef struct HRI_task_node
// {
//  HRI_task_desc hri_task;
//  int task_plan_id;
//  world_state_configs before_task;
//  world_state_configs after_task;
//  
//  traj_for_HRI_task traj;
// 
// }HRI_task_node;


typedef enum MA_transition_vis_effort_type 
{
 MA_NO_VIS_EFFORT=0,
 MA_HEAD_EFFORT,
 MA_HEAD_TORSO_EFFORT,// for PR2 it will correspond to turn around
 MA_WHOLE_BODY_CURR_POS_EFFORT_VIS, // for PR2 it will correspond to switch the posture between high and low 
 MA_WHOLE_BODY_CHANGE_POS_EFFORT_VIS,
 
  //NOTE:Define any other level here and don't forget to add them in the mapping list in init_effort_name_ID_map_for_human
 
 MA_NO_POSSIBLE_VIS_EFFORT,
 
 
 MA_MAXI_NUM_TRANS_VIS_EFFORTS
  
}MA_transition_vis_effort_type;

typedef enum MA_transition_reach_effort_type 
{
 MA_NO_REACH_EFFORT=0,
 MA_ARM_EFFORT,
 MA_ARM_TORSO_EFFORT,// for PR2 it will correspond to turn around
 MA_WHOLE_BODY_CURR_POS_EFFORT_REACH, // for PR2 it will correspond to switch the posture between high and low 
 MA_WHOLE_BODY_CHANGE_POS_EFFORT_REACH,
 
 //NOTE:Define any other level here and don't forget to add them in the mapping list in init_effort_name_ID_map_for_human
 
 MA_NO_POSSIBLE_REACH_EFFORT,
 
 
 MA_MAXI_NUM_TRANS_REACH_EFFORT
  
}MA_transition_reach_effort_type;

typedef struct HRI_task_agent_effort_level
{
 int performing_agent;
 int target_agent;
 int effort_for_agent;
 int task;
 
 int vis_relevent;
 int reach_relevant;
 
 MA_transition_vis_effort_type maxi_vis_accept;
 MA_transition_reach_effort_type maxi_reach_accept;
 MA_transition_vis_effort_type mini_non_vis_req;
 MA_transition_reach_effort_type mini_non_reach_req;
 
}HRI_task_agent_effort_level;

typedef enum ability_type_for_effort_level
{
 VIS_ABILITY=0,
 REACH_ABILITY,
 
 MAXI_NUM_ABILITY_TYPE_FOR_EFFORT
}ability_type_for_effort_level;

typedef struct analysis_type_effort_level_group
{
 
 int num_analysis_types;
 int analysis_types[50];
  
}analysis_type_effort_level_group;

typedef struct agent_ability_effort_tuple
{
  int agent;//Should be the element of HRI_TASK_AGENT
  int ability_type;//should be the element of ability_type_for_effort_level
  int effort_level;//should be the element of MA_transition_reach_effort_type or MA_transition_vis_effort_type 
  int by_hand;//In the case the ability type is REACH_ABILITY. Should be the element of MA_agent_hand_name
  int val; //1 or 0 for true or false 
  int operation_type; //1 for union, 2 for intersection. This is optional and will be used to indicate in the cases where the set operations are performed on a ordered list of this structure
  bool on_plane; //optional, used to filter the resultant set based on support planes
}agent_ability_effort_tuple;

typedef struct agent_ability_effort_set
{
  agent_ability_effort_tuple element[100];
  int num_elements;
  
}agent_ability_effort_set;

typedef struct show_taskability_params
{
 int show_all_taskability_graph;
 int show_all_manipulability_graph;
 
 int show_taskability_node;
 int show_TN_edge;
 int show_TN_candidates;
 
 int show_TN_by_node_ID;
 int TN_ID;
 
 int show_TN_by_agent;
 int TN_perf_ag;
 int TN_targ_ag;
 int TN_task;
 
 int show_manipulability_node;
 int show_MN_by_node_ID;
 int MN_ID;
 
 int show_MN_by_agent;
 int MN_perf_ag;
 int MN_targ_obj;
 
}show_taskability_params;

typedef struct agent_effort_configs
{
  std::vector <configPt> configs;
  std::vector <int> by_hand[MAXI_NUM_OF_HANDS_OF_AGENT]; //Index of this vector should be synchronized with the index of configs.
  int effort_level;
  int analysis_state;
  
  
}agent_effort_configs;


typedef struct object_putinto_points
{
 int object_index;
 std::vector <point_co_ordi> points; 
 int need_updation;
}object_putinto_points;


#endif
