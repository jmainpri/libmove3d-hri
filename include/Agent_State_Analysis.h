#ifndef _AGENT_STATE_ANALYSIS_H
#define _AGENT_STATE_ANALYSIS_H

#include <vector>
#include <P3d-pkg.h>

extern int get_index_of_robot_by_name(char *rob_name);


typedef struct Agent_State_Analysis_thresholds
{
 double maxi_num_prev_states_to_store;
 //// Following will be used for comparing two successive configurations
 double agents_whole_body_pos_tolerance;//in m, will be used for x,y,z
 double agents_whole_body_orient_tolerance;//in rad, will be used for yaw, pitch, roll
 
 double agents_torso_orient_tolerance;//in rad,will be used for yaw, pitch, roll
 
 double agents_head_orient_tolerance;//in rad, will be used for yaw, pitch, roll
 
 double agents_hand_pos_tolerance;//in m, will be used for x,y,z
 
 // Following will be used for temporal reasoning 
 double min_period_for_agent_is_moving;// in ms, will be used to calculate based on continuous change for this period
 double min_period_for_agent_is_not_moving;// in ms, will be used to calculate based on continuous not-change for this period
 
 double min_period_for_agent_is_turning;// in ms, will be used to calculate based on continuous change for this period
 double min_period_for_agent_is_not_turning;// in ms, will be used to calculate based on continuous not-change for this period
 
 double min_period_for_agent_head_is_turning;// in ms, will be used to calculate based on continuous change for this period
 double min_period_for_agent_head_is_not_turning;// in ms, will be used to calculate based on continuous not-change for this period
 
 double min_period_for_agent_hand_is_moving;// in ms, will be used to calculate based on continuous change for this period
 double min_period_for_agent_hand_is_not_moving;// in ms, will be used to calculate based on continuous not-change for this period
 
}Agent_State_Analysis_thresholds;



typedef struct agents_state_diff
{
  int agent_has_moved;
  int agent_torso_has_turned;
  int agent_head_has_turned;
  int agent_R_hand_has_moved;
  int agent_L_hand_has_moved;
  
}agents_state_diff;

typedef struct agents_state_conti_diff_info
{
  double agent_has_moved_conti_for_period;
  double agent_torso_has_turned_conti_for_period;
  double agent_head_has_turned_conti_for_period;
  double agent_R_hand_has_moved_conti_for_period;
  double agent_L_hand_has_moved_conti_for_period;
  
  double agent_has_not_moved_conti_for_period;
  double agent_torso_has_not_turned_conti_for_period;
  double agent_head_has_not_turned_conti_for_period;
  double agent_R_hand_has_not_moved_conti_for_period;
  double agent_L_hand_has_not_moved_conti_for_period;
  
}agents_state_conti_diff_info;


typedef enum agents_motion_status
{
  AGENT_MOVING=0,
  AGENT_STATIC,
  AGENT_HAS_MOVED,
  
  AGENT_MOTION_STATUS_UNKNOWN
  
}agents_motion_status;

typedef enum agents_head_status
{
  AGENT_HEAD_MOVING=0,
  AGENT_HEAD_STATIC,
  AGENT_HEAD_HAS_MOVED,
  
  AGENT_HEAD_STATUS_UNKNOWN
}agents_head_status;


typedef enum agents_hand_status
{
  AGENT_HAND_MOVING=0,
  AGENT_HAND_STATIC,
  AGENT_HAND_HAS_MOVED,
  
  AGENT_HAND_STATUS_UNKNOWN
}agents_hand_status;

typedef enum agents_static_hand_mode
{
  AGENT_HAND_AT_REST_MODE=0,
  AGENT_HAND_AT_MANIPULATION_MODE, //due to the limited perception it would be difficult to distinguish between pointing, reaching to grasp, waiting to take etc.It would be better that based on the current context and task such facts would be computed at higher level
  
  AGENT_HAND_STATIC_MODE_UNKNOWN
}agents_static_hand_mode;

typedef enum agents_torso_status
{
  AGENT_TORSO_MOVING=0,
  AGENT_TORSO_STATIC,
  AGENT_TORSO_HAS_MOVED,
  
  AGENT_TORSO_STATUS_UNKNOWN
  
}agents_torso_status;


typedef struct agents_joint_indices_for_ASA
{
 int body_jnt;
 
 int torso_yaw;
 int torso_pitch;
 int torso_roll;
 
 int head_yaw;
 int head_pitch;
 int head_roll;
 
 int R_hand_jnt;
 
 int L_hand_jnt;
  
}agents_joint_indices_for_ASA;

typedef struct agents_Q_indices_for_ASA
{
 int body_Q_x;
 int body_Q_y;
 int body_Q_z;
 int body_Q_yaw;
 int body_Q_pitch;
 int body_Q_roll;
 
 int torso_Q_yaw;
 int torso_Q_pitch;
 int torso_Q_roll;
 
 int head_Q_yaw;
 int head_Q_pitch;
 int head_Q_roll;
 /*
 int R_hand_Q_x;
 int R_hand_Q_Y;
 int R_hand_Q_Z;
 int R_hand_Q_yaw;
 int R_hand_Q_pitch;
 int R_hand_Q_roll;
 
 int L_hand_Q_x;
 int L_hand_Q_Y;
 int L_hand_Q_Z;
 int L_hand_Q_yaw;
 int L_hand_Q_pitch;
 int L_hand_Q_roll;
  */
}agents_Q_indices_for_ASA;


typedef struct agents_info_for_ASA
{
  int agent_index;
  char agent_name[50]; 
  Agent_State_Analysis_thresholds ASA_threshold;
  agents_joint_indices_for_ASA joint_indx;
  agents_Q_indices_for_ASA Q_indx;
  
}agents_info_for_ASA;

typedef struct agents_activity_facts
{
   
   int agent_index; //in env list
   char agent_name[50];
  
   agents_motion_status whole_body;
   agents_torso_status torso;
   agents_head_status head;
   agents_hand_status right_hand;
   agents_hand_status left_hand;
   agents_static_hand_mode right_hand_mode;
   agents_static_hand_mode left_hand_mode;
   
}agents_activity_facts;

typedef struct agents_config_info_at_time
{
  ////float clock;
  ////time_t at_time;
  timeval at_time;
  double time_diff_from_prev_config;//in ms
  
  configPt config;
  p3d_matrix4 R_hand_pos;
  p3d_matrix4 L_hand_pos;
  
  agents_state_diff diff_prev_config;
  agents_state_conti_diff_info conti_diff_info;
  
}agents_config_info_at_time;

typedef struct agents_prev_conf_info
{ 
  agents_info_for_ASA *for_agent;
  std::vector <agents_config_info_at_time> agents_config_info;
  
}agents_prev_conf_info;

#endif