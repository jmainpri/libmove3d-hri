#ifndef _AGENT_STATE_ANALYSIS_H
#define _AGENT_STATE_ANALYSIS_H

#include <vector>
#include <P3d-pkg.h>

extern int get_index_of_robot_by_name(char *rob_name);

/*
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
*/

typedef enum Agent_State_Analysis_thresholds
{
  ASA_maxi_num_prev_states_to_store=0,
 //// Following will be used for comparing two successive configurations
 ASA_agents_whole_body_pos_tolerance,//in m, will be used for x,y,z
 ASA_agents_whole_body_orient_tolerance,//in rad, will be used for yaw, pitch, roll
 
 ASA_agents_torso_orient_tolerance,//in rad,will be used for yaw, pitch, roll
 
 ASA_agents_head_orient_tolerance,//in rad, will be used for yaw, pitch, roll
 
 ASA_agents_hand_pos_tolerance,//in m, will be used for x,y,z
 
 // Following will be used for temporal reasoning 
 ASA_min_period_for_agent_is_moving,// in ms, will be used to calculate based on continuous change for this period
 ASA_min_period_for_agent_is_not_moving,// in ms, will be used to calculate based on continuous not-change for this period
 
 ASA_min_period_for_agent_whole_body_is_turning,// in ms, will be used to calculate based on continuous change for this period
 ASA_min_period_for_agent_whole_body_is_not_turning,// in ms, will be used to calculate based on continuous not-change for this period
 
 ASA_min_period_for_agent_torso_is_turning,// in ms, will be used to calculate based on continuous change for this period
 ASA_min_period_for_agent_torso_is_not_turning,// in ms, will be used to calculate based on continuous not-change for this period
 
 ASA_min_period_for_agent_head_is_turning,// in ms, will be used to calculate based on continuous change for this period
 ASA_min_period_for_agent_head_is_not_turning,// in ms, will be used to calculate based on continuous not-change for this period
 
 ASA_min_period_for_agent_hand_is_moving,// in ms, will be used to calculate based on continuous change for this period
 ASA_min_period_for_agent_hand_is_not_moving,// in ms, will be used to calculate based on continuous not-change for this period
 //NOTE : Add any new threshold here, before the last element
 
 
 MAXI_NUM_OF_THRESHOLDS_FOR_ASA
}Agent_State_Analysis_thresholds;

typedef struct agents_state_diff
{
  int agent_has_moved;
  int agent_whole_body_has_turned;
  int agent_torso_has_turned;
  int agent_head_has_turned;
  int agent_R_hand_has_moved;
  int agent_L_hand_has_moved;
  
}agents_state_diff;

typedef struct agents_state_conti_diff_info
{
  double agent_has_moved_conti_for_period;
  double agent_whole_body_has_turned_conti_for_period;
  double agent_torso_has_turned_conti_for_period;
  double agent_head_has_turned_conti_for_period;
  double agent_R_hand_has_moved_conti_for_period;
  double agent_L_hand_has_moved_conti_for_period;
  
  double agent_has_not_moved_conti_for_period;
  double agent_whole_body_has_not_turned_conti_for_period;
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
  AGENT_DID_NOT_MOVE,
  
  AGENT_MOTION_STATUS_UNKNOWN,//NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_MOTION_STATUS
  
}agents_motion_status;

typedef enum agents_head_status
{
  AGENT_HEAD_TURNING=0,
  AGENT_HEAD_STATIC,
  AGENT_HEAD_HAS_TURNED,
  AGENT_HEAD_DID_NOT_TURN,
  
  AGENT_HEAD_STATUS_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_HEAD_STATUS
  
}agents_head_status;


typedef enum agents_hand_status
{
  AGENT_HAND_MOVING=0,
  AGENT_HAND_STATIC,
  AGENT_HAND_HAS_MOVED,
  AGENT_HAND_DID_NOT_MOVE,
  
  AGENT_HAND_STATUS_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_HAND_STATUS
}agents_hand_status;

typedef enum agents_hand_config_mode
{
  AGENT_HAND_AT_REST_MODE=0,
  AGENT_HAND_AT_MANIPULATION_MODE, //due to the limited perception it would be difficult to distinguish between pointing, reaching to grasp, waiting to take etc.It would be better that based on the current context and task such facts would be computed at higher level
  
  AGENT_HAND_CONFIG_MODE_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_HAND_CONFIG_MODE
  
}agents_hand_config_mode;

typedef enum agents_hand_occupancy_mode
{
  AGENT_HAND_HOLDING_OBJECT=0,
  AGENT_HAND_FREE_OF_OBJECT,
  
  AGENT_HAND_OCCUPANCY_MODE_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_HAND_OCCUPANCY_MODE
  
}agents_hand_occupancy_mode;

typedef enum agents_hand_rest_mode_types
{
  AGENT_HAND_REST_BY_POSTURE=0,
  AGENT_HAND_REST_ON_SUPPORT,
  AGENT_HAND_NOT_IN_REST,
  
  AGENT_HAND_REST_MODE_UNKNOWN,//NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_OF_AGENTS_HAND_REST_MODE
}agents_hand_rest_mode_types;

typedef struct agent_hand_occupancy_info
{
  agents_hand_occupancy_mode occupancy_mode;
  char object_in_hand[100];
  int index_obj; //Should be synchronized with the global index of obj in env 
}agent_hand_occupancy_info;

typedef struct agent_hand_rest_info
{
  agents_hand_rest_mode_types rest_type;
  char hand_on_support_obj[100];
  int index_obj; //Should be synchronized with the global index of obj in env 
}agent_hand_rest_info;

typedef enum agents_whole_body_turn_status
{
  AGENT_WHOLE_BODY_TURNING=0,
  AGENT_WHOLE_BODY_NOT_TURNING,
  AGENT_WHOLE_BODY_HAS_TURNED,
  AGENT_WHOLE_BODY_DID_NOT_TURN,
  
  AGENT_WHOLE_BODY_STATUS_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_WHOLE_BODY_STATUS
  
}agents_whole_body_turn_status;


typedef enum agents_torso_status
{
  AGENT_TORSO_TURNING=0,
  AGENT_TORSO_STATIC,
  AGENT_TORSO_HAS_TURNED,
  AGENT_TORSO_DID_NOT_TURN,
  
  AGENT_TORSO_STATUS_UNKNOWN, //NOTE ADD any new state before the last element and Don't forget to add new states in the corresponding maps
  
  MAXI_NUM_AGENTS_TORSO_STATUS
  
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
 int R_shoulder_x_jnt;
 int R_shoulder_y_jnt;
 int R_shoulder_z_jnt;
 int R_elbow_jnt;
 
 int L_hand_jnt;
 int L_shoulder_x_jnt;
 int L_shoulder_y_jnt;
 int L_shoulder_z_jnt;
 int L_elbow_jnt;
  
 int R_hip_x_jnt;
 int R_hip_y_jnt;
 int R_hip_z_jnt;
 
 int L_hip_x_jnt;
 int L_hip_y_jnt;
 int L_hip_z_jnt;

 int R_knee_jnt;
 int L_knee_jnt;
 
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
 
 int R_shoulder_x_Q;
 int R_shoulder_y_Q;
 int R_shoulder_z_Q;
 int R_elbow_Q;
 
 int L_shoulder_x_Q;
 int L_shoulder_y_Q;
 int L_shoulder_z_Q;
 int L_elbow_Q;
 
 int R_hip_x_Q;
 int R_hip_y_Q;
 int R_hip_z_Q;
 
 int L_hip_x_Q;
 int L_hip_y_Q;
 int L_hip_z_Q;
 
 int R_knee_Q;
 int L_knee_Q;
 
}agents_Q_indices_for_ASA;


typedef struct agents_info_for_ASA
{
  int agent_index;
  char agent_name[50]; 
  ////Agent_State_Analysis_thresholds ASA_threshold[MAXI_NUM_OF_THRESHOLDS_FOR_ASA];
  double ASA_threshold[MAXI_NUM_OF_THRESHOLDS_FOR_ASA];
  agents_joint_indices_for_ASA joint_indx;
  agents_Q_indices_for_ASA Q_indx;
  
}agents_info_for_ASA;

typedef struct agents_activity_facts
{
   
   int agent_index; //in env list
   ////char agent_name[50];
  
   agents_motion_status whole_body;
   agents_whole_body_turn_status whole_body_turn;
   agents_torso_status torso;
   agents_head_status head;
   agents_hand_status right_hand;
   agents_hand_status left_hand;
   agents_hand_config_mode right_hand_mode;
   agents_hand_config_mode left_hand_mode;
   agent_hand_occupancy_info right_hand_occup;
   agent_hand_occupancy_info left_hand_occup;
   agent_hand_rest_info right_hand_rest_info;
   agent_hand_rest_info left_hand_rest_info;
   
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
  ////////agents_info_for_ASA *for_agent;
  std::vector <agents_config_info_at_time> agents_config_info;
  
}agents_prev_conf_info;

#endif