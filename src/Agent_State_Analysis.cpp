//
// C++ Implementation: Agent's States Analyses
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@laas.fr>, (C) 2011
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <list>
#include <string>
#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <map>

#include <Util-pkg.h>
#include <P3d-pkg.h>
#include <Collision-pkg.h>

#include "Agent_State_Analysis.h"
#include "Agent_State_Analysis_proto.h"
#include "Mightability_Analysis.h"


extern int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int get_indices_for_MA_agents();
////extern int assign_indices_of_robots();

p3d_env *envPt_ASA;//ASA:Agent's State Analysis

agents_prev_conf_info human1_prev_configs;

agents_info_for_ASA agents_for_ASA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int find_activity_fact_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

agents_activity_facts Ag_Activity_Fact[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int agents_for_ASA_initialized=0;
int index_test_cube_ASA;
configPt agents_tmp_config_ASA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

std::map<int,std::string > agent_motion_status_map;
std::map<int,std::string > agent_head_status_map;
std::map<int,std::string > agent_hand_status_map;
std::map<int,std::string > agent_hand_config_mode_map;
std::map<int,std::string > agent_torso_status_map;

// Internal functions
static int get_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent);
static int init_agent_state_analysis();
static int init_enable_agents_for_facts();
static int get_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK]);
static int init_thresholds_for_ASA();
static int init_all_agents_activity_facts();
static int alloc_agents_tmp_config_for_ASA();
static int create_agents_facts_name_maps();
static int print_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK]);

int hri_execute_Agent_State_Analysis_functions()
{
  printf(" Inside hri_execute_Agent_State_Analysis_functions()\n");
  if(agents_for_ASA_initialized==0)
  {
    get_indices_for_MA_agents();
    init_enable_agents_for_facts();
    init_agent_state_analysis();
    init_thresholds_for_ASA();
    init_all_agents_activity_facts();
    index_test_cube_ASA=get_index_of_robot_by_name("VISBALL_MIGHTABILITY");
     alloc_agents_tmp_config_for_ASA();
     create_agents_facts_name_maps();
    agents_for_ASA_initialized=1;
  }
  ////get_human_activity_facts(human1_fact );
  //////get_human_activity_facts();
 get_agents_activity_facts(find_activity_fact_for);
 print_agents_activity_facts(find_activity_fact_for);
  
}

int alloc_agents_tmp_config_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
   
  agents_tmp_config_ASA[i]= MY_ALLOC(double,envPt_ASA->robot[indices_of_MA_agents[i]]->nb_dof);
  p3d_get_robot_config_into(envPt_ASA->robot[indices_of_MA_agents[i]],&agents_tmp_config_ASA[i]);
  }
}

int destroy_agents_tmp_config_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
   
  MY_FREE(agents_tmp_config_ASA[i],double,envPt_ASA->robot[indices_of_MA_agents[i]]->nb_dof);
  }
}

int create_agents_facts_name_maps()
{

agent_motion_status_map[AGENT_MOVING]="AGENT_MOVING";
agent_motion_status_map[AGENT_STATIC]="AGENT_STATIC";
agent_motion_status_map[AGENT_HAS_MOVED]="AGENT_HAS_MOVED";
agent_motion_status_map[AGENT_DID_NOT_MOVE]="AGENT_DID_NOT_MOVE";
agent_motion_status_map[AGENT_MOTION_STATUS_UNKNOWN]="AGENT_MOTION_STATUS_UNKNOWN";

agent_head_status_map[AGENT_HEAD_TURNING]="AGENT_HEAD_TURNING";
agent_head_status_map[AGENT_HEAD_STATIC]="AGENT_HEAD_STATIC";
agent_head_status_map[AGENT_HEAD_HAS_TURNED]="AGENT_HEAD_HAS_TURNED";
agent_head_status_map[AGENT_HEAD_DID_NOT_TURN]="AGENT_HEAD_DID_NOT_TURN";
agent_head_status_map[AGENT_HEAD_STATUS_UNKNOWN]="AGENT_HEAD_STATUS_UNKNOWN";

agent_hand_status_map[AGENT_HAND_MOVING]="AGENT_HAND_MOVING";
agent_hand_status_map[AGENT_HAND_STATIC]="AGENT_HAND_STATIC";
agent_hand_status_map[AGENT_HAND_HAS_MOVED]="AGENT_HAND_HAS_MOVED";
agent_hand_status_map[AGENT_HAND_DID_NOT_MOVE]="AGENT_HAND_DID_NOT_MOVE";
agent_hand_status_map[AGENT_HAND_STATUS_UNKNOWN]="AGENT_HAND_STATUS_UNKNOWN";

agent_hand_config_mode_map[AGENT_HAND_AT_REST_MODE]="AGENT_HAND_AT_REST_MODE";
agent_hand_config_mode_map[AGENT_HAND_AT_MANIPULATION_MODE]="AGENT_HAND_AT_MANIPULATION_MODE";
agent_hand_config_mode_map[AGENT_HAND_CONFIG_MODE_UNKNOWN]="AGENT_HAND_CONFIG_MODE_UNKNOWN";

agent_torso_status_map[AGENT_TORSO_TURNING]="AGENT_TORSO_TURNING";
agent_torso_status_map[AGENT_TORSO_STATIC]="AGENT_TORSO_STATIC";
agent_torso_status_map[AGENT_TORSO_HAS_TURNED]="AGENT_TORSO_HAS_TURNED";
agent_torso_status_map[AGENT_TORSO_DID_NOT_TURN]="AGENT_TORSO_DID_NOT_TURN";
agent_torso_status_map[AGENT_TORSO_STATUS_UNKNOWN]="AGENT_TORSO_STATUS_UNKNOWN";

}

int init_all_agents_activity_facts()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
	Ag_Activity_Fact[HUMAN1_MA].agent_index=-1;
	Ag_Activity_Fact[HUMAN1_MA].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	Ag_Activity_Fact[HUMAN1_MA].torso=AGENT_TORSO_STATUS_UNKNOWN;
	Ag_Activity_Fact[HUMAN1_MA].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	Ag_Activity_Fact[HUMAN1_MA].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	Ag_Activity_Fact[HUMAN1_MA].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	Ag_Activity_Fact[HUMAN1_MA].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	Ag_Activity_Fact[HUMAN1_MA].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	
	
	break;
      case JIDO_MA:
	Ag_Activity_Fact[JIDO_MA].agent_index=-1;
	Ag_Activity_Fact[JIDO_MA].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	Ag_Activity_Fact[JIDO_MA].torso=AGENT_TORSO_STATUS_UNKNOWN;
	Ag_Activity_Fact[JIDO_MA].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	Ag_Activity_Fact[JIDO_MA].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	Ag_Activity_Fact[JIDO_MA].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	Ag_Activity_Fact[JIDO_MA].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	Ag_Activity_Fact[JIDO_MA].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	break;
	
	
    }
  }
  
}

int init_thresholds_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
	agents_for_ASA[HUMAN1_MA].ASA_threshold.maxi_num_prev_states_to_store=100;
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_whole_body_pos_tolerance=0.01;//in m
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_whole_body_orient_tolerance=DTOR(1);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_torso_orient_tolerance=DTOR(1);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_head_orient_tolerance=DTOR(0.2);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_hand_pos_tolerance=0.005;//in m
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_moving=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_not_moving=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_turning=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_not_turning=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_head_is_turning=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_head_is_not_turning=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_hand_is_moving=1000;//in ms
	agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_hand_is_not_moving=1000;//in ms
	
	
	break;
      case JIDO_MA:
	
	break;
    }
  }
}

int init_indices_of_agent_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case JIDO_MA:
//       agents_for_ASA[i].agent_index=get_index_of_robot_by_name("JIDOKUKA_ROBOT");
//       strcpy(agents_for_ASA[i].agent_name,"JIDOKUKA_ROBOT");
      agents_for_ASA[i].agent_index=indices_of_MA_agents[i];
      strcpy(agents_for_ASA[i].agent_name,envPt_ASA->robot[agents_for_ASA[i].agent_index]->name);
      
      agents_for_ASA[i].joint_indx.body_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "platformJoint");
      
      agents_for_ASA[i].joint_indx.torso_yaw=-1;
      agents_for_ASA[i].joint_indx.torso_pitch=-1;
      agents_for_ASA[i].joint_indx.torso_roll=-1;
      
      agents_for_ASA[i].joint_indx.head_yaw=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "Pan");
      
      agents_for_ASA[i].joint_indx.head_pitch=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "Tilt");
      
      agents_for_ASA[i].joint_indx.head_roll=-1;
      
      agents_for_ASA[i].joint_indx.R_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "RightWrist");
      
      agents_for_ASA[i].joint_indx.L_hand_jnt=-1;
      
      break;
#ifdef HRP2_EXISTS_FOR_MA
      case HRP2_MA:
	 agents_for_ASA[i].agent_index=indices_of_MA_agents[i];
      strcpy(agents_for_ASA[i].agent_name,envPt_ASA->robot[agents_for_ASA[i].agent_index]->name);
      break;
#endif

      case HUMAN1_MA:
// 	agents_for_ASA[i].agent_index=get_index_of_robot_by_name("ACHILE_HUMAN1");
//       strcpy(agents_for_ASA[i].agent_name,"ACHILE_HUMAN1");
      agents_for_ASA[i].agent_index=indices_of_MA_agents[i];
      strcpy(agents_for_ASA[i].agent_name,envPt_ASA->robot[agents_for_ASA[i].agent_index]->name);
      
      agents_for_ASA[i].joint_indx.body_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "Pelvis");
      
      agents_for_ASA[i].Q_indx.body_Q_x=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.body_jnt]->index_dof;
      agents_for_ASA[i].Q_indx.body_Q_y=agents_for_ASA[i].Q_indx.body_Q_x+1;
      agents_for_ASA[i].Q_indx.body_Q_z=agents_for_ASA[i].Q_indx.body_Q_x+2;
      agents_for_ASA[i].Q_indx.body_Q_yaw=agents_for_ASA[i].Q_indx.body_Q_x+3;
      agents_for_ASA[i].Q_indx.body_Q_pitch=agents_for_ASA[i].Q_indx.body_Q_x+4;
      agents_for_ASA[i].Q_indx.body_Q_roll=agents_for_ASA[i].Q_indx.body_Q_x+5;
      
      
      agents_for_ASA[i].joint_indx.torso_yaw=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoX");
      agents_for_ASA[i].joint_indx.torso_pitch=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoY");
      agents_for_ASA[i].joint_indx.torso_roll=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoZ");
      
      agents_for_ASA[i].Q_indx.torso_Q_yaw=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.torso_yaw]->index_dof;
      agents_for_ASA[i].Q_indx.torso_Q_pitch=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.torso_pitch]->index_dof;
      agents_for_ASA[i].Q_indx.torso_Q_roll=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.torso_roll]->index_dof;
      
      agents_for_ASA[i].joint_indx.head_yaw=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadZ");
      agents_for_ASA[i].joint_indx.head_pitch=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadY");
      agents_for_ASA[i].joint_indx.head_roll=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadX");
      
      agents_for_ASA[i].Q_indx.head_Q_yaw=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.head_yaw]->index_dof;
      agents_for_ASA[i].Q_indx.head_Q_pitch=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.head_pitch]->index_dof;
      agents_for_ASA[i].Q_indx.head_Q_roll=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.head_roll]->index_dof;
      
      agents_for_ASA[i].joint_indx.R_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rPalm");
      ////printf(">><< rPalm jnt index=%d\n",agents_for_ASA[i].joint_indx.R_hand_jnt);
      agents_for_ASA[i].joint_indx.R_shoulder_x_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rShoulderX");
      agents_for_ASA[i].joint_indx.R_shoulder_y_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rShoulderY");
      agents_for_ASA[i].joint_indx.R_shoulder_z_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rShoulderZ");
      agents_for_ASA[i].Q_indx.R_shoulder_x_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_shoulder_x_jnt]->index_dof;
      agents_for_ASA[i].Q_indx.R_shoulder_y_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_shoulder_y_jnt]->index_dof;
      agents_for_ASA[i].Q_indx.R_shoulder_z_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_shoulder_z_jnt]->index_dof;
      agents_for_ASA[i].joint_indx.R_elbow_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rElbowZ");
      agents_for_ASA[i].Q_indx.R_elbow_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_elbow_jnt]->index_dof;
      
      agents_for_ASA[i].joint_indx.L_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lPalm");
      agents_for_ASA[i].joint_indx.L_shoulder_x_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lShoulderX");
      agents_for_ASA[i].joint_indx.L_shoulder_y_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lShoulderY");
      agents_for_ASA[i].joint_indx.L_shoulder_z_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lShoulderZ");
      agents_for_ASA[i].Q_indx.L_shoulder_x_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_shoulder_x_jnt]->index_dof;
      agents_for_ASA[i].Q_indx.L_shoulder_y_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_shoulder_y_jnt]->index_dof;
      agents_for_ASA[i].Q_indx.L_shoulder_z_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_shoulder_z_jnt]->index_dof;
      agents_for_ASA[i].joint_indx.L_elbow_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lElbowZ");
      agents_for_ASA[i].Q_indx.L_elbow_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_elbow_jnt]->index_dof;
      
      
      ////agents_for_ASA[i].joint_indx.L_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lPalm");
      
      break;

#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	
      break;
#endif
      
#ifdef PR2_EXISTS_FOR_MA
      case PR2_MA:
	 /*indices_of_MA_agents[i]=get_index_of_robot_by_name("PR2_ROBOT");
	 indices_of_eye_joint_MA_agents[i]=p3d_get_robot_jnt_index_by_name(envPt_MM->robot[indices_of_MA_agents[i]], (char*) "Eyes");
	 */
      break;
#endif
      
    }
    
  }
  
}

int init_enable_agents_for_facts()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
     find_activity_fact_for[i]=0;
  }
 find_activity_fact_for[HUMAN1_MA]=1;
  
}

int init_agent_state_analysis()
{
   envPt_ASA = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
   init_indices_of_agent_for_ASA();
}

int is_hand_on_support(HRI_TASK_AGENT_ENUM for_agent, MA_agent_hand_name for_hand)
{
  int agent_index=indices_of_MA_agents[for_agent];
  int hand_joint;
  char hand_name[50];
  
  if(for_hand==MA_RIGHT_HAND)
  {
   //// printf(" Inside is_hand_on_support for right hand\n");
  hand_joint=agents_for_ASA[for_agent].joint_indx.R_hand_jnt;
  strcpy(hand_name,"rHand");
 
  }
  
  if(for_hand==MA_LEFT_HAND)
  {
    //// printf(" Inside is_hand_on_support for left hand\n");
  hand_joint=agents_for_ASA[for_agent].joint_indx.L_hand_jnt;
  strcpy(hand_name,"lHand");
  }
  
  for(int i=0; i<envPt_ASA->robot[agent_index]->no; i++)
   {
    ////printf(" >>>> envPt_ASA->robot[agent_index]->o[i]->name = %s \n", envPt_ASA->robot[agent_index]->o[i]->name);
    if(strcasestr(envPt_ASA->robot[agent_index]->o[i]->name,hand_name))
    {
       p3d_col_deactivate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->o[i]);
       
    }
   }
  ////p3d_col_deactivate_rob_rob(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]);
  
  ////p3d_col_deactivate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->joints[hand_joint]->o);
  
  
  double x=envPt_ASA->robot[agent_index]->joints[hand_joint]->abs_pos[0][3];
  double y=envPt_ASA->robot[agent_index]->joints[hand_joint]->abs_pos[1][3];
  double z=envPt_ASA->robot[agent_index]->joints[hand_joint]->abs_pos[2][3];
 
  
  double decrement_z_by=0.03;
  
  for(int j=0; j<2;j++)
  {
   p3d_set_freeflyer_pose2(envPt_ASA->robot[index_test_cube_ASA], x,y,z,0,0,0);
  int kcd_with_report=0;
  int res = p3d_col_test_robot(envPt_ASA->robot[index_test_cube_ASA],kcd_with_report);
  if(res>0)
   {
     ////pqp_print_colliding_pair();
     printf(" Human hand is on a support \n");
     /////p3d_col_activate_rob_rob(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]);
     ////p3d_col_activate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->joints[hand_joint]->o);
    for(int i=0; i<envPt_ASA->robot[agent_index]->no; i++)
    {
    ////printf(" >>>> envPt_ASA->robot[agent_index]->o[i]->name = %s \n", envPt_ASA->robot[agent_index]->o[i]->name);
     if(strcasestr(envPt_ASA->robot[agent_index]->o[i]->name,hand_name))
     {
       p3d_col_activate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->o[i]);
       
     }
    }
    p3d_set_freeflyer_pose2(envPt_ASA->robot[index_test_cube_ASA], 0,0,0,0,0,0);
     return 1;
   }
   
   z-=decrement_z_by;
  }
  
  ////p3d_col_activate_rob_rob(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]);
  ////p3d_col_activate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->joints[hand_joint]->o);
  for(int i=0; i<envPt_ASA->robot[agent_index]->no; i++)
    {
    ////printf(" >>>> envPt_ASA->robot[agent_index]->o[i]->name = %s \n", envPt_ASA->robot[agent_index]->o[i]->name);
     if(strcasestr(envPt_ASA->robot[agent_index]->o[i]->name,hand_name))
     {
       p3d_col_activate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->o[i]);
       
     }
    }
     p3d_set_freeflyer_pose2(envPt_ASA->robot[index_test_cube_ASA], 0,0,0,0,0,0);
  return 0;
  
}



int is_hand_in_rest_config(HRI_TASK_AGENT_ENUM for_agent, MA_agent_hand_name for_hand)
{
 //// printf(" Inside is_hand_in_rest_config\n");
   int agent_index=indices_of_MA_agents[for_agent];
  
  p3d_get_robot_config_into(envPt_ASA->robot[agent_index],&agents_tmp_config_ASA[for_agent]);
  
  int shoulder_x_Q;
  int shoulder_z_Q;
  int elbow_Q;
  
 if(for_hand==MA_RIGHT_HAND)
  {
  shoulder_x_Q=agents_for_ASA[for_agent].Q_indx.R_shoulder_x_Q;
  shoulder_z_Q=agents_for_ASA[for_agent].Q_indx.R_shoulder_z_Q;
  elbow_Q=agents_for_ASA[for_agent].Q_indx.R_elbow_Q;
 
  if(agents_tmp_config_ASA[for_agent][shoulder_x_Q]<DTOR(70)||agents_tmp_config_ASA[for_agent][shoulder_z_Q]>DTOR(20)||agents_tmp_config_ASA[for_agent][shoulder_z_Q]<DTOR(-20)||agents_tmp_config_ASA[for_agent][elbow_Q]>DTOR(20)||agents_tmp_config_ASA[for_agent][elbow_Q]<DTOR(-20))
   {
     return 0;
   }
  }
  
  if(for_hand==MA_LEFT_HAND)
  {
  shoulder_x_Q=agents_for_ASA[for_agent].Q_indx.L_shoulder_x_Q;
  shoulder_z_Q=agents_for_ASA[for_agent].Q_indx.L_shoulder_z_Q;
  elbow_Q=agents_for_ASA[for_agent].Q_indx.L_elbow_Q;
  ////printf("agents_tmp_config_ASA[for_agent][shoulder_x_Q] =%lf\n",RTOD(agents_tmp_config_ASA[for_agent][shoulder_x_Q]));
  if(agents_tmp_config_ASA[for_agent][shoulder_x_Q]>DTOR(-70)||agents_tmp_config_ASA[for_agent][shoulder_z_Q]>DTOR(20)||agents_tmp_config_ASA[for_agent][shoulder_z_Q]<DTOR(-20)||agents_tmp_config_ASA[for_agent][elbow_Q]>DTOR(20)||agents_tmp_config_ASA[for_agent][elbow_Q]<DTOR(-20))
   {
     return 0;
   }
  }
  
  return 1;
  
}

int get_agents_static_hand_mode(HRI_TASK_AGENT_ENUM for_agent, MA_agent_hand_name for_hand, agents_hand_config_mode &res_hand_mode)
{
 int hand_on_support_res=is_hand_on_support(for_agent, for_hand);
  ////printf(" after test is_hand_on_support \n");
 if(hand_on_support_res==0)
 {
   ////printf(" Hand is NOT on a support \n");
   int hand_in_rest_config=is_hand_in_rest_config(for_agent, for_hand);
   
   if(hand_in_rest_config==1)
   {
     res_hand_mode=AGENT_HAND_AT_REST_MODE;
     ////printf(" but Hand is in rest config \n");
   }
   else
   {
     res_hand_mode=AGENT_HAND_AT_MANIPULATION_MODE;
     ////printf(" Hand is in manipulation config \n");
   }
 }
 else
 {
   ////printf(" Hand is ON a support \n");
   res_hand_mode=AGENT_HAND_AT_REST_MODE;
   
 }
 
 return 1;
}

int get_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent )
{
  
  double curr_threshold;
  double curr_min_time_period;
  
  int index=indices_of_MA_agents[for_agent];
  Ag_Activity_Fact[for_agent].agent_index=index;
  
  ////envPt_ASA = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  ////////printf("Inside get_human_activity_facts()\n");
  ////int index=agents_fact.agent_index;
  //////int index=get_index_of_robot_by_name("ACHILE_HUMAN1");
  agents_config_info_at_time curr_config;
  ////curr_config.clock=clock();
  ////time(&curr_config.at_time);
  
  gettimeofday(&curr_config.at_time,NULL);
  
  //printf(" After getting clock\n");
  curr_config.config=MY_ALLOC(double,envPt_ASA->robot[index]->nb_dof);
  p3d_get_robot_config_into(envPt_ASA->robot[index],&curr_config.config);
  
  int R_H_jnt_index=agents_for_ASA[HUMAN1_MA].joint_indx.R_hand_jnt;
  int L_H_jnt_index=agents_for_ASA[HUMAN1_MA].joint_indx.L_hand_jnt;
  
   for(int i=0 ; i<=3 ; i++)
    {
    for(int j=0 ; j<=3 ; j++)
      {
      curr_config.R_hand_pos[i][j]=envPt_ASA->robot[index]->joints[R_H_jnt_index]->abs_pos[i][j];
      ////printf(" storing for agent=%s, joint indx=%d, i=%d, j=%d, val = %lf\n",envPt_ASA->robot[index]->name, R_H_jnt_index,i,j,envPt_ASA->robot[index]->joints[R_H_jnt_index]->abs_pos[i][j]);
      curr_config.L_hand_pos[i][j]=envPt_ASA->robot[index]->joints[L_H_jnt_index]->abs_pos[i][j];
      }
    }
    //printf(" 
  //printf("got human config \n");
  ////return 1;
  if(human1_prev_configs.agents_config_info.size()<=1)
  {
    printf(" Insufficient data \n");
    
      
    curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
    curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
    curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
    curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
    curr_config.diff_prev_config.agent_has_moved=-1;
    curr_config.diff_prev_config.agent_head_has_turned=-1;
    curr_config.diff_prev_config.agent_torso_has_turned=-1;
    curr_config.diff_prev_config.agent_L_hand_has_moved=-1;
    curr_config.diff_prev_config.agent_R_hand_has_moved=-1;
    
    human1_prev_configs.agents_config_info.push_back(curr_config);
    return 0;
  }
  
  if(human1_prev_configs.agents_config_info.size()>=agents_for_ASA[for_agent].ASA_threshold.maxi_num_prev_states_to_store)
  {
    ////////printf(" Forgetting the oldest configuration of human \n");  
    agents_config_info_at_time prev_config=human1_prev_configs.agents_config_info.front();
    MY_FREE(prev_config.config,double,envPt_ASA->robot[index]->nb_dof);
    human1_prev_configs.agents_config_info.erase(human1_prev_configs.agents_config_info.begin());
    
  }
    
  int has_body_moved=0;
  
  //Comparing whole base position
  agents_config_info_at_time prev_config=human1_prev_configs.agents_config_info.back();
  ////double elapsedTime= ( curr_config.clock-prev_config.clock ) /CLOCKS_PER_SEC;
  ////double elapsedTime=difftime(curr_config.at_time,prev_config.at_time);
  long int mtime, seconds, useconds;

    seconds  = curr_config.at_time.tv_sec  - prev_config.at_time.tv_sec;
    useconds = curr_config.at_time.tv_usec - prev_config.at_time.tv_usec;
     ////////printf(" curr_config.at_time.tv_sec = %ld, prev_config.at_time.tv_sec =%ld\n",curr_config.at_time.tv_sec,prev_config.at_time.tv_sec);
     
      ////////printf(" curr_config.at_time.tv_usec = %ld, prev_config.at_time.tv_usec =%ld\n",curr_config.at_time.tv_usec,prev_config.at_time.tv_usec);
     ////////printf(" seconds = %ld, useconds =%ld\n",seconds,useconds);
     
     
      double elapsedTime = ((seconds) * 1000 + useconds/1000.0) ;
    
    ////////printf("Elapsed time: %lf milliseconds\n", elapsedTime);

  curr_config.time_diff_from_prev_config=elapsedTime;
  printf("Time diff from prev config= %lf milliseconds\n", curr_config.time_diff_from_prev_config);
  
  ////printf(" prev conf time=%lf, curr conf time= %lf, time period : %lf s\n",prev_config.clock, curr_config.clock, elapsedTime);
               
  ////printf(" prev conf time=%ld, curr conf time= %ld, time period : %lf s\n",prev_config.at_time/3600, curr_config.at_time/3600, elapsedTime);
    int Q_index_x=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_x;
    int Q_index_y=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_y;
    int Q_index_z=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_z;
    
    curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_whole_body_pos_tolerance;
    ////////printf(" curr_threshold for whole body moved = %lf\n",curr_threshold);
    
    
  if(fabs(curr_config.config[Q_index_x]-prev_config.config[Q_index_x])>=curr_threshold||fabs(curr_config.config[Q_index_y]-prev_config.config[Q_index_y])>=curr_threshold||fabs(curr_config.config[Q_index_z]-prev_config.config[Q_index_z])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_has_moved=1;
     ///////printf(">>>> Human whole body has moved \n");
     Ag_Activity_Fact[for_agent].whole_body=AGENT_HAS_MOVED;
   
  }
  else
  {
    curr_config.diff_prev_config.agent_has_moved=0;
    Ag_Activity_Fact[for_agent].whole_body=AGENT_DID_NOT_MOVE;
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_moving;
  
  if(curr_config.diff_prev_config.agent_has_moved==1)
  {
    curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
   if(prev_config.diff_prev_config.agent_has_moved==1)
    {
      curr_config.conti_diff_info.agent_has_moved_conti_for_period=prev_config.conti_diff_info.agent_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_has_moved_conti_for_period>=curr_min_time_period)
    {
      ////////printf(" **** Human is moving \n");
      Ag_Activity_Fact[for_agent].whole_body=AGENT_MOVING;
    }
  }
  else
  {
    curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_not_moving;
  
    if(prev_config.diff_prev_config.agent_has_moved==0)
    {
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=prev_config.conti_diff_info.agent_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human is NOT moving \n");
      Ag_Activity_Fact[for_agent].whole_body=AGENT_STATIC;
    }
  }
  
   int Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_yaw;
   int Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_pitch;
   int Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_roll;
   
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_whole_body_orient_tolerance;
   
  curr_config.diff_prev_config.agent_torso_has_turned=0;
  Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_DID_NOT_TURN;
  
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_torso_has_turned=1;
    //////printf(">>>>>> Human whole body has turned \n");
    Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_HAS_TURNED;
  }
  
   Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_yaw;
   Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_pitch;
    Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_roll;
    
    curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_torso_orient_tolerance;
   
    
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_torso_has_turned=1;
    ///////printf(">>>>>>>> Human torso has turned \n");
    Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_HAS_TURNED;
  }
  
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_turning;
  
  if(curr_config.diff_prev_config.agent_torso_has_turned==1)
  {
    curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
  if(prev_config.diff_prev_config.agent_torso_has_turned==1)
    {
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=prev_config.conti_diff_info.agent_torso_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human is turning \n");
      Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_TURNING;
      
    }
  }
  else
  {
    curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_not_turning;
    if(prev_config.diff_prev_config.agent_torso_has_turned==0)
    {
      curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=prev_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
       curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period>=curr_min_time_period)
    {
      /////printf(" **** Human is NOT turning \n");
      Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_STATIC;
      
    }
  }
  
  
   Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_yaw;
   Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_pitch;
   Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_roll;
    
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_head_orient_tolerance;
   
  
   ////printf(" Prev val of Head not turned continuously for period = %lf \n",prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period);
   
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_head_has_turned=1;
    ////printf(">>>>>>>> Human head has turned \n");
    Ag_Activity_Fact[for_agent].head=AGENT_HEAD_HAS_TURNED;
  }
  else
  {
    curr_config.diff_prev_config.agent_head_has_turned=0;
    Ag_Activity_Fact[for_agent].head=AGENT_HEAD_DID_NOT_TURN;
    ////printf(">>>>>>>> Human head has not turned \n");
  }
  
   curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_head_is_turning;
  if(curr_config.diff_prev_config.agent_head_has_turned==1)
  {
    curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
    
  if(prev_config.diff_prev_config.agent_head_has_turned==1)
    {
      curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=prev_config.conti_diff_info.agent_head_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
      
    }
    else
    {
      curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
      
    }
    
    if(curr_config.conti_diff_info.agent_head_has_turned_conti_for_period>=curr_min_time_period)
    {
      ////printf(" **** Human head is turning \n");
      Ag_Activity_Fact[for_agent].head=AGENT_HEAD_TURNING;
    }
  }
  else
  {
    curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
    
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_head_is_not_turning;
    
    if(prev_config.diff_prev_config.agent_head_has_turned==0)
    {
      ////printf(" Adding head not turned time, curr_not_turn_time=+=prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config 
      curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
    }
    ////printf(" Head not turned continuously for period = %lf \n",curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period);
    if(curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human head is NOT turning \n");
      Ag_Activity_Fact[for_agent].head=AGENT_HEAD_STATIC;
      
    }
  }
   
  /////Now for the hand movement
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_hand_pos_tolerance;
   
  
  ////printf(" R_hand_pos= (%lf, %lf, %lf) \n", curr_config.R_hand_pos[0][3], curr_config.R_hand_pos[1][3], curr_config.R_hand_pos[2][3]);
  
  if(fabs(curr_config.R_hand_pos[0][3]-prev_config.R_hand_pos[0][3])>=curr_threshold||fabs(curr_config.R_hand_pos[1][3]-prev_config.R_hand_pos[1][3])>=curr_threshold||fabs(curr_config.R_hand_pos[2][3]-prev_config.R_hand_pos[2][3])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_R_hand_has_moved=1;
    //////printf(" >>>>>>>>>> Human's Right Hand has moved\n");
    Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_HAS_MOVED;
  }
  else
  {
    curr_config.diff_prev_config.agent_R_hand_has_moved=0;
    Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_DID_NOT_MOVE;
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_moving;
  
  if(curr_config.diff_prev_config.agent_R_hand_has_moved==1)
  {
    curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
  if(prev_config.diff_prev_config.agent_R_hand_has_moved==1)
    {
      curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=prev_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human Right Hand is Moving \n");
      Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_MOVING;
    }
  }
  else
  {
    curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=0;
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_not_moving;
    if(prev_config.diff_prev_config.agent_R_hand_has_moved==0)
    {
      curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=prev_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      ///////printf(" **** Human Right Hand is NOT Moving \n");
      Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_STATIC;
      
    }
  }
  
  get_agents_static_hand_mode(for_agent,MA_RIGHT_HAND, Ag_Activity_Fact[for_agent].right_hand_mode);
  
  
  curr_config.diff_prev_config.agent_L_hand_has_moved=0;
  if(fabs(curr_config.L_hand_pos[0][3]-prev_config.L_hand_pos[0][3])>=curr_threshold||fabs(curr_config.L_hand_pos[1][3]-prev_config.L_hand_pos[1][3])>=curr_threshold||fabs(curr_config.L_hand_pos[2][3]-prev_config.L_hand_pos[2][3])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_L_hand_has_moved=1;
     ///////printf(" >>>>>>>>>> Human's Left Hand has moved\n");
     Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_HAS_MOVED;
  }
  else
  {
    curr_config.diff_prev_config.agent_L_hand_has_moved=0;
    Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_DID_NOT_MOVE;
  
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_moving;
  
  if(curr_config.diff_prev_config.agent_L_hand_has_moved==1)
  {
    curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
  if(prev_config.diff_prev_config.agent_L_hand_has_moved==1)
    {
      curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=prev_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human Left Hand is Moving \n");
      Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_MOVING;
    }
  }
  else
  {
    curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=0;
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_not_moving;
    if(prev_config.diff_prev_config.agent_L_hand_has_moved==0)
    {
      curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=prev_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      //////printf(" **** Human Left Hand is NOT Moving \n");
      Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_STATIC;
      
    }
  }
  
  get_agents_static_hand_mode(for_agent,MA_LEFT_HAND,Ag_Activity_Fact[for_agent].left_hand_mode);
  
  human1_prev_configs.agents_config_info.push_back(curr_config);
  ////printf(
  ////
  
  ////if(
}

int get_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK])
{
 for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
 {
   switch(i)
   {
     case HUMAN1_MA:
       if(find_facts_for[i]==1)
       {
	 get_human_activity_facts(HUMAN1_MA);
       }
       
       break;
   }
 }
  
}

int print_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent)
{
  printf(" ========= Status for agent %s =======\n",envPt_ASA->robot[Ag_Activity_Fact[for_agent].agent_index]->name);

  printf(" %s \n",agent_motion_status_map[Ag_Activity_Fact[for_agent].whole_body].c_str());
  printf(" %s \n",agent_torso_status_map[Ag_Activity_Fact[for_agent].torso].c_str());
  printf(" %s \n",agent_head_status_map[Ag_Activity_Fact[for_agent].head].c_str());
    
     printf(" Right Hand: %s \n",agent_hand_status_map[Ag_Activity_Fact[for_agent].right_hand].c_str());
     printf(" Right Hand: %s \n",agent_hand_config_mode_map[Ag_Activity_Fact[for_agent].right_hand_mode].c_str());
     printf(" Left Hnad: %s \n",agent_hand_status_map[Ag_Activity_Fact[for_agent].left_hand].c_str());
     printf(" Left Hnad: %s \n",agent_hand_config_mode_map[Ag_Activity_Fact[for_agent].left_hand_mode].c_str());
     
      
  ////Ag_Activity_Fact[for_agent].
}

int print_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK])
{
 for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
 {
   switch(i)
   {
     case HUMAN1_MA:
       if(find_facts_for[i]==1)
       {
	 print_human_activity_facts(HUMAN1_MA);
       }
       
       break;
   }
 }
  
}