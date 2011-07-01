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
agents_activity_facts Ag_Activity_Prev_Fact_for_print[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int agents_for_ASA_initialized=0;
int agents_for_ASA_prepared=0;
extern char CURRENT_OBJECT_TO_MANIPULATE[50];

int PRINT_AGENT_STATE_FACTS=1;

int index_test_cube_ASA;
configPt test_cube_tmp_config;

configPt agents_tmp_config_ASA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
Agent_Activity_fact_by_names Hum_facts_by_name;

std::map<int,std::string > agent_motion_status_map;
std::map<int,std::string > agent_head_status_map;
std::map<int,std::string > agent_hand_status_map;
std::map<int,std::string > agent_hand_config_mode_map;
std::map<int,std::string > agent_hand_occupancy_info_map;
std::map<int,std::string > agent_hand_rest_info_map;
std::map<int,std::string > agent_torso_status_map;
std::map<int,std::string > agent_whole_body_turn_status_map;

// Internal functions
static int get_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent);
static int init_agent_state_analysis();
static int init_enable_agents_for_facts();
static int get_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK]);
//static int init_thresholds_for_ASA(char *file_name_with_path);
static int init_all_agents_activity_facts();
static int alloc_agents_tmp_config_for_ASA();
static int create_agents_facts_name_maps();
static int print_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK]);
static int prepare_for_Agent_State_Analysis();


int hri_execute_Agent_State_Analysis_functions()
{
  
  ////printf(" Inside hri_execute_Agent_State_Analysis_functions()\n");
  /*
    if(agents_for_ASA_initialized==0)
    {
    char *threshold_file_path="/home/akpandey/Human_State_Analysis_Thresholds.txt";
    get_indices_for_MA_agents();
    init_enable_agents_for_facts();
    init_agent_state_analysis();
    init_thresholds_for_ASA(threshold_file_path);
    init_all_agents_activity_facts();
    index_test_cube_ASA=get_index_of_robot_by_name("VISBALL_MIGHTABILITY");
    alloc_agents_tmp_config_for_ASA();
    create_agents_facts_name_maps();
    agents_for_ASA_initialized=1;
    }
  */
  ////get_human_activity_facts(human1_fact );
  //////get_human_activity_facts();
  if(agents_for_ASA_prepared==1)
    {
      int support_index;
      ////is_object_laying_on_a_support(get_index_of_robot_by_name(CURRENT_OBJECT_TO_MANIPULATE), support_index);
      get_agents_activity_facts(find_activity_fact_for);

 
      if(PRINT_AGENT_STATE_FACTS==1)
	{
	  print_agents_activity_facts(find_activity_fact_for);
	}
    }
}

int prepare_for_Agent_State_Analysis(char *threshold_file_path)
{
  printf("agents_for_ASA_initialized=%d\n",agents_for_ASA_initialized);
  ////char *threshold_file_path="/home/akpandey/Human_State_Analysis_Thresholds.txt";
  if(agents_for_ASA_initialized==0)
    {
      printf(">>>> ASA_ERROR: First Initialize Agents for ASA, most probably the function to init is init_agents_for_MA_and_ASA()\n");
      agents_for_ASA_prepared=0;
      return 0;
    }
    
  ////get_indices_for_MA_agents();
  init_enable_agents_for_facts();
  init_agent_state_analysis();
    
  int init_thr_res=init_thresholds_for_ASA(threshold_file_path);
  if(init_thr_res==0)
    return 0;
    
  init_all_agents_activity_facts();
  index_test_cube_ASA=get_index_of_robot_by_name("VISBALL_MIGHTABILITY");
  alloc_agents_tmp_config_for_ASA();
  create_agents_facts_name_maps();
  agents_for_ASA_prepared=1;
  return 1;
}

int alloc_agents_tmp_config_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
    {
   
      agents_tmp_config_ASA[i]= MY_ALLOC(double,envPt_ASA->robot[indices_of_MA_agents[i]]->nb_dof);
      p3d_get_robot_config_into(envPt_ASA->robot[indices_of_MA_agents[i]],&agents_tmp_config_ASA[i]);
    }
  
  test_cube_tmp_config=MY_ALLOC(double,envPt_ASA->robot[index_test_cube_ASA]->nb_dof);
  p3d_get_robot_config_into(envPt_ASA->robot[index_test_cube_ASA],&test_cube_tmp_config);
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

  agent_hand_occupancy_info_map[AGENT_HAND_HOLDING_OBJECT]="AGENT_HAND_HOLDING_OBJECT";
  agent_hand_occupancy_info_map[AGENT_HAND_FREE_OF_OBJECT]="AGENT_HAND_FREE_OF_OBJECT";
  agent_hand_occupancy_info_map[AGENT_HAND_OCCUPANCY_MODE_UNKNOWN]="AGENT_HAND_OCCUPANCY_MODE_UNKNOWN";


  agent_hand_rest_info_map[AGENT_HAND_REST_BY_POSTURE]="AGENT_HAND_REST_BY_POSTURE";
  agent_hand_rest_info_map[AGENT_HAND_REST_ON_SUPPORT]="AGENT_HAND_REST_ON_SUPPORT";
  agent_hand_rest_info_map[AGENT_HAND_NOT_IN_REST]="AGENT_HAND_NOT_IN_REST";
  agent_hand_rest_info_map[AGENT_HAND_REST_MODE_UNKNOWN]="AGENT_HAND_REST_MODE_UNKNOWN";

  agent_whole_body_turn_status_map[AGENT_WHOLE_BODY_TURNING]="AGENT_WHOLE_BODY_TURNING";
  agent_whole_body_turn_status_map[AGENT_WHOLE_BODY_NOT_TURNING]="AGENT_WHOLE_BODY_NOT_TURNING";
  agent_whole_body_turn_status_map[AGENT_WHOLE_BODY_HAS_TURNED]="AGENT_WHOLE_BODY_HAS_TURNED";
  agent_whole_body_turn_status_map[AGENT_WHOLE_BODY_DID_NOT_TURN]="AGENT_WHOLE_BODY_DID_NOT_TURN";
  agent_whole_body_turn_status_map[AGENT_WHOLE_BODY_STATUS_UNKNOWN]="AGENT_WHOLE_BODY_STATUS_UNKNOWN";

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
	  Ag_Activity_Fact[i].agent_index=-1;
	  Ag_Activity_Fact[i].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].torso=AGENT_TORSO_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	  Ag_Activity_Fact[i].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	
	  Ag_Activity_Fact[i].right_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].right_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	
	
	  Ag_Activity_Prev_Fact_for_print[i].agent_index=-1;
	  Ag_Activity_Prev_Fact_for_print[i].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].torso=AGENT_TORSO_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	  Ag_Activity_Prev_Fact_for_print[i].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	
	  break;

#ifdef JIDO_EXISTS_FOR_MA
	case JIDO_MA:
	  Ag_Activity_Fact[i].agent_index=-1;
	  Ag_Activity_Fact[i].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].torso=AGENT_TORSO_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	  Ag_Activity_Fact[i].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	
	  Ag_Activity_Fact[i].right_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].right_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Fact[i].left_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	
	
	  Ag_Activity_Prev_Fact_for_print[i].agent_index=-1;
	  Ag_Activity_Prev_Fact_for_print[i].whole_body=AGENT_MOTION_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].torso=AGENT_TORSO_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].head=AGENT_HEAD_STATUS_UNKNOWN;
	
	  Ag_Activity_Prev_Fact_for_print[i].right_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand=AGENT_HAND_STATUS_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_mode=AGENT_HAND_CONFIG_MODE_UNKNOWN;
	
	
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].right_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_occup.occupancy_mode=AGENT_HAND_OCCUPANCY_MODE_UNKNOWN;
	  Ag_Activity_Prev_Fact_for_print[i].left_hand_rest_info.rest_type=AGENT_HAND_REST_MODE_UNKNOWN;
	
	
	  break;
#endif
	
	}
    }
  
}

FILE *Thr_FP; 
int init_thresholds_for_ASA(char *file_name_with_path)
{
  char line[200];
  char th_name[100];
  double th_val;
  char req_conversion_type[50];
  int j=0;
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
    {
      switch(i)
	{
	case HUMAN1_MA:
	  Thr_FP = fopen (file_name_with_path, "r"); 
	  if(Thr_FP==NULL)
	    {
	      printf(" >>>> ASA ERROR: Can not open the threshold file %s for reading the values for human\n", file_name_with_path);
	      return 0;
	  
	    }
	
	  printf(" Opening the threshold file %s for reading the threshold values for human\n", file_name_with_path);
	 
	  for( j=0; j<MAXI_NUM_OF_THRESHOLDS_FOR_ASA; j++)
	    {
	      if(fgets(line, 150, Thr_FP) != NULL)
		{
		  sscanf (line, "%s %lf %s", &th_name, &th_val, &req_conversion_type);
		  if(strcmp("DTOR", req_conversion_type)==0)
		    {
		      printf(" Got %s, conv type= %s \n",line, req_conversion_type);
		      th_val=DTOR(th_val);
		    }
		  agents_for_ASA[HUMAN1_MA].ASA_threshold[j]=th_val;
		  printf(" Setting %s = % lf\n", th_name, agents_for_ASA[HUMAN1_MA].ASA_threshold[j]);
		}
	    }
	  /*
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_maxi_num_prev_states_to_store]=100;
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_agents_whole_body_pos_tolerance]=0.01;//in m
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_agents_whole_body_orient_tolerance]=DTOR(1);//rad
	
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_agents_torso_orient_tolerance]=DTOR(1);//rad
	
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_agents_head_orient_tolerance]=DTOR(0.2);//rad
	
	    agents_for_ASA[HUMAN1_MA].ASA_threshold[ASA_agents_hand_pos_tolerance]=0.005;//in m
	
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_moving=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_not_moving=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_turning=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_is_not_turning=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_head_is_turning=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_head_is_not_turning=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_hand_is_moving=1000;//in ms
	    agents_for_ASA[HUMAN1_MA].ASA_threshold.min_period_for_agent_hand_is_not_moving=1000;//in ms
	
	  */
	  fclose(Thr_FP);
	  break;

#ifdef JIDO_EXISTS_FOR_MA
	case JIDO_MA:
	
	  break;
#endif
	}
    }
  return 1;
}

int init_indices_of_agent_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
    {
      switch(i)
	{
#ifdef JIDO_EXISTS_FOR_MA
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
#endif
      
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
      
	  agents_for_ASA[i].joint_indx.R_hip_x_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rHipX");
	  agents_for_ASA[i].joint_indx.R_hip_y_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rHipY");
	  agents_for_ASA[i].joint_indx.R_hip_z_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rHipZ");
	  agents_for_ASA[i].Q_indx.R_hip_x_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_hip_x_jnt]->index_dof;
	  agents_for_ASA[i].Q_indx.R_hip_y_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_hip_y_jnt]->index_dof;
	  agents_for_ASA[i].Q_indx.R_hip_z_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_hip_z_jnt]->index_dof;
      
	  agents_for_ASA[i].joint_indx.R_knee_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rKnee");
	  agents_for_ASA[i].Q_indx.R_knee_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.R_knee_jnt]->index_dof;
      
	  agents_for_ASA[i].joint_indx.L_hip_x_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lHipX");
	  agents_for_ASA[i].joint_indx.L_hip_y_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lHipY");
	  agents_for_ASA[i].joint_indx.L_hip_z_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lHipZ");
	  agents_for_ASA[i].Q_indx.L_hip_x_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_hip_x_jnt]->index_dof;
	  agents_for_ASA[i].Q_indx.L_hip_y_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_hip_y_jnt]->index_dof;
	  agents_for_ASA[i].Q_indx.L_hip_z_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_hip_z_jnt]->index_dof;
      
	  agents_for_ASA[i].joint_indx.L_knee_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lKnee");
	  agents_for_ASA[i].Q_indx.L_knee_Q=envPt_ASA->robot[agents_for_ASA[i].agent_index]->joints[agents_for_ASA[i].joint_indx.L_knee_jnt]->index_dof;
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


int is_hand_on_support(HRI_TASK_AGENT_ENUM for_agent, MA_agent_hand_name for_hand, int &supp_index)
{
  int on_support=0;
  int agent_index=indices_of_MA_agents[for_agent];
  int hand_joint;
  char hand_name[50];
  
  if(for_hand==MA_RIGHT_HAND)
    {
      ////printf(" Inside is_hand_on_support for right hand\n");
      hand_joint=agents_for_ASA[for_agent].joint_indx.R_hand_jnt;
      strcpy(hand_name,"rHand");
 
    }
  
  if(for_hand==MA_LEFT_HAND)
    {
      ////printf(" Inside is_hand_on_support for left hand\n");
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
 
  test_cube_tmp_config[6]=x;
  test_cube_tmp_config[7]=y;
  ////envPt_ASA->robot[index_test_cube_ASA]->joints[1]->abs_pos[0][3]=x;
  ////envPt_ASA->robot[index_test_cube_ASA]->joints[1]->abs_pos[1][3]=y;
  
  double decrement_z_by=0.03;
  
  for(int j=0; j<2;j++)
    {
      test_cube_tmp_config[8]=z;
      p3d_set_and_update_this_robot_conf(envPt_ASA->robot[index_test_cube_ASA], test_cube_tmp_config);
   
      ////envPt_ASA->robot[index_test_cube_ASA]->joints[1]->abs_pos[2][3]=z;
    
      ////p3d_set_freeflyer_pose2(envPt_ASA->robot[index_test_cube_ASA], x,y,z,0,0,0);
      int kcd_with_report=0;
      int res = p3d_col_test_robot(envPt_ASA->robot[index_test_cube_ASA],kcd_with_report);
      if(res>0)
	{
	  ////pqp_print_colliding_pair();
	  //////// printf(" Human hand is on a support \n");
     
	  /*p3d_obj *o1;
	    p3d_obj *o2;
	    p3d_obj *sup_obj;
		
	    printf(" From current pos \n");
	    pqp_colliding_pair(&o1, &o2);
	    printf(" obj1= %s, obj2 =%s\n",o1->name,o2->name);
	  */
		
	  char obj_1[200], obj_2[200];
	  char sup_obj_name[200];
	  pqp_colliding_obj_name_pair(obj_1, obj_2);
	  //////printf("obj1= %s, obj2 =%s\n",obj_1,obj_2);
		
	  if(strcasestr(obj_1,envPt_ASA->robot[index_test_cube_ASA]->name))
	    {
	      //sup_obj=o2;
	      strcpy(sup_obj_name,obj_2);
                  
	      ////int coll_obj_index=get_index_of_robot_by_name(o2->name);
	      ////is_object_laying_on_a_support()
	    }
	  else
	    {
	      //sup_obj=o1;
	      strcpy(sup_obj_name,obj_1);
	    }
	  //////printf(" sup_obj_name=%s\n",sup_obj_name);
	  int pt_ctr=0;
	  char obj_name[100];
	  while(sup_obj_name[pt_ctr]!='.'&&sup_obj_name[pt_ctr]!='\0')
	    {
	      obj_name[pt_ctr]=sup_obj_name[pt_ctr];
	      pt_ctr++;
	    }
	  obj_name[pt_ctr]='\0';
	  /*
	    int obj_name_end_indx;
	    obj_name_end_indx = strcspn (sup_obj_name,".");
	    printf(" obj_name_end_indx=%d\n",obj_name_end_indx);
	    char * obj_name;
	    strncpy(obj_name,sup_obj_name,obj_name_end_indx);
	    obj_name[obj_name_end_indx]='\0';
	  */
	  supp_index=get_index_of_robot_by_name(obj_name);
		   
	  //////printf(" support is %s, supp_index=%d\n",obj_name, supp_index);
		  
	  /////p3d_col_activate_rob_rob(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]);
	    ////p3d_col_activate_rob_obj(envPt_ASA->robot[index_test_cube_ASA],envPt_ASA->robot[agent_index]->joints[hand_joint]->o);
	    on_support=1;
	    break;
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
  ////     p3d_set_freeflyer_pose2(envPt_ASA->robot[index_test_cube_ASA], 0,0,0,0,0,0);
  test_cube_tmp_config[6]=0;
  test_cube_tmp_config[7]=0;
  test_cube_tmp_config[8]=0;
  p3d_set_and_update_this_robot_conf(envPt_ASA->robot[index_test_cube_ASA], test_cube_tmp_config);
    
  if(on_support==1)
    return 1;
     
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
 
      if(agents_tmp_config_ASA[for_agent][shoulder_x_Q]<DTOR(60)||
	 agents_tmp_config_ASA[for_agent][shoulder_z_Q]>DTOR(30)||
	 agents_tmp_config_ASA[for_agent][shoulder_z_Q]<DTOR(-30)||
	 agents_tmp_config_ASA[for_agent][elbow_Q]>DTOR(40)||
	 agents_tmp_config_ASA[for_agent][elbow_Q]<DTOR(-40))
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
      if(agents_tmp_config_ASA[for_agent][shoulder_x_Q]>DTOR(-60)||
	 agents_tmp_config_ASA[for_agent][shoulder_z_Q]>DTOR(30)||
	 agents_tmp_config_ASA[for_agent][shoulder_z_Q]<DTOR(-30)||
	 agents_tmp_config_ASA[for_agent][elbow_Q]>DTOR(40)||
	 agents_tmp_config_ASA[for_agent][elbow_Q]<DTOR(-40))
	{
	  return 0;
	}
    }
  
  return 1;
  
}

int get_agents_hand_info(HRI_TASK_AGENT_ENUM for_agent, MA_agent_hand_name for_hand,
			 agents_hand_config_mode &res_hand_mode, 
			 agent_hand_occupancy_info &res_hand_occup, 
			 agent_hand_rest_info &res_hand_rest_info)
{
  int hand_supp_index;
  int hand_on_support_res=is_hand_on_support(for_agent, for_hand,hand_supp_index);
  ////printf(" hand_on_support_res = %d \n",hand_on_support_res);
 
  if(hand_on_support_res==1 && hand_supp_index >=0)
    {
      ////printf(" hand support obj name = %s\n",envPt_ASA->robot[hand_supp_index]->name);
   
      if(strcasestr(envPt_ASA->robot[hand_supp_index]->name,"SHELF")||
	 strcasestr(envPt_ASA->robot[hand_supp_index]->name,"TABLE")||
	 strcasestr(envPt_ASA->robot[hand_supp_index]->name,"CHAIR"))
	{
	  res_hand_mode=AGENT_HAND_AT_REST_MODE;
	  strcpy(res_hand_rest_info.hand_on_support_obj,envPt_ASA->robot[hand_supp_index]->name);
	  res_hand_rest_info.index_obj=hand_supp_index;
	  res_hand_rest_info.rest_type=AGENT_HAND_REST_ON_SUPPORT;
	  res_hand_occup.occupancy_mode=AGENT_HAND_FREE_OF_OBJECT;
	  return 1;
	  ////printf(" Support is heavy \n");
	}
      else
	{
	  if(hand_supp_index==indices_of_MA_agents[for_agent])
	    {
	      res_hand_mode=AGENT_HAND_AT_REST_MODE;
	      strcpy(res_hand_rest_info.hand_on_support_obj,envPt_ASA->robot[hand_supp_index]->name);
	      res_hand_rest_info.index_obj=hand_supp_index;
	      res_hand_rest_info.rest_type=AGENT_HAND_REST_ON_SUPPORT;
	      res_hand_occup.occupancy_mode=AGENT_HAND_FREE_OF_OBJECT;
	      ////printf("Support is human itself \n");
	      return 1;
	    }
	  else
	    {
	      p3d_col_deactivate_rob_rob(envPt_ASA->robot[indices_of_MA_agents[for_agent]], envPt_ASA->robot[hand_supp_index]);
	      int obj_supp_indx;
	      int is_obj_on_support_res=is_object_laying_on_a_support(hand_supp_index,obj_supp_indx);
	      if(is_obj_on_support_res==1)
		{
		  res_hand_mode=AGENT_HAND_AT_REST_MODE;
		  strcpy(res_hand_rest_info.hand_on_support_obj,envPt_ASA->robot[hand_supp_index]->name);
		  res_hand_rest_info.index_obj=hand_supp_index;
		  res_hand_rest_info.rest_type=AGENT_HAND_REST_ON_SUPPORT;
     
		  res_hand_occup.occupancy_mode=AGENT_HAND_FREE_OF_OBJECT;
		}
	      else
		{
		  res_hand_mode=AGENT_HAND_AT_MANIPULATION_MODE;
     
		  strcpy(res_hand_occup.object_in_hand,envPt_ASA->robot[hand_supp_index]->name);
		  res_hand_occup.index_obj=hand_supp_index;
		  res_hand_occup.occupancy_mode=AGENT_HAND_HOLDING_OBJECT;
     
		  res_hand_rest_info.rest_type=AGENT_HAND_NOT_IN_REST;
     
     
		}
   
	      p3d_col_activate_rob_rob(envPt_ASA->robot[indices_of_MA_agents[for_agent]], envPt_ASA->robot[hand_supp_index]);
 
	      return 1;
	    }
	}
    }
  ////printf(" after test is_hand_on_support \n");
  if(hand_on_support_res==0)
    {
      ////printf(" Hand is NOT on a support \n");
      int hand_in_rest_config=is_hand_in_rest_config(for_agent, for_hand);
   
      if(hand_in_rest_config==1)
	{
	  res_hand_mode=AGENT_HAND_AT_REST_MODE;
	  res_hand_occup.occupancy_mode=AGENT_HAND_FREE_OF_OBJECT;
	  res_hand_rest_info.rest_type=AGENT_HAND_REST_BY_POSTURE;
	  ////printf(" but Hand is in rest config \n");
	}
      else
	{
	  res_hand_mode=AGENT_HAND_AT_MANIPULATION_MODE;
      
	  res_hand_occup.occupancy_mode=AGENT_HAND_FREE_OF_OBJECT;
	  res_hand_rest_info.rest_type=AGENT_HAND_NOT_IN_REST;
	  ////printf(" Hand is in manipulation config \n");
	}
    }
  //  else
  //  {
  //    ////printf(" Hand is ON a support \n");
  //    res_hand_mode=AGENT_HAND_AT_REST_MODE;
  //    
  //  }
 
  return 1;
}

extern int NEED_CURRENT_VISIBILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int NEED_ALL_VISIBILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int NEED_ALL_REACHABILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int NEED_CURRENT_REACHABILITY_UPDATE_AGENT[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern robots_status robots_status_for_Mightability_Maps[100];

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
      curr_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period=0;
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
      curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
      curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
      curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
      curr_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period=0;
      curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
    
      curr_config.diff_prev_config.agent_has_moved=-1;
      curr_config.diff_prev_config.agent_head_has_turned=-1;
      curr_config.diff_prev_config.agent_whole_body_has_turned=-1;
      curr_config.diff_prev_config.agent_torso_has_turned=-1;
      curr_config.diff_prev_config.agent_L_hand_has_moved=-1;
      curr_config.diff_prev_config.agent_R_hand_has_moved=-1;
    
    
      human1_prev_configs.agents_config_info.push_back(curr_config);
      return 0;
    }
  
  if(human1_prev_configs.agents_config_info.size()>=agents_for_ASA[for_agent].ASA_threshold[ASA_maxi_num_prev_states_to_store])
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
  //////////printf("Time diff from prev config= %lf milliseconds\n", curr_config.time_diff_from_prev_config);
  
  ////printf(" prev conf time=%lf, curr conf time= %lf, time period : %lf s\n",prev_config.clock, curr_config.clock, elapsedTime);
               
  ////printf(" prev conf time=%ld, curr conf time= %ld, time period : %lf s\n",prev_config.at_time/3600, curr_config.at_time/3600, elapsedTime);
  int Q_index_x=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_x;
  int Q_index_y=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_y;
  int Q_index_z=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_z;
    
  curr_threshold=agents_for_ASA[for_agent].ASA_threshold[ASA_agents_whole_body_pos_tolerance];
  ////////printf(" curr_threshold for whole body moved = %lf\n",curr_threshold);
    
    
  if(fabs(curr_config.config[Q_index_x]-prev_config.config[Q_index_x])>=curr_threshold||
     fabs(curr_config.config[Q_index_y]-prev_config.config[Q_index_y])>=curr_threshold||
     fabs(curr_config.config[Q_index_z]-prev_config.config[Q_index_z])>=curr_threshold)
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
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_is_moving];
  
  if(curr_config.diff_prev_config.agent_has_moved==1)
    {
      // If the agent has just moved then update only the current states of visibility for all the agents and current state of reachability only for the current agent 
      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	{
	  NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;
	  ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i];
	  ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	}
      NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
      robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
     
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
      if(prev_config.diff_prev_config.agent_has_moved==1)
	{
	  curr_config.conti_diff_info.agent_has_moved_conti_for_period=
	    prev_config.conti_diff_info.agent_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
	}
    
      if(curr_config.conti_diff_info.agent_has_moved_conti_for_period>=curr_min_time_period)
	{
	  ////////printf(" **** Human is moving \n");
	  Ag_Activity_Fact[for_agent].whole_body=AGENT_MOVING;
     
	  /*  // If agent is moving then update only the current states of visibility for all the agents and current state of reachability only for the current agent 
	      for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	      {
	      NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;
	      ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i];
	      ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	      }
	      NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
	  */
	}
      else
	{
	  // TODO Write code for appropriate updation
	}
    }
  else
    {
      curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
      curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_is_not_moving];
  
      if(prev_config.diff_prev_config.agent_has_moved==0)
	{
	  ////printf(" The agent has moved previously also\n");
	  curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=
	    prev_config.conti_diff_info.agent_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
	  ////printf(" First time the agent did not moved \n");
	  //This is the first time agent has been detected to be not moved so update all the revelant MA, as for the next time of has not moved no need to update any MA
	  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	    {
	      ////NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i];
	      NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=1;
	      ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	    }
	  NEED_ALL_REACHABILITY_UPDATE_AGENT[for_agent]=1;
	  robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
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
   
  curr_threshold=agents_for_ASA[for_agent].ASA_threshold[ASA_agents_whole_body_orient_tolerance];
   
  curr_config.diff_prev_config.agent_whole_body_has_turned=0;
  Ag_Activity_Fact[for_agent].whole_body_turn=AGENT_WHOLE_BODY_DID_NOT_TURN;
  
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||
     fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||
     fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
    {
      curr_config.diff_prev_config.agent_whole_body_has_turned=1;
      //////printf(">>>>>> Human whole body has turned \n");
      Ag_Activity_Fact[for_agent].whole_body_turn=AGENT_WHOLE_BODY_HAS_TURNED;
    
      ////NEED_ALL_VISIBILITY_UPDATE_AGENT[for_agent]=1;
      NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
      ////NEED_ALL_REACHABILITY_UPDATE_AGENT[for_agent]=1;
      NEED_CURRENT_VISIBILITY_UPDATE_AGENT[for_agent]=1;
      robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
    }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_whole_body_is_turning];
  
  if(curr_config.diff_prev_config.agent_whole_body_has_turned==1)
    {
      ////for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      ////{
      ////NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;
      ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i];
      ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
      ////}
      ////NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
      ////robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
     
      curr_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period=0;
      if(prev_config.diff_prev_config.agent_whole_body_has_turned==1)
	{
	  curr_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period=0;
	}
    
      if(curr_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period>=curr_min_time_period)
	{
	  //////printf(" **** Human is turning \n");
	  Ag_Activity_Fact[for_agent].whole_body_turn=AGENT_WHOLE_BODY_TURNING;
      
	}
    }
  else
    {
      curr_config.conti_diff_info.agent_whole_body_has_turned_conti_for_period=0;
    
      curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_whole_body_is_not_turning];
      if(prev_config.diff_prev_config.agent_whole_body_has_turned==0)
	{
	  curr_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period=0;
	  //This is the first time agent has been detected not to be turning after a turn so update all the revelant MA, as for the next time of has not moved no need to update any MA
	  ////for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	  ////{
	  ////NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i];
	  NEED_ALL_VISIBILITY_UPDATE_AGENT[for_agent]=1;
	  ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	  ////}
	  NEED_ALL_REACHABILITY_UPDATE_AGENT[for_agent]=1;
	  robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
	}
    
      if(curr_config.conti_diff_info.agent_whole_body_has_not_turned_conti_for_period>=curr_min_time_period)
	{
	  /////printf(" **** Human is NOT turning \n");
	  Ag_Activity_Fact[for_agent].whole_body_turn=AGENT_WHOLE_BODY_NOT_TURNING;
	}
    }
  
  Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_yaw;
  Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_pitch;
  Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_roll;
    
  curr_threshold=agents_for_ASA[for_agent].ASA_threshold[ASA_agents_torso_orient_tolerance];
    
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||
     fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||
     fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
    {
      curr_config.diff_prev_config.agent_torso_has_turned=1;
      ///////printf(">>>>>>>> Human torso has turned \n");
	Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_HAS_TURNED;
    
	NEED_CURRENT_VISIBILITY_UPDATE_AGENT[for_agent]=1;
	NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
	robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
    }
  
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_torso_is_turning];
  
  if(curr_config.diff_prev_config.agent_torso_has_turned==1)
    {
      ////for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
      ////{
      ////NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;
      ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i];
      ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
      ////}
      ////NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;
      ////robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
     
      curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
      if(prev_config.diff_prev_config.agent_torso_has_turned==1)
	{
	  curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_torso_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
	}
    
      if(curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period>=
	 curr_min_time_period)
	{
	  //////printf(" **** Human is turning \n");
	  Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_TURNING;
      
	}
    }
  else
    {
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    
      curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_torso_is_not_turning];
      if(prev_config.diff_prev_config.agent_torso_has_turned==0)
	{
	  curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
	}
      else
	{
	  curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
	  //This is the first time agent has been detected to be turning so update all the revelant MA, as for the next time of has not moved no need to update any MA
	  ////for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	  ////{
	  NEED_CURRENT_VISIBILITY_UPDATE_AGENT[for_agent]=1;;
	  ////NEED_ALL_VISIBILITY_UPDATE_AGENT[for_agent]=1;
	  ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	  ////}
	  NEED_CURRENT_REACHABILITY_UPDATE_AGENT[for_agent]=1;//As only the torso has turned
	  robots_status_for_Mightability_Maps[indices_of_MA_agents[for_agent]].has_moved=1;
	}
    
      if(curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period>=
	 curr_min_time_period)
	{
	  /////printf(" **** Human is NOT turning \n");
	  Ag_Activity_Fact[for_agent].torso=AGENT_TORSO_STATIC;
      
	}
    }
  
  
  Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_yaw;
  Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_pitch;
  Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_roll;
    
  curr_threshold=agents_for_ASA[for_agent].ASA_threshold[ASA_agents_head_orient_tolerance];
   
  
  ////printf(" Prev val of Head not turned continuously for period = %lf \n",prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period);
   
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold
     ||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold
     ||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
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
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_head_is_turning];
  if(curr_config.diff_prev_config.agent_head_has_turned==1)
    {
      NEED_CURRENT_VISIBILITY_UPDATE_AGENT[for_agent]=1;;
     
      curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
    
      if(prev_config.diff_prev_config.agent_head_has_turned==1)
	{
	  curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_head_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
      
	}
      else
	{
	  curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
      
	}
    
      if(curr_config.conti_diff_info.agent_head_has_turned_conti_for_period>=
	 curr_min_time_period)
	{
	  ////printf(" **** Human head is turning \n");
	  Ag_Activity_Fact[for_agent].head=AGENT_HEAD_TURNING;
	}
    }
  else
    {
      curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
    
      curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_head_is_not_turning];
    
      if(prev_config.diff_prev_config.agent_head_has_turned==0)
	{
	  ////printf(" Adding head not turned time, curr_not_turn_time=+=prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config 
	  curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=
	    prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
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
    curr_threshold=agents_for_ASA[for_agent].ASA_threshold[ASA_agents_hand_pos_tolerance];
   
  
    ////printf(" R_hand_pos= (%lf, %lf, %lf) \n", curr_config.R_hand_pos[0][3], curr_config.R_hand_pos[1][3], curr_config.R_hand_pos[2][3]);
  
    if(fabs(curr_config.R_hand_pos[0][3]-prev_config.R_hand_pos[0][3])>=curr_threshold||
       fabs(curr_config.R_hand_pos[1][3]-prev_config.R_hand_pos[1][3])>=curr_threshold||
       fabs(curr_config.R_hand_pos[2][3]-prev_config.R_hand_pos[2][3])>=curr_threshold)
      {
	curr_config.diff_prev_config.agent_R_hand_has_moved=1;
	//////printf(" >>>>>>>>>> Human's Right Hand has moved\n");
	Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_HAS_MOVED;
	for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	  {
	    NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;;
	    ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=1;
	    ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	  }
      }
    else
      {
	curr_config.diff_prev_config.agent_R_hand_has_moved=0;
	Ag_Activity_Fact[for_agent].right_hand=AGENT_HAND_DID_NOT_MOVE;
      }
  
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_hand_is_moving];
  
    if(curr_config.diff_prev_config.agent_R_hand_has_moved==1)
      {
	curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
	if(prev_config.diff_prev_config.agent_R_hand_has_moved==1)
	  {
	    curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=
	      prev_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
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
	curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_hand_is_not_moving];
	if(prev_config.diff_prev_config.agent_R_hand_has_moved==0)
	  {
	    curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=
	      prev_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
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
  
    ////get_agents_static_hand_mode(for_agent,MA_RIGHT_HAND, Ag_Activity_Fact[for_agent].right_hand_mode);
    get_agents_hand_info(for_agent,MA_RIGHT_HAND, Ag_Activity_Fact[for_agent].right_hand_mode, 
			 Ag_Activity_Fact[for_agent].right_hand_occup, 
			 Ag_Activity_Fact[for_agent].right_hand_rest_info);
  
  
    curr_config.diff_prev_config.agent_L_hand_has_moved=0;
    if(fabs(curr_config.L_hand_pos[0][3]-prev_config.L_hand_pos[0][3])>=curr_threshold||
       fabs(curr_config.L_hand_pos[1][3]-prev_config.L_hand_pos[1][3])>=curr_threshold||
       fabs(curr_config.L_hand_pos[2][3]-prev_config.L_hand_pos[2][3])>=curr_threshold)
      {
	curr_config.diff_prev_config.agent_L_hand_has_moved=1;
	///////printf(" >>>>>>>>>> Human's Left Hand has moved\n");
	Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_HAS_MOVED;
     
	for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
	  {
	    NEED_CURRENT_VISIBILITY_UPDATE_AGENT[i]=1;;
	    ////NEED_ALL_VISIBILITY_UPDATE_AGENT[i]=1;
	    ////    NEED_ALL_REACHABILITY_UPDATE_AGENT[i];
     
	  }
      }
    else
      {
	curr_config.diff_prev_config.agent_L_hand_has_moved=0;
	Ag_Activity_Fact[for_agent].left_hand=AGENT_HAND_DID_NOT_MOVE;
  
      }
  
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_hand_is_moving];
  
    if(curr_config.diff_prev_config.agent_L_hand_has_moved==1)
      {
	curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
	if(prev_config.diff_prev_config.agent_L_hand_has_moved==1)
	  {
	    curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=
	      prev_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
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
	curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold[ASA_min_period_for_agent_hand_is_not_moving];
	if(prev_config.diff_prev_config.agent_L_hand_has_moved==0)
	  {
	    curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=
	      prev_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
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
  
    ////get_agents_static_hand_mode(for_agent,MA_LEFT_HAND,Ag_Activity_Fact[for_agent].left_hand_mode);
    get_agents_hand_info(for_agent,MA_LEFT_HAND, 
			 Ag_Activity_Fact[for_agent].left_hand_mode, 
			 Ag_Activity_Fact[for_agent].left_hand_occup, 
			 Ag_Activity_Fact[for_agent].left_hand_rest_info);
  
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

int did_human_activity_facts_change(HRI_TASK_AGENT_ENUM for_agent, 
                                    agents_activity_facts &prev_fact, 
                                    agents_activity_facts &curr_fact)
{
  ///printf(" ========= Status for agent %s =======\n",envPt_ASA->robot[Ag_Activity_Fact[for_agent].agent_index]->name);
  
  if(prev_fact.whole_body!=curr_fact.whole_body)
    {
      return 1;
    }
  
  if(prev_fact.whole_body_turn!=curr_fact.whole_body_turn)
    {
      return 1;
    }
  
  if(prev_fact.torso!=curr_fact.torso)
    {
      return 1;
    }
  
  if(prev_fact.head!=curr_fact.head)
    {
      return 1;
    }
  
  if(prev_fact.right_hand!=curr_fact.right_hand)
    {
      return 1;
    }
  
  if(prev_fact.right_hand_mode!=curr_fact.right_hand_mode)
    {
      return 1;
    }
  
  if(prev_fact.right_hand_occup.occupancy_mode!=curr_fact.right_hand_occup.occupancy_mode)
    {
      return 1;
    }
  
  if(prev_fact.right_hand_rest_info.rest_type!=curr_fact.right_hand_rest_info.rest_type)
    {
      return 1;
    }
  
  if(prev_fact.left_hand!=curr_fact.left_hand)
    {
      return 1;
    }
  
  if(prev_fact.left_hand_mode!=curr_fact.left_hand_mode)
    {
      return 1;
    }
      
  if(prev_fact.left_hand_occup.occupancy_mode!=curr_fact.left_hand_occup.occupancy_mode)
    {
      return 1;
    }
  
  if(prev_fact.left_hand_rest_info.rest_type!=curr_fact.left_hand_rest_info.rest_type)
    {
      return 1;
    }
  
  return 0;
  ////Ag_Activity_Fact[for_agent].
}

int copy_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent, agents_activity_facts &from_fact, agents_activity_facts &to_fact)
{
  ///printf(" ========= Status for agent %s =======\n",envPt_ASA->robot[Ag_Activity_Fact[for_agent].agent_index]->name);
  to_fact.whole_body=from_fact.whole_body;
  to_fact.whole_body_turn=from_fact.whole_body_turn;
  to_fact.torso=from_fact.torso;
  to_fact.head=from_fact.head;
  to_fact.right_hand=from_fact.right_hand;
  to_fact.right_hand_mode=from_fact.right_hand_mode;
  to_fact.left_hand=from_fact.left_hand;
  to_fact.left_hand_mode=from_fact.left_hand_mode;
  
  to_fact.right_hand_occup.occupancy_mode=from_fact.right_hand_occup.occupancy_mode;
  to_fact.right_hand_occup.index_obj=from_fact.right_hand_occup.index_obj;
  strcpy(to_fact.right_hand_occup.object_in_hand,from_fact.right_hand_occup.object_in_hand);
  to_fact.right_hand_rest_info.rest_type=from_fact.right_hand_rest_info.rest_type;
  to_fact.right_hand_rest_info.index_obj=from_fact.right_hand_rest_info.index_obj;
  strcpy(to_fact.right_hand_rest_info.hand_on_support_obj,from_fact.right_hand_rest_info.hand_on_support_obj);
  
  to_fact.left_hand_occup.occupancy_mode=from_fact.left_hand_occup.occupancy_mode;
  to_fact.left_hand_occup.index_obj=from_fact.left_hand_occup.index_obj;
  strcpy(to_fact.left_hand_occup.object_in_hand,from_fact.left_hand_occup.object_in_hand);
  to_fact.left_hand_rest_info.rest_type=from_fact.left_hand_rest_info.rest_type;
  to_fact.left_hand_rest_info.index_obj=from_fact.left_hand_rest_info.index_obj;
  strcpy(to_fact.left_hand_rest_info.hand_on_support_obj,from_fact.left_hand_rest_info.hand_on_support_obj);
  return 1;
      
  ////Ag_Activity_Fact[for_agent].
}

int get_symbolic_name_human_activity_fact(HRI_TASK_AGENT_ENUM for_agent, agents_activity_facts &ag_act_fact, Agent_Activity_fact_by_names &Hum_facts_by_name)
{
  
  strcpy(Hum_facts_by_name.agent_motion, agent_motion_status_map[ag_act_fact.whole_body].c_str());
  
  strcpy(Hum_facts_by_name.body_turn, agent_whole_body_turn_status_map[ag_act_fact.whole_body_turn].c_str());
  strcpy(Hum_facts_by_name.torso_turn, agent_torso_status_map[ag_act_fact.torso].c_str());
  strcpy(Hum_facts_by_name.head_turn,agent_head_status_map[ag_act_fact.head].c_str());
    
     strcpy(Hum_facts_by_name.right_hand_status,agent_hand_status_map[ag_act_fact.right_hand].c_str());
     strcpy(Hum_facts_by_name.right_hand_config_mode,agent_hand_config_mode_map[ag_act_fact.right_hand_mode].c_str());
     
     if(ag_act_fact.right_hand_mode==AGENT_HAND_AT_REST_MODE)
     {
       strcpy(Hum_facts_by_name.right_hand_rest_mode_type,agent_hand_rest_info_map[ag_act_fact.right_hand_rest_info.rest_type].c_str());
       
       if(ag_act_fact.right_hand_rest_info.rest_type==AGENT_HAND_REST_ON_SUPPORT)
       { 
       strcpy(Hum_facts_by_name.right_hand_on_support, ag_act_fact.right_hand_rest_info.hand_on_support_obj);
       }
       else
       {
       strcpy(Hum_facts_by_name.right_hand_on_support,"NOT_RELEVANT");
       }
     }
     else
     {
       strcpy(Hum_facts_by_name.right_hand_rest_mode_type,"NOT_RELEVANT");
     }
     
     strcpy(Hum_facts_by_name.right_hand_occup_mode,agent_hand_occupancy_info_map[ag_act_fact.right_hand_occup.occupancy_mode].c_str());
     
     if(ag_act_fact.right_hand_occup.occupancy_mode==AGENT_HAND_HOLDING_OBJECT)
     {
       strcpy(Hum_facts_by_name.right_hand_holding_object,ag_act_fact.right_hand_occup.object_in_hand);
     }
     else
     {
       strcpy(Hum_facts_by_name.right_hand_holding_object,"NOT_RELEVANT");
     }
     
     
     strcpy(Hum_facts_by_name.left_hand_status,agent_hand_status_map[ag_act_fact.left_hand].c_str());
     strcpy(Hum_facts_by_name.left_hand_config_mode,agent_hand_config_mode_map[ag_act_fact.left_hand_mode].c_str());
     
     if(ag_act_fact.left_hand_mode==AGENT_HAND_AT_REST_MODE)
     {
       strcpy(Hum_facts_by_name.left_hand_rest_mode_type,agent_hand_rest_info_map[ag_act_fact.left_hand_rest_info.rest_type].c_str());
       
       if(ag_act_fact.left_hand_rest_info.rest_type==AGENT_HAND_REST_ON_SUPPORT)
       { 
       strcpy(Hum_facts_by_name.left_hand_on_support, ag_act_fact.left_hand_rest_info.hand_on_support_obj);
       }
        else
       {
       strcpy(Hum_facts_by_name.left_hand_on_support,"NOT_RELEVANT");
       }
     }
     else
     {
       strcpy(Hum_facts_by_name.left_hand_rest_mode_type,"NOT_RELEVANT");
     }
     
     strcpy(Hum_facts_by_name.left_hand_occup_mode,agent_hand_occupancy_info_map[ag_act_fact.left_hand_occup.occupancy_mode].c_str());
     
     if(ag_act_fact.left_hand_occup.occupancy_mode==AGENT_HAND_HOLDING_OBJECT)
     {
       strcpy(Hum_facts_by_name.left_hand_holding_object,ag_act_fact.left_hand_occup.object_in_hand);
     }
      else
     {
       strcpy(Hum_facts_by_name.left_hand_holding_object,"NOT_RELEVANT");
     }
     
  
}

int print_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent, agents_activity_facts &ag_act_fact)
{

  printf(" ========= Status for agent %s =======\n",envPt_ASA->robot[ag_act_fact.agent_index]->name);

  printf(" %s \n",agent_motion_status_map[ag_act_fact.whole_body].c_str());
  
  printf(" %s \n",agent_whole_body_turn_status_map[ag_act_fact.whole_body_turn].c_str());
  printf(" %s \n",agent_torso_status_map[ag_act_fact.torso].c_str());
  printf(" %s \n",agent_head_status_map[ag_act_fact.head].c_str());
    
  printf(" Right Hand Status: %s \n",agent_hand_status_map[ag_act_fact.right_hand].c_str());
  printf(" Right Hand Mode: %s \n",agent_hand_config_mode_map[ag_act_fact.right_hand_mode].c_str());
     
  if(ag_act_fact.right_hand_mode==AGENT_HAND_AT_REST_MODE)
    {
      printf(" >>> Rest mode type: %s \n",agent_hand_rest_info_map[ag_act_fact.right_hand_rest_info.rest_type].c_str());
       
      if(ag_act_fact.right_hand_rest_info.rest_type==AGENT_HAND_REST_ON_SUPPORT)
	{ 
	  printf(" >>> Resting on support: %s \n", ag_act_fact.right_hand_rest_info.hand_on_support_obj);
	}
    }
     
  printf(" Right Hand occupancy: %s \n",agent_hand_occupancy_info_map[ag_act_fact.right_hand_occup.occupancy_mode].c_str());
     
  if(ag_act_fact.right_hand_occup.occupancy_mode==AGENT_HAND_HOLDING_OBJECT)
    {
      printf(" >>> Holding Object %s \n",ag_act_fact.right_hand_occup.object_in_hand);
    }
     
     
  printf(" Left Hand Status: %s \n",agent_hand_status_map[ag_act_fact.left_hand].c_str());
  printf(" Left Hand Mode: %s \n",agent_hand_config_mode_map[ag_act_fact.left_hand_mode].c_str());
     
  if(ag_act_fact.left_hand_mode==AGENT_HAND_AT_REST_MODE)
    {
      printf(" >>> Rest mode type: %s \n",agent_hand_rest_info_map[ag_act_fact.left_hand_rest_info.rest_type].c_str());
       
      if(ag_act_fact.left_hand_rest_info.rest_type==AGENT_HAND_REST_ON_SUPPORT)
	{ 
	  printf(" >>> Resting on support: %s \n", ag_act_fact.left_hand_rest_info.hand_on_support_obj);
	}
    }
     
  printf(" Left Hand occupancy: %s \n",agent_hand_occupancy_info_map[ag_act_fact.left_hand_occup.occupancy_mode].c_str());
     
  if(ag_act_fact.left_hand_occup.occupancy_mode==AGENT_HAND_HOLDING_OBJECT)
    {
      printf(" >>> Holding Object %s \n",ag_act_fact.left_hand_occup.object_in_hand);
    }
     
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
	 if(did_human_activity_facts_change(HRI_TASK_AGENT_ENUM(i), Ag_Activity_Prev_Fact_for_print[i], Ag_Activity_Fact[i]))
	 {
	 get_symbolic_name_human_activity_fact(HRI_TASK_AGENT_ENUM(i),Ag_Activity_Fact[i], Hum_facts_by_name);
	 print_human_activity_facts(HRI_TASK_AGENT_ENUM(i), Ag_Activity_Fact[i]);
	 copy_human_activity_facts(HRI_TASK_AGENT_ENUM(i),Ag_Activity_Fact[i],Ag_Activity_Prev_Fact_for_print[i]);
	 }
       }

       
	  break;
	}
    }
  
}

int is_object_laying_on_a_support(int obj_index, int &support_index)
{
  ////////printf(" Inside is_object_laying_on_a_support\n");
  int kcd_with_report=0;
  double act_z_val=envPt_ASA->robot[obj_index]->joints[1]->abs_pos[2][3];
  int res = p3d_col_test_robot(envPt_ASA->robot[obj_index],kcd_with_report);

  if(res>0)
    {
      char obj_1[200], obj_2[200];
      char sup_obj_name[200];
      pqp_colliding_obj_name_pair(obj_1, obj_2);
      ////////printf("obj1= %s, obj2 =%s\n",obj_1,obj_2);
		
      if(strcasestr(obj_1,envPt_ASA->robot[obj_index]->name))
	{
	  //sup_obj=o2;
	  strcpy(sup_obj_name,obj_2);
                  
	  ////int coll_obj_index=get_index_of_robot_by_name(o2->name);
	  ////is_object_laying_on_a_support()
	}
      else
	{
	  //sup_obj=o1;
	  strcpy(sup_obj_name,obj_1);
	}
      ////////printf(" sup_obj_name=%s\n",sup_obj_name);
      int pt_ctr=0;
      char obj_name[100];
      while(sup_obj_name[pt_ctr]!='.'&&sup_obj_name[pt_ctr]!='\0')
	{
	  obj_name[pt_ctr]=sup_obj_name[pt_ctr];
	  pt_ctr++;
	}
      
      obj_name[pt_ctr]='\0';
      /*
	int obj_name_end_indx;
	obj_name_end_indx = strcspn (sup_obj_name,".");
	printf(" obj_name_end_indx=%d\n",obj_name_end_indx);
	char * obj_name;
	strncpy(obj_name,sup_obj_name,obj_name_end_indx);
	obj_name[obj_name_end_indx]='\0';
      */
      support_index=get_index_of_robot_by_name(obj_name);
      envPt_ASA->robot[obj_index]->joints[1]->abs_pos[2][3]=act_z_val;
      ////////printf(" support is %s, supp_index=%d\n",obj_name, support_index);
		  
      return 1;
      // 		p3d_obj *o1;
      // 		p3d_obj *o2;
      // 		printf(" From current pos \n");
      // 		pqp_colliding_pair(&o1, &o2);
      // 		printf(" obj1= %s, obj2 =%s\n",o1->name,o2->name);
      ////pqp_print_colliding_pair();
    }
	      
  envPt_ASA->robot[obj_index]->joints[1]->abs_pos[2][3]-=0.05;
	      
  res = p3d_col_test_robot(envPt_ASA->robot[obj_index],kcd_with_report);
  if(res>0)
    {
      ////////printf(" From down pos \n");
      char obj_1[200], obj_2[200];
      char sup_obj_name[200];
      pqp_colliding_obj_name_pair(obj_1, obj_2);
      ////////printf("obj1= %s, obj2 =%s\n",obj_1,obj_2);
		
      if(strcasestr(obj_1,envPt_ASA->robot[obj_index]->name))
	{
	  //sup_obj=o2;
	  strcpy(sup_obj_name,obj_2);
                  
	  ////int coll_obj_index=get_index_of_robot_by_name(o2->name);
	  ////is_object_laying_on_a_support()
	}
      else
	{
	  //sup_obj=o1;
	  strcpy(sup_obj_name,obj_1);
	}
      ////////printf(" sup_obj_name=%s\n",sup_obj_name);
      int pt_ctr=0;
      char obj_name[100];
      while(sup_obj_name[pt_ctr]!='.')
	{
	  obj_name[pt_ctr]=sup_obj_name[pt_ctr];
	  pt_ctr++;
	}

      obj_name[pt_ctr]='\0';
      /*
	int obj_name_end_indx;
	obj_name_end_indx = strcspn (sup_obj_name,".");
	printf(" obj_name_end_indx=%d\n",obj_name_end_indx);
	char * obj_name;
	strncpy(obj_name,sup_obj_name,obj_name_end_indx);
	obj_name[obj_name_end_indx]='\0';
      */
      support_index=get_index_of_robot_by_name(obj_name);
		   
      ////////printf(" support is %s, supp_index=%d\n",obj_name, support_index);
      envPt_ASA->robot[obj_index]->joints[1]->abs_pos[2][3]=act_z_val;
      return 1;
      // 		p3d_obj *o1;
      // 		p3d_obj *o2;
      // 		
      // 		pqp_colliding_pair(&o1, &o2);
      // 		printf(" obj1= %s, obj2 =%s\n",o1->name,o2->name);
      ////pqp_print_colliding_pair();
    } 
  envPt_ASA->robot[obj_index]->joints[1]->abs_pos[2][3]=act_z_val;
  return 0;
}
