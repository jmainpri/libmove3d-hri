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

#include <Util-pkg.h>
#include <P3d-pkg.h>

#include "Agent_State_Analysis.h"
#include "Agent_State_Analysis_proto.h"
#include "Mightability_Analysis.h"

extern int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern int get_indices_for_MA_agents();

p3d_env *envPt_ASA;//ASA:Agent's State Analysis

agents_prev_conf_info human1_prev_configs;

agents_info_for_ASA agents_for_ASA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int find_activity_fact_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

agents_activity_facts Ag_Activity_Fact[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];

int agents_for_ASA_initialized=0;

// Internal functions
static int get_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent);
static int init_agent_state_analysis();
static int init_enable_agents_for_facts();
static int get_agents_activity_facts(int find_facts_for[MAXI_NUM_OF_AGENT_FOR_HRI_TASK]);
static int init_thresholds_for_ASA();

int hri_execute_Agent_State_Analysis_functions()
{
  printf(" Inside hri_execute_Agent_State_Analysis_functions()\n");
  if(agents_for_ASA_initialized==0)
  {
    get_indices_for_MA_agents();
    init_enable_agents_for_facts();
    init_agent_state_analysis();
    init_thresholds_for_ASA();
  }
  ////get_human_activity_facts(human1_fact );
  //////get_human_activity_facts();
 get_agents_activity_facts(find_activity_fact_for);
  
}

int init_thresholds_for_ASA()
{
  for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    switch(i)
    {
      case HUMAN1_MA:
	agents_for_ASA[HUMAN1_MA].ASA_threshold.maxi_num_prev_states_to_store=100;
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_whole_body_pos_tolerance=0.05;//in m
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_whole_body_orient_tolerance=DTOR(5);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_torso_orient_tolerance=DTOR(5);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_head_orient_tolerance=DTOR(5);//rad
	
	agents_for_ASA[HUMAN1_MA].ASA_threshold.agents_hand_pos_tolerance=0.03;//in m
	
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
      agents_for_ASA[i].agent_index=get_index_of_robot_by_name("JIDOKUKA_ROBOT");
      strcpy(agents_for_ASA[i].agent_name,"JIDOKUKA_ROBOT");
      
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
	 indices_of_MA_agents[i]=get_index_of_robot_by_name("HRP2_ROBOT");
      break;
#endif

      case HUMAN1_MA:
	agents_for_ASA[i].agent_index=get_index_of_robot_by_name("ACHILE_HUMAN1");
      strcpy(agents_for_ASA[i].agent_name,"ACHILE_HUMAN1");
      
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
      printf(">><< rPalm jnt index=%d\n",agents_for_ASA[i].joint_indx.R_hand_jnt);
      
      agents_for_ASA[i].joint_indx.L_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lPalm");
      
      
      ////agents_for_ASA[i].joint_indx.L_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lPalm");
      
      break;

#ifdef HUMAN2_EXISTS_FOR_MA
      case HUMAN2_MA:
	agents_for_ASA[i].agent_index=get_index_of_robot_by_name("ACHILE_HUMAN2");
      strcpy(agents_for_ASA[i].agent_name,"ACHILE_HUMAN2");
      
      agents_for_ASA[i].joint_indx.body_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "Pelvis");
      
      agents_for_ASA[i].joint_indx.torso_yaw=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoX");;
      agents_for_ASA[i].joint_indx.torso_pitch=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoY");;
      agents_for_ASA[i].joint_indx.torso_roll=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "TorsoZ");;
      
      agents_for_ASA[i].joint_indx.head_yaw=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadZ");
      
      agents_for_ASA[i].joint_indx.head_pitch=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadY");
      
      agents_for_ASA[i].joint_indx.head_roll=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "HeadX");
      
      agents_for_ASA[i].joint_indx.R_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "rPalm");
      
      agents_for_ASA[i].joint_indx.L_hand_jnt=p3d_get_robot_jnt_index_by_name(envPt_ASA->robot[agents_for_ASA[i].agent_index], (char*) "lPalm");
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


int get_human_activity_facts(HRI_TASK_AGENT_ENUM for_agent )
{
  double curr_threshold;
  double curr_min_time_period;
  
  int index=indices_of_MA_agents[for_agent];
  ////envPt_ASA = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  printf("Inside get_human_activity_facts()\n");
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
    printf(" Forgetting the oldest configuration of human \n");  
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
     printf(" curr_config.at_time.tv_sec = %ld, prev_config.at_time.tv_sec =%ld\n",curr_config.at_time.tv_sec,prev_config.at_time.tv_sec);
     
      printf(" curr_config.at_time.tv_usec = %ld, prev_config.at_time.tv_usec =%ld\n",curr_config.at_time.tv_usec,prev_config.at_time.tv_usec);
     printf(" seconds = %ld, useconds =%ld\n",seconds,useconds);
     
     
      double elapsedTime = ((seconds) * 1000 + useconds/1000.0) ;
    
    printf("Elapsed time: %lf milliseconds\n", elapsedTime);

  curr_config.time_diff_from_prev_config=elapsedTime;
  
  ////printf(" prev conf time=%lf, curr conf time= %lf, time period : %lf s\n",prev_config.clock, curr_config.clock, elapsedTime);
               
  ////printf(" prev conf time=%ld, curr conf time= %ld, time period : %lf s\n",prev_config.at_time/3600, curr_config.at_time/3600, elapsedTime);
    int Q_index_x=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_x;
    int Q_index_y=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_y;
    int Q_index_z=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_z;
    
    curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_whole_body_pos_tolerance;
    printf(" curr_threshold for whole body moved = %lf\n",curr_threshold);
    
    curr_config.diff_prev_config.agent_has_moved=0;
  if(fabs(curr_config.config[Q_index_x]-prev_config.config[Q_index_x])>=curr_threshold||fabs(curr_config.config[Q_index_y]-prev_config.config[Q_index_y])>=curr_threshold||fabs(curr_config.config[Q_index_z]-prev_config.config[Q_index_z])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_has_moved=1;
     printf(">>>> Human whole body has moved \n");
     
   
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_moving;
  
  if(curr_config.diff_prev_config.agent_has_moved==1)
  {
   if(prev_config.diff_prev_config.agent_has_moved==1)
    {
      curr_config.conti_diff_info.agent_has_moved_conti_for_period+=prev_config.conti_diff_info.agent_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_has_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human is moving \n");
    }
  }
  else
  {
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_not_moving;
  
    if(prev_config.diff_prev_config.agent_has_moved==0)
    {
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period+=prev_config.conti_diff_info.agent_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human is NOT moving \n");
    }
  }
  
   int Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_yaw;
   int Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_pitch;
   int Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.body_Q_roll;
   
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_whole_body_orient_tolerance;
   
  curr_config.diff_prev_config.agent_torso_has_turned=0;
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_torso_has_turned=1;
    printf(">>>>>> Human whole body has turned \n");
  }
  
   Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_yaw;
   Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_pitch;
    Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.torso_Q_roll;
    
    curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_torso_orient_tolerance;
   
    
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_torso_has_turned=1;
    printf(">>>>>>>> Human torso has turned \n");
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_turning;
  
  if(curr_config.diff_prev_config.agent_torso_has_turned==1)
  {
  if(prev_config.diff_prev_config.agent_torso_has_turned==1)
    {
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period+=prev_config.conti_diff_info.agent_torso_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_torso_has_turned_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human is turning \n");
      
    }
  }
  else
  {
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_is_not_turning;
    if(prev_config.diff_prev_config.agent_torso_has_turned==0)
    {
      curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period+=prev_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
       curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_torso_has_not_turned_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human is NOT turning \n");
      
    }
  }
  
  
   Q_index_yaw=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_yaw;
   Q_index_pitch=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_pitch;
   Q_index_roll=agents_for_ASA[HUMAN1_MA].Q_indx.head_Q_roll;
    
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_head_orient_tolerance;
   
  curr_config.diff_prev_config.agent_head_has_turned=0;
  
  if(fabs(curr_config.config[Q_index_yaw]-prev_config.config[Q_index_yaw])>=curr_threshold||fabs(curr_config.config[Q_index_pitch]-prev_config.config[Q_index_pitch])>=curr_threshold||fabs(curr_config.config[Q_index_roll]-prev_config.config[Q_index_roll])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_head_has_turned=1;
    printf(">>>>>>>> Human head has turned \n");
  }
  
   curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_head_is_turning;
  if(curr_config.diff_prev_config.agent_head_has_turned==1)
  {
  if(prev_config.diff_prev_config.agent_head_has_turned==1)
    {
      curr_config.conti_diff_info.agent_head_has_turned_conti_for_period+=prev_config.conti_diff_info.agent_head_has_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_head_has_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_head_has_turned_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human head is turning \n");
      
    }
  }
  else
  {
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_head_is_not_turning;
    if(prev_config.diff_prev_config.agent_head_has_turned==0)
    {
      curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+=prev_config.conti_diff_info.agent_head_has_not_turned_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_head_has_not_turned_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human head is NOT turning \n");
      
    }
  }
   
  /////Now for the hand movement
   curr_threshold=agents_for_ASA[for_agent].ASA_threshold.agents_hand_pos_tolerance;
   
  curr_config.diff_prev_config.agent_R_hand_has_moved=0;
  ////printf(" R_hand_pos= (%lf, %lf, %lf) \n", curr_config.R_hand_pos[0][3], curr_config.R_hand_pos[1][3], curr_config.R_hand_pos[2][3]);
  
  if(fabs(curr_config.R_hand_pos[0][3]-prev_config.R_hand_pos[0][3])>=curr_threshold||fabs(curr_config.R_hand_pos[1][3]-prev_config.R_hand_pos[1][3])>=curr_threshold||fabs(curr_config.R_hand_pos[2][3]-prev_config.R_hand_pos[2][3])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_R_hand_has_moved=1;
    printf(" >>>>>>>>>> Human's Right Hand has moved\n");
    
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_moving;
  
  if(curr_config.diff_prev_config.agent_R_hand_has_moved==1)
  {
  if(prev_config.diff_prev_config.agent_R_hand_has_moved==1)
    {
      curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period+=prev_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_R_hand_has_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human Right Hand is Moving \n");
      
    }
  }
  else
  {
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_not_moving;
    if(prev_config.diff_prev_config.agent_R_hand_has_moved==0)
    {
      curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period+=prev_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_R_hand_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human Right Hand is NOT Moving \n");
      
    }
  }
  
  
  curr_config.diff_prev_config.agent_L_hand_has_moved=0;
  if(fabs(curr_config.L_hand_pos[0][3]-prev_config.L_hand_pos[0][3])>=curr_threshold||fabs(curr_config.L_hand_pos[1][3]-prev_config.L_hand_pos[1][3])>=curr_threshold||fabs(curr_config.L_hand_pos[2][3]-prev_config.L_hand_pos[2][3])>=curr_threshold)
  {
    curr_config.diff_prev_config.agent_L_hand_has_moved=1;
     printf(" >>>>>>>>>> Human's Left Hand has moved\n");
  }
  
  curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_moving;
  
  if(curr_config.diff_prev_config.agent_L_hand_has_moved==1)
  {
  if(prev_config.diff_prev_config.agent_L_hand_has_moved==1)
    {
      curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period+=prev_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
      curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_L_hand_has_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human Left Hand is Moving \n");
      
    }
  }
  else
  {
    curr_min_time_period=agents_for_ASA[for_agent].ASA_threshold.min_period_for_agent_hand_is_not_moving;
    if(prev_config.diff_prev_config.agent_L_hand_has_moved==0)
    {
      curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period+=prev_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period+curr_config.time_diff_from_prev_config;
    }
    else
    {
     curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period=0;
    }
    
    if(curr_config.conti_diff_info.agent_L_hand_has_not_moved_conti_for_period>=curr_min_time_period)
    {
      printf(" **** Human Left Hand is NOT Moving \n");
      
    }
  }
  
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