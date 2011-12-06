//
// C++ Implementation: HRI_tasks
//
// Description: 
//
//
// Author: Amit Kumar Pandey <akpandey@verdier.laas.fr>, (C) 2010
//
// Copyright: See COPYING file that comes with this distribution
//
/*
 cd /home/akpandey/AKP_modules/BioMove3D_New3/build/;
 /home/akpandey/AKP_modules/BioMove3D_New3/build/Debug/bin/i386-linux/move3d-studio -f /home/akpandey/AKP_modules/mhp_new/BioMove3DDemos_new3/GS/gsJidoKukaSAHandSM_MA_new.p3d -sc /home/akpandey/AKP_modules/mhp_new/BioMove3DDemos_new3/GS/SCENARIO/ManipulationTestSAHand_MA_new4.sce -dmax 0.01
*/

#include <list>
#include <string>
#include <iostream>
#include <map>

#include <Util-pkg.h>
#include <P3d-pkg.h>
#include <Rrt-pkg.h>
#include <Planner-pkg.h>
#include <Localpath-pkg.h>
#include <Collision-pkg.h>
#include <Graphic-pkg.h>
#include <GraspPlanning-pkg.h>
#include <math.h>
#include <../lightPlanner/proto/lightPlannerApi.h>
#include <../lightPlanner/proto/ManipulationPlanner.hpp>
#include <../lightPlanner/proto/ManipulationUtils.hpp>
#include <../lightPlanner/proto/ManipulationArmData.hpp>
#include <../lightPlanner/proto/ManipulationStruct.hpp>
#include <../lightPlanner/proto/robotPos.h>

#include "hri.h"

#include "hri_bitmap/hri_bitmap_util.h"
#include "hri_bitmap/hri_bitmap_draw.h"
#include "hri_bitmap/hri_bitmap_cost.h"
#include "hri_bitmap/hri_bitmap_bin_heap.h"
#include "HRI_tasks.h"
#include "Mightability_Analysis.h"
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <hri_affordance_include_proto.h>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <QtCore/qshareddata.h>

using namespace std;
HRI_TASK_TYPE CURRENT_HRI_MANIPULATION_TASK;
//candidate_poins_for_task candidate_points_to_put;
//candidate_poins_for_task candidate_points_to_show;
//candidate_poins_for_task candidate_points_to_hide;
//candidate_poins_for_task candidate_points_to_hide_away;
//candidate_poins_for_task candidate_points_to_give;
//candidate_poins_for_task candidate_points_to_putinto_by_jido;
//candidate_poins_for_task candidate_points_to_putinto_blue_trashbin_by_jido;
//candidate_poins_for_task candidate_points_to_putinto_pink_trashbin_by_jido;
//candidate_poins_for_task current_candidate_points_to_putinto;
//candidate_poins_for_task candidate_points_to_put_away;
//candidate_poins_for_task candidate_points_to_displace_obj;

candidate_poins_for_task *CANDIDATE_POINTS_FOR_CURRENT_TASK;
candidate_poins_for_task resultant_current_candidate_point;

extern int UPDATE_MIGHTABILITY_MAP_INFO;
extern int SHOW_MIGHTABILITY_MAP_INFO;
extern char CURRENT_OBJECT_TO_MANIPULATE[50];

extern p3d_env *envPt_MM;
////extern HRI_AGENT * primary_human_MM;
////extern HRI_AGENT * jido_robot_MM;
////extern HRI_AGENT * hrp2_robot_MM;
extern struct robots_indices rob_indx;
static ManipulationPlanner *manipulation= NULL;

//extern candidate_poins_for_task resultant_current_candidate_point;
extern int indices_of_MA_agents[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_BY;
extern HRI_TASK_AGENT CURRENT_TASK_PERFORMED_FOR;
extern int CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS;
extern HRI_AGENT *HRI_AGENTS_FOR_MA[MAXI_NUM_OF_AGENT_FOR_HRI_TASK];
extern double mini_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];
extern double maxi_visibility_threshold_for_task[MAXI_NUM_OF_HRI_TASKS];
extern struct object_Symbolic_Mightability_Maps_Relation object_MM;

////action_performance_node curret_task;
std::vector<HRI_task_node> HRI_task_list;
std::map<int,std::string > HRI_task_NAME_ID_map;
////std::map<std::string,int > HRI_task_ID_NAME_map;
std::map<int,std::string > HRI_sub_task_NAME_ID_map;
std::map<std::string, int > HRI_task_plan_DESC_ID_map;
world_state_configs final_configs_after_a_task;

int CURRENT_HRI_TASK_PLAN_ID_TO_SHOW;
int INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW;
int SHOW_HRI_TASK_TRAJ_TYPE;
int SHOW_HRI_PLAN_TYPE;
////int CURRENT_TASKABILITY_NODE_ID_TO_SHOW;
////int CURRENT_MANIPULABILITY_NODE_ID_TO_SHOW;

int JIDO_HAND_TYPE=1;//1 for gripper, 2 for SAHAND

std::list<gpGrasp> *CURRENT_ALL_GRASP_FOR_OBJECT;
std::list<gpGrasp> *CURRENT_CANDIDATE_GRASP_FOR_TASK;
std::list<gpPlacement> *CURRENT_CANDIDATE_PLACEMENT_LIST;

int IS_PERFORMING_AGENT_MASTER;
int TASK_IS_FOR_PROACTIVE_BEHAVIOR;
int HRI_TASK_PLAN_IN_CARTESIAN=0;

char SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ[50];

extern p3d_matrix4 WRIST_FRAME;
extern p3d_matrix4 HEAD_FRAME;

int AT_LEAST_1_GRASP_LIFT_FOUND=0;//Used to break the task planner for next effort level if the robot failed to find the path to pick the object at first effort level itself
extern int SHOW_HOW_TO_PLACE_AT;
extern point_co_ordi TO_SHOW_PLACEMENT_POINT;
int cells_tested_for_current_task[100][100][100];//TODO Allocate memory dynamically based upon the actual dimension of the 3D grid

int TASK_CURRENTLY_SUPPORTED[MAXI_NUM_OF_HRI_TASKS]; //To keep track of tasks which have been fully implemented

extern analysis_type_effort_level_group Analysis_type_Effort_level[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_ABILITY_TYPE_FOR_EFFORT][50];//3rd index will be synchronized with the enum of the corresponding effort levels

// Will be used at various places to check the manipulability of an object by the agent
// NOTE: Don't forget to add any new object for which the grasp has been calculated or the grasp file exists and increse the NUM_VALID_GRASPABLE_OBJ value
char MANIPULABLE_OBJECTS[MAXI_NUM_OF_ALLOWED_OBJECTS_IN_ENV][50]={"LOTR_TAPE","GREY_K7","GREY_TAPE","SURPRISE_BOX","TOYCUBE_WOOD"};
int NUM_VALID_GRASPABLE_OBJ=5;

int GRASP_EXISTS_FOR_OBJECT[MAXI_NUM_OF_ALLOWED_OBJECTS_IN_ENV];

extern int HRP2_CURRENT_STATE;
extern int HUMAN1_CURRENT_STATE_MM;
#ifdef HUMAN2_EXISTS_FOR_MA
extern int HUMAN2_CURRENT_STATE_MM;//HRI_STANDING;
#endif
#ifdef PR2_EXISTS_FOR_MA
extern int PR2_CURRENT_POSTURE;
#endif

//TODO : Put below in HRI_tasks_Proto.h

int get_ranking_based_on_view_point(p3d_matrix4 view_frame,point_co_ordi point,p3d_rob *object, p3d_rob *human, std::list<gpPlacement> &placementList);
int get_placements_at_position(p3d_rob *object, point_co_ordi point, std::list<gpPlacement> placementList, int no_rot, std::list<gpPlacement> &placementListOut);
int copy_HRI_task_candidate_points(candidate_poins_for_task *from_candidate_points, candidate_poins_for_task *to_candidate_points);

void fct_draw_loop();

void* (*XFORM_update_func)()=NULL;

int default_drawtraj_fct_without_XFORM(p3d_rob* robot, p3d_localpath* curLp);

int (*default_drawtraj_fct_ptr)(p3d_rob* robot, p3d_localpath* curLp)=default_drawtraj_fct_without_XFORM;

int set_current_HRI_manipulation_task(int arg)
{
 switch(arg)
 {
 case 0:
 CURRENT_HRI_MANIPULATION_TASK=MAKE_OBJECT_ACCESSIBLE;
 break;
 case 1:
 CURRENT_HRI_MANIPULATION_TASK=SHOW_OBJECT;
 break;
 case 2:
 CURRENT_HRI_MANIPULATION_TASK=GIVE_OBJECT;
 break;
 case 3:
 CURRENT_HRI_MANIPULATION_TASK=HIDE_OBJECT;
 break;
 case 4:
 CURRENT_HRI_MANIPULATION_TASK=PUT_AWAY_OBJECT;
 break;
 case 5:
 CURRENT_HRI_MANIPULATION_TASK=HIDE_AWAY_OBJECT;
 break;
 case 6:
 CURRENT_HRI_MANIPULATION_TASK=MAKE_SPACE_FREE_OF_OBJECT;
 break;
 case 7:
 CURRENT_HRI_MANIPULATION_TASK=PUT_INTO_OBJECT;
 break;
 }
}



int get_HRI_task_id_type_by_name(char task_name[50], int &task_type)
{
  std::map<int, string>::iterator it;
 
 for(it = HRI_task_NAME_ID_map.begin(); it != HRI_task_NAME_ID_map.end(); ++it)
 {
 if(strcasestr(task_name,it->second.c_str()))
  {
   task_type=(HRI_TASK_TYPE)it->first;
   return 1;
  
  }
 
 }
 
 printf(" Task name %s in unknown \n",task_name);
 return -1;
}

int convert_symbolic_HRI_task_desc_into(symbolic_HRI_task_desc &HRI_task_ip, HRI_task_desc &task_to_validate)
{ // map<char,int>::iterator it;
 printf(" Inside convert_symbolic_HRI_task_desc_into() with task name= %s, for object %s, by agent = %s, for agent= %s\n",HRI_task_ip.task_name,HRI_task_ip.for_object, HRI_task_ip.by_agent,HRI_task_ip.for_agent);
 
 std::map<int, string>::iterator it;
 int task_known=0;
 for(it = HRI_task_NAME_ID_map.begin(); it != HRI_task_NAME_ID_map.end(); ++it)
 {
 if(strcasestr(HRI_task_ip.task_name,it->second.c_str()))
  {
   task_to_validate.task_type=(HRI_TASK_TYPE)it->first;
   task_known=1;
   break;
  }
 
 }
 
 if(task_known==0)
 {
 printf("  >>>> HRI_TASK ERROR : The task to perform is not known. \n"); 
 return -1;
 }

 int for_object_index=get_index_of_robot_by_name(HRI_task_ip.for_object);
 if(for_object_index==-1)
 {
  printf(" >>>> HRI_TASK ERROR : The object %s for which the task is to be performed is not known. \n",HRI_task_ip.for_object); 
  return -2;
 }
 task_to_validate.for_object=HRI_task_ip.for_object;
 
 int by_agent_index=get_index_of_robot_by_name(HRI_task_ip.by_agent);
 if(by_agent_index==-1)
 {
  printf(" >>>> HRI_TASK ERROR : The agent %s to perform the task is not known. \n",HRI_task_ip.by_agent); 
  return -3;
 }

 int for_agent_index=get_index_of_robot_by_name(HRI_task_ip.for_agent);
 if(for_agent_index==-1)
 {
  printf(" >>>> HRI_TASK ERROR : The agent %s for whom the task is to be performed is not known. \n",HRI_task_ip.for_agent); 
  return -4;
 }

 for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
 {
   if(indices_of_MA_agents[i]==by_agent_index)
   {
     task_to_validate.by_agent=(HRI_TASK_AGENT)i;
     break;
   }
 }
 
 for(int i=0;i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
 {
   if(indices_of_MA_agents[i]==for_agent_index)
   {
     task_to_validate.for_agent=(HRI_TASK_AGENT)i;
     break;
   }
 }
 
  
  
 return 1;
 
}

static void initManipulation() {
  if (manipulation == NULL) {
	p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");//justin//JIDOKUKA_ROBOT
	manipulation= new ManipulationPlanner(robotPt);
//         manipulation->setArmType(GP_LWR); // set the arm type
  }
  return;
}

int init_manipulation_planner(char robot_name[100])
{
if ( manipulation== NULL )
   {
      ////p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");//justin//JIDOKUKA_ROBOT
      p3d_rob * robotPt= p3d_get_robot_by_name(robot_name);
      manipulation= new ManipulationPlanner(robotPt);
//         manipulation->setArmType(GP_LWR); // set the arm type
   }
return 1;
}



int get_grasp_list_for_object(char *obj_to_manipulate, std::list<gpGrasp> &graspList)
{
gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();

printf(" armHandProp.type=%d\n",armHandProp.type);

 gpGet_grasp_list ( obj_to_manipulate, armHandProp.type, graspList );
 
 if(graspList.size()>=1)
return 1;
 else
return 0;
}

int reduce_grasp_list_for_hand_over_task(std::list<gpGrasp> &orig_graspList, char* obj_to_manipulate, char* from_hand, char* to_hand, std::list<gpGrasp> &res_graspList )
{
 p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
// std::list<gpGrasp> graspList2;
// graspList2= graspList;
 gpGrasp_handover_filter(p3d_get_robot_by_name ( from_hand ), p3d_get_robot_by_name ( to_hand ), object, res_graspList, orig_graspList);
  printf(" After gpGrasp_handover_filter()\n");
  printf(" orig_graspList.size()=%d,res_graspList.size()=%d\n",orig_graspList.size(),res_graspList.size());
  
  return 1;
}

int show_current_task_candidate_points(int show_weight_by_color, int show_weight_by_length, candidate_poins_for_task *candidate_points)
{
////////printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE);
/*
 switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 case MAKE_OBJECT_ACCESSIBLE:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_put;
  
  ////show_weighted_candidate_points_to_put_obj(show_weight_by_color);
 break;
 case SHOW_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_show;
  ////show_weighted_candidate_points_to_show_obj(show_weight_by_color);
 break;
 case GIVE_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_give;
  ////show_weighted_candidate_points_to_give_obj(show_weight_by_color);
 break;
 case HIDE_OBJECT:
  CANDIDATE_POINTS_FOR_CURRENT_TASK=&candidate_points_to_hide;
  ////show_weighted_candidate_points_to_hide_obj();
 break;
 }
*/
printf("candidate_points->no_points=%d\n",candidate_points->no_points);
CANDIDATE_POINTS_FOR_CURRENT_TASK=candidate_points;
show_candidate_points_for_current_task(show_weight_by_color,show_weight_by_length);
 
}

int find_HRI_task_candidate_points(HRI_TASK_TYPE CURR_TASK, char *obj_to_manipulate, HRI_TASK_AGENT performed_by,  HRI_TASK_AGENT performed_for, int performing_agent_rank, candidate_poins_for_task *curr_resultant_candidate_points, int consider_object_dimension)
{
  
  ////int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
    
      ////find_candidate_points_for_current_HRI_task(CURR_TASK,  for_agent, JIDO_MA,curr_resultant_candidate_points);
   ////if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==0)
   /////find_candidate_points_for_current_HRI_task(CURR_TASK,  performed_by, performed_for,curr_resultant_candidate_points);
   
   /////if(CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS==1)
   init_visibility_acceptance_for_tasks();
 
 
   find_candidate_points_for_current_HRI_task_for_object(CURR_TASK,  performed_by, performed_for,performing_agent_rank, curr_resultant_candidate_points, obj_to_manipulate, consider_object_dimension);
   
   
   switch (CURR_TASK)
   {
     case MAKE_OBJECT_ACCESSIBLE:
        assign_weights_on_candidte_points_to_put_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
     break;
     
     case SHOW_OBJECT:
	////assign_weights_on_candidte_points_to_show_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
	assign_weights_on_candidte_points_to_show_obj_new(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for], performing_agent_rank);
      break;
      
     case GIVE_OBJECT:
       
       assign_weights_on_candidte_points_to_give_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for],performing_agent_rank);
      break;
      
      case HIDE_OBJECT:
       assign_weights_on_candidte_points_to_hide_obj(obj_to_manipulate, curr_resultant_candidate_points, indices_of_MA_agents[performed_by], indices_of_MA_agents[performed_for]);
      break;
      
   }
  
 
   reverse_sort_HRI_task_weighted_candidate_points(curr_resultant_candidate_points);
    

   CANDIDATE_POINTS_FOR_TASK_FOUND=1;
   
   ////CANDIDATE_POINTS_FOR_CURRENT_TASK=curr_resultant_candidate_points;
   copy_HRI_task_candidate_points(curr_resultant_candidate_points,&resultant_current_candidate_point);
   CANDIDATE_POINTS_FOR_CURRENT_TASK=&resultant_current_candidate_point;
   ////MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);
    

     return 1;
}
/*
int store_initial_world_state_for_action(action_performance_node curret_task, std::list<action_performance_node> &task_list)
{
    action_performance_node tmp_node;
 printf(" Inside store_current_world_state()\n");

 strcpy(tmp_node.action.action_name,curret_task.action_name);
 strcpy(tmp_node.action.active_agent_name,curret_task.active_agent_name);
 strcpy(tmp_node.action.object_name,curret_task.object_name);
 strcpy(tmp_node.action.target_agent_name,curret_task.target_agent_name);

    int nr = envPt_MM->nr;
    int nr_ctr=0;
    for(;nr_ctr<nr;nr_ctr++)
     {
       tmp_node.WS_before_exec.robot_config[nr_ctr]=MY_ALLOC(double,envPt_MM->robot[nr_ctr]->nb_dof); 
       p3d_get_robot_config_into(envPt_MM->robot[nr_ctr],&tmp_node.WS_before_exec.robot_config[nr_ctr]);
     }   
   
   
}
*/
int get_robot_proactive_solution_info( HRI_task_desc curr_task, traj_for_HRI_task &res_traj)
{

CURRENT_HRI_MANIPULATION_TASK=curr_task.task_type;
CURRENT_TASK_PERFORMED_BY=curr_task.by_agent;
CURRENT_TASK_PERFORMED_FOR=curr_task.for_agent;
strcpy(CURRENT_OBJECT_TO_MANIPULATE,curr_task.for_object.c_str());

printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s, TASK_PERFORMED_BY=%d, TASK_PERFORMED_FOR=%d\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE, CURRENT_TASK_PERFORMED_BY, CURRENT_TASK_PERFORMED_FOR);

#ifdef JIDO_EXISTS_FOR_MA
if(CURRENT_TASK_PERFORMED_BY==JIDO_MA||CURRENT_TASK_PERFORMED_FOR==JIDO_MA)
 {
 //case JIDO_MA:
  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[JIDO_MA]]->name);
// break;

 }
#endif

#ifdef PR2_EXISTS_FOR_MA
 if(CURRENT_TASK_PERFORMED_BY==PR2_MA||CURRENT_TASK_PERFORMED_FOR==PR2_MA)
 {
 //case JIDO_MA:
  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[PR2_MA]]->name);
// break;

 }
#endif

candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
std::list<gpGrasp> grasps_for_object;
std::list<gpGrasp> candidate_grasps_for_task;
std::list<gpPlacement> curr_placementList;

int performing_agent_rank;
if(IS_PERFORMING_AGENT_MASTER==1)
performing_agent_rank=1;//Master
else
performing_agent_rank=0;//Slave

find_HRI_task_candidate_points(CURRENT_HRI_MANIPULATION_TASK,CURRENT_OBJECT_TO_MANIPULATE,CURRENT_TASK_PERFORMED_BY,CURRENT_TASK_PERFORMED_FOR,performing_agent_rank,curr_resultant_candidate_points, CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS);

get_grasp_list_for_object(CURRENT_OBJECT_TO_MANIPULATE, grasps_for_object);
candidate_grasps_for_task=grasps_for_object;
CURRENT_ALL_GRASP_FOR_OBJECT=&grasps_for_object;
CURRENT_CANDIDATE_GRASP_FOR_TASK=&candidate_grasps_for_task;

char taken_by_hand[50];
#ifdef JIDO_EXISTS_FOR_MA
  if(JIDO_HAND_TYPE==1)
 {
   strcpy(taken_by_hand,"JIDO_GRIPPER");
 }
  if(JIDO_HAND_TYPE==2)
 {
   strcpy(taken_by_hand,"SAHandRight");
 }
#endif

#ifdef PR2_EXISTS_FOR_MA
strcpy(taken_by_hand,"PR2_GRIPPER");
#endif

int validate_task_result=0;

switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 case GIVE_OBJECT:
  {
 
  /*char from_hand[50];
  
  if(JIDO_HAND_TYPE==1)
 {
   strcpy(from_hand,"JIDO_GRIPPER");
 }
  if(JIDO_HAND_TYPE==2)
 {
   strcpy(from_hand,"SAHandRight");
 }
 */
  char* taken_from_hand="SAHandRight2";

  
  reduce_grasp_list_for_hand_over_task(grasps_for_object, CURRENT_OBJECT_TO_MANIPULATE, taken_by_hand, taken_from_hand, candidate_grasps_for_task );
  printf(" CURRENT_CANDIDATE_GRASP_FOR_TASK has been set\n");
CURRENT_CANDIDATE_GRASP_FOR_TASK=&candidate_grasps_for_task;
  
   get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
   CURRENT_CANDIDATE_PLACEMENT_LIST=&curr_placementList;
   res_traj.task_type=CURRENT_HRI_MANIPULATION_TASK;
  
  validate_task_result=JIDO_find_solution_to_take_new3(CURRENT_OBJECT_TO_MANIPULATE, CURRENT_HRI_MANIPULATION_TASK,  curr_task.for_agent, taken_by_hand, curr_task.by_agent, curr_resultant_candidate_points, candidate_grasps_for_task, curr_placementList, res_traj);//NOTE: Since it is interpretation of give from human to robot so the arguments have been swapped for take task
  }
 break;
 
 case SHOW_OBJECT:
 {
  get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
 } 
 break;

 case MAKE_OBJECT_ACCESSIBLE:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis
res_traj.task_type=CURRENT_HRI_MANIPULATION_TASK;
  
 validate_task_result=JIDO_find_solution_to_take_new3(CURRENT_OBJECT_TO_MANIPULATE, CURRENT_HRI_MANIPULATION_TASK, curr_task.for_agent, taken_by_hand, curr_task.by_agent, curr_resultant_candidate_points, candidate_grasps_for_task, curr_placementList, res_traj);//NOTE: Since it is interpretation of make accessible from human to robot so the arguments have been swapped for take task
  }
 break;
 
case HIDE_OBJECT:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis

  ////remove_hide_places_based_due_to_object(( char* ) CURRENT_OBJECT_TO_MANIPULATE);
  }
 break;
 
 }

 UPDATE_MIGHTABILITY_MAP_INFO=1;
  SHOW_MIGHTABILITY_MAP_INFO=1;   
 
  return validate_task_result;
 
}

int find_current_HRI_manip_task_solution(HRI_task_desc curr_task, traj_for_HRI_task &res_traj)
{

CURRENT_HRI_MANIPULATION_TASK=curr_task.task_type;
CURRENT_TASK_PERFORMED_BY=curr_task.by_agent;
CURRENT_TASK_PERFORMED_FOR=curr_task.for_agent;
strcpy(CURRENT_OBJECT_TO_MANIPULATE,curr_task.for_object.c_str());

printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s, TASK_PERFORMED_BY=%d, TASK_PERFORMED_FOR=%d\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE, CURRENT_TASK_PERFORMED_BY, CURRENT_TASK_PERFORMED_FOR);


#ifdef JIDO_EXISTS_FOR_MA
if(CURRENT_TASK_PERFORMED_BY==JIDO_MA||CURRENT_TASK_PERFORMED_FOR==JIDO_MA)
 {
 //case JIDO_MA:
  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[JIDO_MA]]->name);
// break;

 }
#endif

#ifdef PR2_EXISTS_FOR_MA
 if(CURRENT_TASK_PERFORMED_BY==PR2_MA||CURRENT_TASK_PERFORMED_FOR==PR2_MA)
 {
 //case PR2_MA:
  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[PR2_MA]]->name);
// break;

 }
#endif
 
candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
std::list<gpGrasp> grasps_for_object;
std::list<gpGrasp> candidate_grasps_for_task;
std::list<gpPlacement> curr_placementList;

int performing_agent_rank;
printf(" >> IS_PERFORMING_AGENT_MASTER=%d\n",IS_PERFORMING_AGENT_MASTER);
if(IS_PERFORMING_AGENT_MASTER==1)
performing_agent_rank=1;//Master
else
performing_agent_rank=0;//Slave

if(CURRENT_HRI_MANIPULATION_TASK==HIDE_OBJECT)
{
CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS=1;//to avoid trying to hide behind the target object itself
  
}

find_HRI_task_candidate_points(CURRENT_HRI_MANIPULATION_TASK,CURRENT_OBJECT_TO_MANIPULATE,CURRENT_TASK_PERFORMED_BY,CURRENT_TASK_PERFORMED_FOR,performing_agent_rank,curr_resultant_candidate_points, CONSIDER_OBJECT_DIMENSION_FOR_CANDIDATE_PTS);

if(curr_resultant_candidate_points->no_points<=0)
{
 printf(" HRI TASK Planner ERROR: No Candidate point to find the solution with current effort level \n");
 AT_LEAST_1_GRASP_LIFT_FOUND=1; //To avoid prematured termination without testing for next effort level
  MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);
 return 0;
}

//*** To check if the cell has already been tested in the previous call of the function, may be with lower effort level

for(int i=0;i<curr_resultant_candidate_points->no_points;i++)
{
   int cell_x=(curr_resultant_candidate_points->point[i].x- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  


	  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	    {
      
 
	     int cell_y=(curr_resultant_candidate_points->point[i].y- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
	      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		{
		 int cell_z=(curr_resultant_candidate_points->point[i].z- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
		  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		    {
		      ////curr_cell=
		      ////curr_cell->x=cell_x;
		      ////curr_cell->y=cell_y;
		      ////curr_cell->z=cell_z;
		     if(cells_tested_for_current_task[cell_x][cell_y][cell_z]==1)
	      {
		curr_resultant_candidate_points->status[i]=5;//already tested and failed for the same task
	      }
		    } 
		}
	    }

	

}
			
//**** END To check if the cell has already been tested 
			
get_grasp_list_for_object(CURRENT_OBJECT_TO_MANIPULATE, grasps_for_object);

candidate_grasps_for_task=grasps_for_object;

char by_hand[50];
#ifdef JIDO_EXISTS_FOR_MA
  if(JIDO_HAND_TYPE==1)
 {
   strcpy(by_hand,"JIDO_GRIPPER");
 }
  if(JIDO_HAND_TYPE==2)
 {
   strcpy(by_hand,"SAHandRight");
 }
#endif

#ifdef PR2_EXISTS_FOR_MA
strcpy(by_hand,"PR2_GRIPPER");
#endif

switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 case GIVE_OBJECT:
  {
 
  char* to_hand="SAHandRight2";

  reduce_grasp_list_for_hand_over_task(grasps_for_object, CURRENT_OBJECT_TO_MANIPULATE, by_hand, to_hand, candidate_grasps_for_task );

   get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
  
   
  }
 break;
 
 case SHOW_OBJECT:
 {
  get_placements_in_3D ( CURRENT_OBJECT_TO_MANIPULATE,  curr_placementList );
 } 
 break;

 case MAKE_OBJECT_ACCESSIBLE:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis

  }
 break;
 
case HIDE_OBJECT:
  {
  p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) CURRENT_OBJECT_TO_MANIPULATE );
  gpCompute_stable_placements (object, curr_placementList ); //this will give just based on the object and infinite plane the different ways an object can be put onto the place with different faces touching the plane. It will not give different orientation of the object along the vertical axis

  ////remove_hide_places_based_due_to_object(( char* ) CURRENT_OBJECT_TO_MANIPULATE);
  }
 break;
 
 }

 res_traj.task_type=CURRENT_HRI_MANIPULATION_TASK;
  
 int validate_task_result=0;
 int is_performing_agent_supported_by_planner=0;
 
 #ifdef JIDO_EXISTS_FOR_MA
 if(CURRENT_TASK_PERFORMED_BY==JIDO_MA)
 { 
   is_performing_agent_supported_by_planner=1;
 }
#endif
#ifdef PR2_EXISTS_FOR_MA
 if(CURRENT_TASK_PERFORMED_BY==PR2_MA)
 { 
   is_performing_agent_supported_by_planner=1;
 }
#endif

 if(is_performing_agent_supported_by_planner==1)
 {
  int filter_contact_polygon_inside_support=1;
 validate_task_result=robot_perform_task ( CURRENT_OBJECT_TO_MANIPULATE, CURRENT_HRI_MANIPULATION_TASK, CURRENT_TASK_PERFORMED_BY, by_hand, CURRENT_TASK_PERFORMED_FOR, curr_resultant_candidate_points, candidate_grasps_for_task, curr_placementList,  filter_contact_polygon_inside_support, res_traj);
 }
 else
 {
   printf(" >>>> ERROR: Performing the task by this agent is not supported at trajectory finding level. May be you just want to get the candidate places. For this use appropriate requests/functions\n");
 }
 
  ////////int validate_task_result=JIDO_find_solution_to_take ( CURRENT_OBJECT_TO_MANIPULATE, CURRENT_HRI_MANIPULATION_TASK, CURRENT_TASK_PERFORMED_FOR, curr_resultant_candidate_points, candidate_grasps_for_task, curr_placementList,  res_traj);


////JIDO_find_HRI_task_solution(CURRENT_HRI_MANIPULATION_TASK, HUMAN1_MA, CURRENT_OBJECT_TO_MANIPULATE);

/*
 switch(CURRENT_HRI_MANIPULATION_TASK)
 {
 #ifdef HRI_JIDO 
 case MAKE_OBJECT_ACCESSIBLE:
 JIDO_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case SHOW_OBJECT:
 JIDO_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case GIVE_OBJECT:
 JIDO_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case HIDE_OBJECT:
 JIDO_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 #endif
  #ifdef HRI_HRP2 
 case MAKE_OBJECT_ACCESSIBLE:
 HRP2_make_obj_accessible_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case SHOW_OBJECT:
 ////printf(">>> CURRENT_HRI_MANIPULATION_TASK=%d\n",CURRENT_HRI_MANIPULATION_TASK);
 HRP2_show_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case GIVE_OBJECT:
 HRP2_give_obj_to_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 case HIDE_OBJECT:
 HRP2_hide_obj_from_human ( CURRENT_OBJECT_TO_MANIPULATE );
 break;
 #endif
 }
*/

   MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);

  UPDATE_MIGHTABILITY_MAP_INFO=1;
  SHOW_MIGHTABILITY_MAP_INFO=1;   
  
  
 return validate_task_result;
}


#ifndef COMMENT_TMP
int JIDO_make_obj_accessible_to_humanOld ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
std::vector <p3d_traj*> trajs;
ManipulationData configs(manipulation->robot());
p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
int armID= 0;
p3d_matrix4 T0, T, tAtt;
gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
std::vector <double>  m_objStart, m_objGoto;
status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
			
  if(status==MANIPULATION_TASK_OK)
    //grasp= manipulation->getCurrentGrasp();
    grasp= *( manipulation->getManipulationData().getGrasp() );

  else { 
    printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
    return 1;
  }

  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];

  p3d_mat4Mult(grasp.frame, armHandProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

p3d_get_freeflyer_pose(object, T0);
point_co_ordi goal_pos, point_to_put;

int i1=0;
result= 0;
for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
 {
printf("checking for place %d \n",i1);

	goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.02;

	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_put.x;
	T[1][3]=point_to_put.y;
	T[2][3]=point_to_put.z;

        p3d_set_freeflyer_pose(object, T);
        g3d_draw_allwin_active();
        for(int i=0; i<50; ++i)
        {
          q = sampleRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
          if(q!=NULL) break;
        }
	if(q==NULL) 
        {
        printf("q==NULL after sampleRobotGraspPosWithoutBase()\n"); 
        continue;
        }
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,manipulation->robotStart(), q,  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", confs, smTrajs);

        printf(" After armPlanTask, result = %d \n",status);
	p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );

        if(status==MANIPULATION_TASK_OK)
        { 
           result= 1;
           break;
        }
//         else
//         { manipulation->cleanRoadmap(); }
 }//end for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
 
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}



int JIDO_make_obj_accessible_to_human_old_new ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_put;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
////    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_put.x;
	T[1][3]=point_to_put.y;
	T[2][3]=point_to_put.z;

        p3d_set_freeflyer_pose2(object, point_to_put.x, point_to_put.y, point_to_put.z, rx, ry, rz);
        
       double visibility_threshold=95.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = sampleRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after sampleRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_put.x;
        m_objGoto[1]= point_to_put.y;
        m_objGoto[2]= point_to_put.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}

int JIDO_show_obj_to_human ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_show_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_show_obj();

 printf ( " <<<<<< candidate_points_to_show.no_points = %d >>>>>>>>\n", candidate_points_to_show.no_points );
 if ( candidate_points_to_show.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_show;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
////    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_show.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_show.point[i1].x;
	goal_pos.y=candidate_points_to_show.point[i1].y;
	goal_pos.z=candidate_points_to_show.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_show.x=goal_pos.x;
	point_to_show.y=goal_pos.y;
	point_to_show.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_show.x;
	obj_tmp_pos[7]=point_to_show.y;
	obj_tmp_pos[8]=point_to_show.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_show.x;
	T[1][3]=point_to_show.y;
	T[2][3]=point_to_show.z;

        p3d_set_freeflyer_pose2(object, point_to_show.x, point_to_show.y, point_to_show.z, rx, ry, rz);
       double visibility_threshold=95.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = sampleRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after sampleRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_show.x;
        m_objGoto[1]= point_to_show.y;
        m_objGoto[2]= point_to_show.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);

        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}
#endif
void fct_draw_loop()
{
#if defined(WITH_XFORMS)
  g3d_screenshot((char *)"Move3D");
#endif
}

double get_wrist_head_alignment_angle(p3d_matrix4 wristFrame, p3d_matrix4 headFrame)
{
  double angle;
  p3d_vector3 wristPos, headPos, wristDir, dir;
  
 //For display purpose
  p3d_mat4Copy(headFrame,HEAD_FRAME);
  
  p3d_mat4ExtractTrans(wristFrame, wristPos);
  p3d_mat4ExtractColumnX(wristFrame, wristDir);
  ////p3d_mat4ExtractColumnZ(wristFrame, wristDir);
  p3d_mat4ExtractTrans(headFrame, headPos);

  p3d_vectSub(headPos, wristPos, dir);
  p3d_vectNormalize(dir, dir);

  angle= acos( p3d_vectDotProd(wristDir, dir) );
  return angle;
}

static int traj_play = TRUE;

int default_drawtraj_fct_without_XFORM(p3d_rob* robot, p3d_localpath* curLp)
{
  g3d_draw_allwin_active();
 //#if defined(WITH_XFORMS)
  //fl_check_forms();
 //#endif
  ////if(XFORM_update_func!=NULL)
  ////XFORM_update_func();
  return(traj_play);
}


//The direct function to reach the candidate point is called instead of reasoning on grasps as done in JIDO_find_solution_to_take and JIDO_find_solution_to_take_new2
int JIDO_find_solution_to_take_new(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{
  printf(" Inside JIDO_find_solution_to_take \n");
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
 
  std::list<gpPlacement> curr_placementList=placementList;
   ////get_placements_in_3D ( obj_to_manipulate,  curr_placementList );
   
   ////std::list<gpGrasp> grasps_for_object;
////get_grasp_list_for_object(obj_to_manipulate, grasps_for_object);
  ////std::list<gpGrasp> graspList=grasps_for_object;
  
  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
  

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }
    
   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   //int armID= 0;
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   obj_refConf= p3d_get_robot_config ( object );


////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   
   
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
   ////for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      ////grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      ////printf ( "grasp id= %d\n",igrasp->ID );
      
       for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d \n",i1 );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z;

			obj_tmp_pos[6]=goal_pos.x;
			obj_tmp_pos[7]=goal_pos.y;
			obj_tmp_pos[8]=goal_pos.z;
			p3d_set_and_update_this_robot_conf ( object, obj_tmp_pos );
			g3d_draw_allwin_active();
			g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[from_agent]->perspective->fov, object, &visibility, 0 );

                                

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
				  printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 
                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility  OK \n");
                             p3d_set_and_update_this_robot_conf ( object, obj_actual_pos ); 
			     g3d_draw_allwin_active();
                      int plac_ctr=0 ;
                       //// for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           
                           p3d_mat4Copy ( Tplacement0, T );

                           ////iplacement->position[0]= goal_pos.x;
                           ////iplacement->position[1]= goal_pos.y;
                           ////iplacement->position[2]= goal_pos.z;
                           ////iplacement->position[3]= 0;
                           ////iplacement->position[4]= 0;
                           ////iplacement->position[5]= P3D_HUGE;
			   
			   std::vector<double> objGoto;
			   objGoto.clear();
			   objGoto.push_back(goal_pos.x);
			   objGoto.push_back(goal_pos.y);
			   objGoto.push_back(goal_pos.z);
			   objGoto.push_back(P3D_HUGE);
			   objGoto.push_back(P3D_HUGE);
			   objGoto.push_back(P3D_HUGE);
			   
			     setMaxNumberOfTryForIK ( 3000 );
			   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
			   status= manipulation->armToFreePoint(armID, refConf, objGoto, NULL, take_trajs);
			   g3d_draw_allwin_active();
	          if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to take  \n");
		     
		      traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);
				  
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( manipulation->robot(), obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
                      return 1;
		  }
		  else
		  {
		    printf ( " No path found to take \n");
		  }
			   /*
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                           g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

      
      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      for ( int i=0; i<5; ++i )
      {
	
	gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
         manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);
         

         if(graspConf==NULL)
         {
          printf(" No IK Found to grasp\n");
          continue;
         }
         printf(" IK found to grasp \n");
	 gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          p3d_set_freeflyer_pose ( object, Tplacement0 );
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
	    double wrist_alignment_angle=get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos);
                           if(  wrist_alignment_angle > 150*DEGTORAD)
                            { 
                            printf(" Wrist Alignment %lf  is not good \n", wrist_alignment_angle*RADTODEG);
                            continue; 
                            }
                            printf(" Wrist Alignment %lf  is good \n", wrist_alignment_angle*RADTODEG);
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

		   #ifdef PR2_EXISTS_FOR_MA
                   if(by_agent==PR2_MA)
                    {
                   fixAllJointsWithoutArm(manipulation->robot(),0);
                    }
                  #endif
                  status= manipulation->armToFree ( armID, refConf, graspConf,TRUE, NULL,  take_trajs );
                  ////status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
//              
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		     
		      traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);
				  
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( manipulation->robot(), obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
                      return 1;
		  }
		  else
		  {
		    printf ( " No path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		  }
      }
      */
			}
		     }
   }
   return 0;
 
}

int JIDO_find_solution_to_take(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{
  printf(" Inside JIDO_find_solution_to_take \n");
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
 
  std::list<gpPlacement> curr_placementList=placementList;
   ////get_placements_in_3D ( obj_to_manipulate,  curr_placementList );
   
   ////std::list<gpGrasp> grasps_for_object;
////get_grasp_list_for_object(obj_to_manipulate, grasps_for_object);
  ////std::list<gpGrasp> graspList=grasps_for_object;
  
  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
  

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }
    
   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   //int armID= 0;
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   obj_refConf= p3d_get_robot_config ( object );


////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   
   
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      
       for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z;
printf(" Support name for placement =%s\n",envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
                      int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= goal_pos.x;
                           iplacement->position[1]= goal_pos.y;
                           iplacement->position[2]= goal_pos.z;
                           //iplacement->position[3]= 0;
                           //iplacement->position[4]= 0;
                           //iplacement->position[5]= P3D_HUGE;
			   
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                           g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

      
      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      for ( int i=0; i<5; ++i )
      {
	
          gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
         manipulation->checkConfigForCartesianMode(refConf, object);
          gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);
         

         if(graspConf==NULL)
         {
          printf(" No IK Found to grasp\n");
          continue;
         }
         printf(" IK found to grasp \n");
	 gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          p3d_set_freeflyer_pose ( object, Tplacement0 );
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
	    double wrist_alignment_angle=get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos);
                           if(  task==GIVE_OBJECT && wrist_alignment_angle > 150*DEGTORAD)
                            { 
                            printf(" Wrist Alignment %lf is not good \n", wrist_alignment_angle*RADTODEG);
                            continue; 
                            }
                            printf(" Wrist Alignment %lf  is good \n", wrist_alignment_angle*RADTODEG);
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

		   #ifdef PR2_EXISTS_FOR_MA
                   if(by_agent==PR2_MA)
                    {
                   fixAllJointsWithoutArm(manipulation->robot(),armID);
                    }
                  #endif
                  status= manipulation->armToFree ( armID, refConf, graspConf,TRUE, NULL,  take_trajs );
                  ////status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
//              
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		     
		      traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);
				  
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( manipulation->robot(), obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
				    
				    strcpy(SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ,envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
				    
				    printf(" SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ = %s\n",SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ);
				    
				    
                      return 1;
		  }
		  else
		  {
		    printf ( " No path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		  }
      }
			}
		     }
   }
   return 0;
 
}

//The order of loop is different from the JIDO_find_solution_to_take_new2
int JIDO_find_solution_to_take_new3(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{
  printf(" Inside JIDO_find_solution_to_take \n");
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
 
  std::list<gpPlacement> curr_placementList=placementList;
   ////get_placements_in_3D ( obj_to_manipulate,  curr_placementList );
   
   ////std::list<gpGrasp> grasps_for_object;
////get_grasp_list_for_object(obj_to_manipulate, grasps_for_object);
  ////std::list<gpGrasp> graspList=grasps_for_object;
  
  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
  

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }
    
   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   //int armID= 0;
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   obj_refConf= p3d_get_robot_config ( object );


////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   
   
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
   ////////for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   ////////{
    /////////  grasp_ctr++;
    ////////  p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
     //////// printf ( "grasp id= %d\n",igrasp->ID );
      
       for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
		       
		       printf ( "checking for place %d \n",i1 );
		       if(  task==GIVE_OBJECT)
		       {
		       if(curr_candidate_points->horizontal_surface_of[i1]>=0)
		        {
			 printf(" Point belongs to horizontal surface, avoid taking close to an horizontal surface\n");
			 continue;
		        }
		       } 
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z;
			
			
			     
//printf(" Support name for placement =%s\n",envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
                      int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           printf ( "checking for place %d, placement config %d\n",i1,plac_ctr );
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= goal_pos.x;
                           iplacement->position[1]= goal_pos.y;
                           iplacement->position[2]= goal_pos.z;
                           //iplacement->position[3]= 0;
                           //iplacement->position[4]= 0;
                           //iplacement->position[5]= P3D_HUGE;
			   
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
			   ////////g3d_draw_allwin_active();
			   int kcd_with_report=0;
                           int res = p3d_col_test_robot(object,kcd_with_report);
                           if(res>0)
			   {
			     printf(" Placement is in Collision \n");
			     continue;
			     
			   }
			   
			g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[from_agent]->perspective->fov, object, &visibility, 0 );

                                

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
				  printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 
                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility  OK \n");
                             ////////p3d_set_and_update_this_robot_conf ( object, obj_actual_pos ); 
			     
			     
                           //////////g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
			   grasp_ctr=0;
			   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
                            {
                            grasp_ctr++;
                           p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                            printf ( "grasp id= %d\n",igrasp->ID );
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

      
      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      for ( int i=0; i<1; ++i )
      {
	
	gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
         manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);
         

         if(graspConf==NULL)
         {
          printf(" No IK Found to grasp\n");
          continue;
         }
         printf(" IK found to grasp \n");
	 gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          p3d_set_freeflyer_pose ( object, Tplacement0 );
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            ////////g3d_draw_allwin_active();

            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
	    double wrist_alignment_angle=get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos);
                           if(  task==GIVE_OBJECT && wrist_alignment_angle > 45*DEGTORAD)
                            { 
                            printf(" Wrist Alignment %lf is not good \n", wrist_alignment_angle*RADTODEG);
                            continue; 
                            }
                            printf(" Wrist Alignment %lf  is good \n", wrist_alignment_angle*RADTODEG);
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

		   #ifdef PR2_EXISTS_FOR_MA
                   if(by_agent==PR2_MA)
                    {
                   fixAllJointsWithoutArm(manipulation->robot(),armID);
                    }
                  #endif
                  status= manipulation->armToFree ( armID, refConf, graspConf,TRUE, NULL,  take_trajs );
                  ////status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
//              
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		      elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Proactive Planner Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
				   
		      traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);
				  
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   ////elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                  //// printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( object, obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
				    
				    if(  task==MAKE_OBJECT_ACCESSIBLE )
                                    { 
				    strcpy(SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ,envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
				    
				    printf(" SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ = %s\n",SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ);
				    
			            }    
                      return 1;
		  }
		  else
		  {
		    printf ( " No path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		  }
      }
      }
			}
		     }
   ////////}
   return 0;
 
}


//The difference from the JIDO_find_solution_to_take is: Collision and visibility tests at placements have been added
int JIDO_find_solution_to_take_new2(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{
  printf(" Inside JIDO_find_solution_to_take \n");
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
 
  std::list<gpPlacement> curr_placementList=placementList;
   ////get_placements_in_3D ( obj_to_manipulate,  curr_placementList );
   
   ////std::list<gpGrasp> grasps_for_object;
////get_grasp_list_for_object(obj_to_manipulate, grasps_for_object);
  ////std::list<gpGrasp> graspList=grasps_for_object;
  
  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
  

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }
    
   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   //int armID= 0;
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   obj_refConf= p3d_get_robot_config ( object );


////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   
   
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      
       for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z;
			
			
			     
//printf(" Support name for placement =%s\n",envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
                      int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           printf ( "checking for place %d, placement config %d, and grasp id= %d\n",i1,plac_ctr, igrasp->ID );
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= goal_pos.x;
                           iplacement->position[1]= goal_pos.y;
                           iplacement->position[2]= goal_pos.z;
                           //iplacement->position[3]= 0;
                           //iplacement->position[4]= 0;
                           //iplacement->position[5]= P3D_HUGE;
			   
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
			   g3d_draw_allwin_active();
			   int kcd_with_report=0;
                           int res = p3d_col_test_robot(object,kcd_with_report);
                           if(res>0)
			   {
			     printf(" Placement is in Collision \n");
			     continue;
			     
			   }
			   
			g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[from_agent]->perspective->fov, object, &visibility, 0 );

                                

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
				  printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 
                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );
                                printf(" Visibility  OK \n");
                             p3d_set_and_update_this_robot_conf ( object, obj_actual_pos ); 
			     
			     
                           g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

      
      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      for ( int i=0; i<1; ++i )
      {
	
	gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
         manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);
         

         if(graspConf==NULL)
         {
          printf(" No IK Found to grasp\n");
          continue;
         }
         printf(" IK found to grasp \n");
	 gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          p3d_set_freeflyer_pose ( object, Tplacement0 );
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
	    double wrist_alignment_angle=get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[from_agent]->perspective->camjoint->abs_pos);
                           if(  task==GIVE_OBJECT && wrist_alignment_angle > 45*DEGTORAD)
                            { 
                            printf(" Wrist Alignment %lf is not good \n", wrist_alignment_angle*RADTODEG);
                            continue; 
                            }
                            printf(" Wrist Alignment %lf  is good \n", wrist_alignment_angle*RADTODEG);
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

		   #ifdef PR2_EXISTS_FOR_MA
                   if(by_agent==PR2_MA)
                    {
                   fixAllJointsWithoutArm(manipulation->robot(),armID);
                    }
                  #endif
                  status= manipulation->armToFree ( armID, refConf, graspConf,TRUE, NULL,  take_trajs );
                  ////status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
//              
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		      elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
				   
		      traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);
				  
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   ////elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                  //// printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config ( manipulation->robot(), obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
				    
				    if(  task==MAKE_OBJECT_ACCESSIBLE )
                                    { 
				    strcpy(SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ,envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]]->name);
				    
				    printf(" SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ = %s\n",SUPPORT_NAME_FOR_HUMAN_TO_PUT_OBJ);
				    
			            }    
                      return 1;
		  }
		  else
		  {
		    printf ( " No path found to take for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
		  }
      }
			}
		     }
   }
   return 0;
 
}


int JIDO_perform_task ( char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs)
{

  
  //FOR PR2 DO fixAllJointsWithoutArm(m_manipulation->robot(),0);
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif

  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
   std::list<gpPlacement> curr_placementList=placementList;

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ////////ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [armID];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [armID].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   ////refConf= p3d_get_robot_config ( envPt_MM->robot[indices_of_MA_agents[by_agent]] );
   obj_refConf= p3d_get_robot_config ( object );

   //// ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );

    
    
////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   
   manipulation->setSafetyDistanceValue ( 0.0 );
   printf(" >>>> Warning: For planning HRI task: Actual safety distance valus was %lf, which has been reset to %lf \n",orig_safety_dist, manipulation->getSafetyDistanceValue());
   
   p3d_rob* support_to_place=NULL;
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif

int support_index=-1;
			       is_object_laying_on_a_support(obj_index, support_index);
				 
                               //////////p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "IKEA_SHELF" );
			       p3d_rob* support= NULL;
			       if(support_index>=0)
			       {
			       support=envPt_MM->robot[support_index];
			       printf(" >> Automatically detected support for object by HRI task planner is: %s\n",support->name);
			       }
			       else
			       {
				 printf(" >>>**>> HRI TASK PLANNER WARNING : The object to grasp has not been found to be on any support. IS THERE ANY PERCEPTION PROBLEM? Any way I will plan assuming the object is hanging in the air \n");
			       }
			       
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );
      g3d_draw_allwin_active();


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      for ( int i=0; i<1; ++i )
      {
	ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
	 manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {
	    //gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          printf(" No IK Found to grasp for test %d\n", i);
	  g3d_draw_allwin_active();
          continue;
         }
         
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
	   gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
           

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );

            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                    manipulation->cleanRoadmap();
		  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
		  #ifdef PR2_EXISTS_FOR_MA
		    if(by_agent==PR2_MA)
		    {
		    fixAllJointsWithoutArm(manipulation->robot(),armID);
		    }
		  #endif
		   g3d_draw_allwin_active();
		   
		   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
              
	       
                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
                  
//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " path found to pick for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );

                    

/* Following is commented because no need to concatanate trajs segments and compute soft motion at this stage
                     manipulation->concatTrajectories ( take_trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );
*/
                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        if(task==MAKE_OBJECT_ACCESSIBLE||task==HIDE_OBJECT)
                        {

                        std::list<gpPlacement> general_stable_configuration_list; 
                        std::list<gpPlacement> tmp_placement_list; 
                        curr_placementList.clear();
                        get_placements_at_position(  object, goal_pos, placementList, 10, curr_placementList );
        //printf("stable_placements_list.size()=%d, curr_placementListOut.size = %d \n", stable_placements_list.size(), curr_placementListOut.size());
                        printf("curr_placementList.size = %d \n",  curr_placementList.size());
                        if(curr_placementList.size()==0)
                          {
                         continue;
        ////return 0;
                          }
                         tmp_placement_list.clear();
                         tmp_placement_list= curr_placementList;
			 support_to_place = envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]];
			 
	                 gpPlacement_on_support_filter ( object, support_to_place,tmp_placement_list,curr_placementList);                          
                          printf(" Support name for placement =%s\n",support_to_place->name);
                          printf("After gpPlacement_on_support_filter(), curr_placementList.size = %d \n",  curr_placementList.size());
                          if(curr_placementList.size()==0)
                          {
                          
                         continue;
        ////return 0;
                          }
                         }

                        if(task!=HIDE_OBJECT)
                        {
                         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[indices_of_MA_agents[for_agent]], curr_placementList );
                        }

                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if(task!=HIDE_OBJECT)
                           {
                           if ( iplacement->stability<=0 ) //As the placement list is sorted based on the visibility range. NOTE: This stability is based on the visibility range of the front
                            {
                              break;
                            }
                           }

                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;

                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                           g3d_draw_allwin_active();
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

                           //// p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );

                           if(task==GIVE_OBJECT||task==SHOW_OBJECT)
                           {
                           p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos) > 120*DEGTORAD)
                            { 
                            printf(" Wrist Alignment is not good \n");
                            continue; 
                            }
                           }
                           
                           #ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif
                            ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                            gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);

                           for ( int j=0; j<1; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                            /*
			      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ////ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
			    */
			    
			      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
	 manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         
	 
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              ////p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,Tfinal_placement);
                              
                              placeConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {
                               printf(" No IK Found to place\n");
			       g3d_draw_allwin_active();
                               continue;
                              }

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                               printf(" Place config IK found\n");
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                ////if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                if(task==SHOW_OBJECT||task==GIVE_OBJECT) //Need to set the robot at placement config to test the visibility of object while in robot's hand
                                {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                }
                                else
                                {
                                if(task==HIDE_OBJECT||task==MAKE_OBJECT_ACCESSIBLE) //Need to set the robot at rest config to test the visibility of object while in robot's hand
                                 {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                 }
                                }
                                g3d_draw_allwin_active();

                               
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 0 );

                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
				////g3d_draw_allwin_active();
                                continue;
                                } 
                                
                                
                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                
                                
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
//                                int support_index=-1;
// 			       is_object_laying_on_a_support(obj_index, support_index);
// 				 
//                                //////////p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "IKEA_SHELF" );
// 			       p3d_rob* support= envPt_MM->robot[support_index];
// 			       printf(" >> Automatically detected support for object by HRI task planner is: %s\n",support->name);
			       
                               p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.03;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               
			         ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
	 manipulation->checkConfigForCartesianMode(graspConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
	 
	   manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
				  
                               liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
//                                liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, p3d_mat4IDENTITY,  0, 0, armID, false );
                                 g3d_draw_allwin_active();
                               if ( liftConf==NULL )
                               {
                                printf(" Fail to find liftConf\n");
                                continue;
                               }
                               printf(" LiftConf found\n");
                              
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);

                                 //This test is to avoid the situations where robot should not make a big loop but it is doing so. Like between grasp and lift configs TODO: Do such tests between approach and open hand grasp configs also
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1)
                                 {
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   printf(" Fail optimizeRedundentJointConfigDist()\n");
                                   continue;
                                  }
                                
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               ////if(liftConf!=NULL)
                               ////{
                                  printf(" %d th IK Found to lift \n",j);
                                 //// ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }                           
		  #ifdef PR2_EXISTS_FOR_MA
		    if(by_agent==PR2_MA)
		    {
		    fixAllJointsWithoutArm(manipulation->robot(),armID);
		    }
		  #endif
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);
                                 
                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                 
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
/*
                                  manipulation->robot()->tcur=place_trajs[0];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);

                                  manipulation->robot()->tcur=place_trajs[1];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);
*/
                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_GRASP;
                                  traj_sub_task.traj=take_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=GRASP;
                                  traj_sub_task.traj=take_trajs[2];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=LIFT_OBJECT;
                                  traj_sub_task.traj=place_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=CARRY_OBJECT;
                                  traj_sub_task.traj=place_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);

                                  if(task==HIDE_OBJECT||task==MAKE_OBJECT_ACCESSIBLE) //Need to place and release the object
                                  {
                                  p3d_set_freeflyer_pose ( object, Tfinal_placement );
                                  //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                                  //printf(" >>> Obj final pos (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);

                                  manipulation->getManipulationData().clear();
                                  manipulation->getManipulationData().setAttachFrame(tAtt);
                                  manipulation->getManipulationData().setGraspConfig(placeConf);
                                  manipulation->getManipulationData().setGrasp(&(*igrasp));

                                  p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );

                                  g3d_draw_allwin_active();

                                  MANIPULATION_TASK_MESSAGE release_obj_status = manipulation->armEscapeObject(armID, placeConf,  object, release_obj_trajs);
                                  printf(" release_obj_status= %d \n",release_obj_status);
                                  ////return 0;
                                  if(release_obj_status==MANIPULATION_TASK_OK)
                                   {
                                  printf("release_obj_trajs.size()=%d\n",release_obj_trajs.size());
                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=RELEASE_OBJECT;
                                  traj_sub_task.traj=release_obj_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task); 

                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=RETREAT_HAND;
                                  traj_sub_task.traj=release_obj_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task); 
                                   }
                                  } 
/* Below is commented because no need to concatinate traj now and no need to generate soft motion
 
                                  manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
*/
				
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config (object, obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
                                   return 1;
                                  }
                                  else 
				  {
				    printf ( " no path found for place \n");
				    ////return 1;
				    
				  }
                               ////}

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }
      manipulation->robot()->isCarryingObject = FALSE;
     
			
   result= 0;
   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( object, obj_refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   //////////p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}

grasp_lift_info curr_grasp_lift_info;

std::map<int, grasp_lift_info> grasp_id_info_map;

int take_lift_traj_found_for_grasp[200];//0 : not tested, 1 : tested and found valid traj, -1: tested and failed to find a valid traj to grasp and lift


int init_take_lift_traj_info()
{
  AT_LEAST_1_GRASP_LIFT_FOUND=0;
  
for(int g_ctr=0; g_ctr<200;g_ctr++)
 {
  take_lift_traj_found_for_grasp[g_ctr]=0;
 }

 std::map<int, grasp_lift_info>::iterator it; 

 for(it=grasp_id_info_map.begin();it!=grasp_id_info_map.end();it++)
 {
   printf(" Freeing for grasp id = %d\n ", (*it).first);
   
 MY_FREE((*it).second.graspConf,double,manipulation->robot()->nb_dof);
 MY_FREE((*it).second.liftConf,double,manipulation->robot()->nb_dof);
 
 //for(unsigned int i=0;i<(*it).second.take_trajs.size();i++)
 //{
   //printf(" destroying traj %d \n",i);
   //TODO: See why the following line is giving seg fault when freed before the second task
   ////////p3d_destroy_traj(manipulation->robot(),(*it).second.take_trajs.at(i));
   
 //}
 
 //TODO: Write code to not to destroy the take traj for which the solution has been found, so that it could be stored for execution in the case of multiple tasks. So, for the time being none of the trajectories are being destroyed. This may cause memory problems.
 p3d_destroy_traj(manipulation->robot(),(*it).second.reach_traj);
 p3d_destroy_traj(manipulation->robot(),(*it).second.pre_grasp_traj);
 p3d_destroy_traj(manipulation->robot(),(*it).second.grasp_traj);
		  
 ////(*it).second.take_trajs.clear();
 
 }
 
 grasp_id_info_map.clear();
 
}

int MAINTAIN_OBJ_FRONT_VIS_CONSTRAINT=0;


int robot_perform_task ( char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, int filter_contact_polygon_inside_support, traj_for_HRI_task &res_trajs)
{
p3d_rob* hand_rob= ( p3d_rob* ) p3d_get_robot_by_name ( by_hand );
  
  //FOR PR2 DO fixAllJointsWithoutArm(m_manipulation->robot(),0);
  int armID= 0;
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif

  char curr_robot_hand_name[50];
  strcpy(curr_robot_hand_name,by_hand);
  /*
  if(JIDO_HAND_TYPE==1)
   {
   strcpy(curr_robot_hand_name,"JIDO_GRIPPER");
   }
   if(JIDO_HAND_TYPE==2)
   {
   strcpy(curr_robot_hand_name,"SAHandRight");
   }
   */
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
  
   std::list<gpPlacement> curr_placementList=placementList;

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " >>**>> HRI TASK PLANNER ERROR : No Candidate points for current effort level\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=HRI_TASK_PLAN_IN_CARTESIAN;
if(PLAN_IN_CARTESIAN == 1) 
    {
      manipulation->setArmCartesian(armID,true);
    /*for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) 
     {
     
     }*/
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ////////ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [armID];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   
   p3d_matrix4 Tplacement0, T;
   p3d_matrix4 Tfinal_placement;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [armID].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> take_trajs;
   std::vector <p3d_traj*> place_trajs;
   std::vector <p3d_traj*> release_obj_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

   traj_for_HRI_sub_task traj_sub_task;

   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL, obj_refConf=NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );
   ////refConf= p3d_get_robot_config ( envPt_MM->robot[indices_of_MA_agents[by_agent]] );
   obj_refConf= p3d_get_robot_config ( object );

   //// ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );

     p3d_matrix4 Tplacement;       
    
////   ( *manipulation->robot()->armManipulationData ) [armID]. ( handFree ) ;

   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   
   manipulation->setSafetyDistanceValue ( 0.00 );
   printf(" >>>> Warning: For planning HRI task: Actual safety distance valus was %lf, which has been reset to %lf \n",orig_safety_dist, manipulation->getSafetyDistanceValue());
   
   p3d_rob* support_to_place=NULL;
 
   gpGrasp_context_collision_filter(graspList, ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ), object, handProp);
   p3d_set_freeflyer_pose2( ( p3d_rob* ) p3d_get_robot_by_name ( curr_robot_hand_name ),5,5,5,0,0,0);
#ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
    
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif

int support_index=-1;
			       is_object_laying_on_a_support(obj_index, support_index);
				 
                               //////////p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "IKEA_SHELF" );
			       p3d_rob* support= NULL;
			       if(support_index>=0)
			       {
			       support=envPt_MM->robot[support_index];
			       printf(" >> Automatically detected support for object by HRI task planner is: %s\n",support->name);
			       }
			       else
			       {
				 printf(" >>>**>> HRI TASK PLANNER WARNING : The object to grasp has not been found to be on any support. IS THERE ANY PERCEPTION PROBLEM? Any way I will plan assuming the object is hanging in the air \n");
			       }

AT_LEAST_1_GRASP_LIFT_FOUND=1;


 for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
  {
                        printf ( ">>>> checking for place %d \n",i1);
			
			if(curr_candidate_points->status[i1]==5)//Already decleared as failed
			{
			  printf(" current candidate point %d has already been decleared as failed \n",i1);
			  
			  continue;
			}
			
	  int cell_x=(curr_candidate_points->point[i1].x- grid_around_HRP2.GRID_SET->realx)/grid_around_HRP2.GRID_SET->pace;  
       
	  

	  if(cell_x>=0&&cell_x<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx)
	    {
      
 
	      int cell_y=(curr_candidate_points->point[i1].y- grid_around_HRP2.GRID_SET->realy)/grid_around_HRP2.GRID_SET->pace; 
      
	      if(cell_y>=0&&cell_y<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny)
		{
		  int cell_z=(curr_candidate_points->point[i1].z- grid_around_HRP2.GRID_SET->realz)/grid_around_HRP2.GRID_SET->pace; 
 
		  if(cell_z>=0&&cell_z<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nz)
		    {
		      ////curr_cell=
		      ////curr_cell->x=cell_x;
		      ////curr_cell->y=cell_y;
		      ////curr_cell->z=cell_z;
		  cells_tested_for_current_task[cell_x][cell_y][cell_z]=1;
		     
		    } 
		}
	    }
			
			if(i1>=1&&AT_LEAST_1_GRASP_LIFT_FOUND==0)
			{
			  printf("AKP ERROR: In the first pass no grasp lift found so not testing for more points \n");
			  
			  break;
			}
			  
			
			
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

			point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

			/*p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);
g3d_draw_allwin_active();
p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z+0.05, rx, ry, rz);
g3d_draw_allwin_active();*/
                        if(task==MAKE_OBJECT_ACCESSIBLE||task==HIDE_OBJECT)
                        {

                        std::list<gpPlacement> general_stable_configuration_list; 
                        std::list<gpPlacement> tmp_placement_list; 
                        curr_placementList.clear();
                        get_placements_at_position(  object, goal_pos, placementList, 10, curr_placementList );
        //printf("stable_placements_list.size()=%d, curr_placementListOut.size = %d \n", stable_placements_list.size(), curr_placementListOut.size());
                        printf("curr_placementList.size = %d \n",  curr_placementList.size());
                        if(curr_placementList.size()==0)
                          {
			    printf(" So trying for next point \n");
                         continue;
        ////return 0;
                          }
                          
                          //// show_all_how_to_placements_in_3D(point_to_give,0,10,0,&curr_placementList);
			    CURRENT_CANDIDATE_PLACEMENT_LIST=&curr_placementList;
			    TO_SHOW_PLACEMENT_POINT=goal_pos;
			    
			    SHOW_HOW_TO_PLACE_AT=1;
			    g3d_draw_allwin_active();
                            SHOW_HOW_TO_PLACE_AT=0;
			    g3d_draw_allwin_active();
			    
                         tmp_placement_list.clear();
                         tmp_placement_list= curr_placementList;
			 support_to_place = envPt_MM->robot[curr_candidate_points->horizontal_surface_of[i1]];
			 
			  printf(" Support name for placement =%s\n",support_to_place->name);
			  
			  if(filter_contact_polygon_inside_support==1)// To further filter that the contact polygon of the placement is actually inside the support polygon
			  {
	                 gpPlacement_on_support_filter ( object, support_to_place,tmp_placement_list,curr_placementList); 
			  printf("After gpPlacement_on_support_filter(), curr_placementList.size = %d \n",  curr_placementList.size());
			  }
			  
                         
                          if(curr_placementList.size()==0)
                          {
                          printf(" ** So trying for next point \n");
                         continue;
        ////return 0;
                          }
                         }
                         
                          
    printf(" Found a valid stable placement list for the current point\n");                     
            grasp_ctr=0;            
   	    CURRENT_CANDIDATE_PLACEMENT_LIST=&curr_placementList;
			    TO_SHOW_PLACEMENT_POINT=goal_pos;
			    
			    SHOW_HOW_TO_PLACE_AT=1;
			    g3d_draw_allwin_active();
                            SHOW_HOW_TO_PLACE_AT=0;
			    g3d_draw_allwin_active();
			    
   AT_LEAST_1_GRASP_LIFT_FOUND=0;
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      
      printf ( "for grasp id= %d\n",igrasp->ID );
      
      if(take_lift_traj_found_for_grasp[grasp_ctr]==-1)//Already tested and failed
      {
	printf(" Already tested and failed \n");
	grasp_ctr++;
	continue; 
      }
      
       //ADDED for precaution
      manipulation->robot()->isCarryingObject =FALSE;
      
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      p3d_set_freeflyer_pose ( object, Tplacement0 );
      g3d_draw_allwin_active();
      p3d_get_freeflyer_pose ( object, Tobject );
      
      if(take_lift_traj_found_for_grasp[grasp_ctr]==0)
      {
      
      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      // it is reactivated in armPickGoTo, so we need to deactivate again:
     
      //ADDED for precaution
      gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
      
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
      ////for ( int i=0; i<1; ++i )
      ////{
	//////////ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
        ////////// p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
	 manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {
	    //gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
          printf(" No IK Found to grasp \n");
	  take_lift_traj_found_for_grasp[grasp_ctr]=-1;
	  g3d_draw_allwin_active();
	  grasp_ctr++;
          continue;
         }
         
         printf ( "graspConf= %p\n", graspConf );
         ////if ( graspConf!=NULL )
         ////{
	   gpActivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
           //////// p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
           

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );

            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
	    printf(" fail to get approachConf\n");
	    take_lift_traj_found_for_grasp[grasp_ctr]=-1;
	    grasp_ctr++;
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
		  printf(" Fail to get open config\n");
		  take_lift_traj_found_for_grasp[grasp_ctr]=-1;
		  grasp_ctr++;
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                    manipulation->cleanRoadmap();
		  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
		  #ifdef PR2_EXISTS_FOR_MA
		    if(by_agent==PR2_MA)
		    {
		    fixAllJointsWithoutArm(manipulation->robot(),armID);
		    }
		  #endif
		   g3d_draw_allwin_active();
		   
		   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
              
	          take_trajs.clear();
                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, take_trajs );
                  
//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status!=MANIPULATION_TASK_OK )
                  {
		    printf(" Fail to find path to grasp the object for grasp_ctr=%d,\n",grasp_ctr );
		    ManipulationUtils::printManipulationMessage ( status );
		     p3d_destroy_config( manipulation->robot(), graspConf );
		      take_lift_traj_found_for_grasp[grasp_ctr]=-1;
		      grasp_ctr++;
		    continue;
		  }
		
		printf ( " ****** path found to grasp the object for grasp_ctr=%d,\n",grasp_ctr );
		 g3d_draw_allwin_active();
		 
		p3d_set_freeflyer_pose ( object, Tplacement0 );
		      
                manipulation->getManipulationData().setAttachFrame(tAtt);
                deactivateCcCntrts(manipulation->robot(), armID);
                p3d_mat4Copy(Tplacement0, Tplacement);
                Tplacement[2][3]+= 0.15;
                p3d_set_freeflyer_pose ( object, Tplacement );
                                g3d_draw_allwin_active();
			       ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                            gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
			   
			       
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
	 manipulation->checkConfigForCartesianMode(graspConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
	 
	   ////manipulation->robot()->isCarryingObject = TRUE;
                                  ////(*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
		
				  g3d_draw_allwin_active();

                               liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  false, false, armID, false );
//                                liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, p3d_mat4IDENTITY,  0, 0, armID, false );
                                 g3d_draw_allwin_active();
                               if ( liftConf==NULL )
                               {
                                printf(" Fail to find liftConf\n");
				 p3d_destroy_config( manipulation->robot(), graspConf );
				take_lift_traj_found_for_grasp[grasp_ctr]=-1;
				grasp_ctr++;
                                continue;
                               }
                               
                               AT_LEAST_1_GRASP_LIFT_FOUND=1;
                               printf(" LiftConf found\n");
			       
			        manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
			       
			       //This test is to avoid the situations where robot should not make a big loop but it is doing so. Like between grasp and lift configs TODO: Do such tests between approach and open hand grasp configs also
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1)
                                 {
				   p3d_destroy_config( manipulation->robot(), graspConf );
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   ////liftConf = NULL;
                                   printf(" Fail optimizeRedundentJointConfigDist()\n");
				   take_lift_traj_found_for_grasp[grasp_ctr]=-1;
				   grasp_ctr++;
                                   continue;
                                  }
                                  
                                   printf("optimization for LiftConf OK\n");
				   
				   //Save this grasp traj and lift config for next point to avoid recalculating 
				   curr_grasp_lift_info.graspConf=MY_ALLOC( double,manipulation->robot()->nb_dof);
				   p3d_copy_config_into ( manipulation->robot(),graspConf, &curr_grasp_lift_info.graspConf );
				    
				    curr_grasp_lift_info.liftConf=MY_ALLOC( double,manipulation->robot()->nb_dof);
				     p3d_copy_config_into ( manipulation->robot(),liftConf, &curr_grasp_lift_info.liftConf );
				     
				     curr_grasp_lift_info.reach_traj=take_trajs[0];
				     curr_grasp_lift_info.pre_grasp_traj=take_trajs[1];
				     curr_grasp_lift_info.grasp_traj=take_trajs[2];
				     
				     grasp_id_info_map[igrasp->ID]=curr_grasp_lift_info;
				     
			        take_lift_traj_found_for_grasp[grasp_ctr]=1;
				manipulation->robot()->isCarryingObject = FALSE;
				////grasp_ctr++;
			       
      }       
      else
      {
	AT_LEAST_1_GRASP_LIFT_FOUND=1;
      if(take_lift_traj_found_for_grasp[grasp_ctr]==1)//Already tested and found valid take traj and lift configs
       {
	graspConf=grasp_id_info_map.find(igrasp->ID)->second.graspConf;
	liftConf=grasp_id_info_map.find(igrasp->ID)->second.liftConf;
	//take_trajs=grasp_id_info_map.find(igrasp->ID)->second.take_trajs;
	take_trajs.clear();
	take_trajs.push_back(grasp_id_info_map.find(igrasp->ID)->second.reach_traj);
	take_trajs.push_back(grasp_id_info_map.find(igrasp->ID)->second.pre_grasp_traj);
	take_trajs.push_back(grasp_id_info_map.find(igrasp->ID)->second.grasp_traj);
       }
       else
       {
	 printf(" >>**>> HRI Task Planner WARNING : The HRI Task planner is reaching here, which is NOT GOOD, Debug it\n");
       }
      }
      grasp_ctr++;
      
/* Following is commented because no need to concatanate trajs segments and compute soft motion at this stage
                     manipulation->concatTrajectories ( take_trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );
*/
                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     //////////for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     ////{
                       ////////// printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        //////////goal_pos.x=curr_candidate_points->point[i1].x;
                       ////////// goal_pos.y=curr_candidate_points->point[i1].y;
                        //////////goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        
                        if(MAINTAIN_OBJ_FRONT_VIS_CONSTRAINT==1)
			{
                        if(task!=HIDE_OBJECT)
                         {
                         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[indices_of_MA_agents[for_agent]], curr_placementList );
                         }
			}
			
                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
			  printf(" >> Testing for placement configuration %d \n", plac_ctr);
                           plac_ctr++;
                           ////iter->draw(0.05);
			   if(MAINTAIN_OBJ_FRONT_VIS_CONSTRAINT==1)
			   {
                           if(task!=HIDE_OBJECT)
                            {
                           if ( iplacement->stability<=0 ) //As the placement list is sorted based on the visibility range. NOTE: This stability is based on the visibility range of the front
                             { 
			      printf(" **** Break testing for next placement configurations for this point as the visibility of the front constraints will not be maintained \n");
                              break;
                             }
                            }
			   }
			   
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;

                           //////////p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
			   
			     int kcd_with_report=0;
                           int res = p3d_col_test_robot(object,kcd_with_report);
                           if(res>0)
			   {
			     printf(" Placement is in Collision \n");
			     continue;
			     
			   }
                           
			   g3d_draw_allwin_active();
			   g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 1 );

                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
                                printf(" Visibility NOT OK \n");
                                //////////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
				////g3d_draw_allwin_active();
				
                                continue;
                                } 
                                 printf(" The object's visibility %lf is good\n",visibility); 
                                
			 
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

                           //// p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
                           
                           if(task==GIVE_OBJECT||task==SHOW_OBJECT)
                           {
                           p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
			   
			   double wrist_alignment=get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos);
			   printf(" Wrist Alignment = %lf \n",wrist_alignment);
                           if( wrist_alignment > 120*DEGTORAD)
                            { 
                            printf(" Wrist Alignment is not good \n");
                            continue; 
                            }
                            else
			    {
			       printf(" Wrist Alignment is good \n ");
			    }
                           }
                           
                          
                           #ifdef PR2_EXISTS_FOR_MA
  if(by_agent==PR2_MA)
  {
   fixAllJointsWithoutArm(manipulation->robot(),armID);
  }
#endif

manipulation->robot()->isCarryingObject = FALSE;
                                 
                            ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                            gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);

                           ////////////for ( int j=0; j<1; ++j )
                           ////{
                              deactivateCcCntrts(manipulation->robot(), armID);
                            /*
			      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ////ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
			    */
			    
			      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
	 manipulation->checkConfigForCartesianMode(refConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
         
	 
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              ////p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,Tfinal_placement);
			      
			      //*****For visualization of the grasp to test
			       gpSet_robot_hand_grasp_configuration(hand_rob, object, *igrasp);
//   gpSet_robot_hand_grasp_open_configuration(HAND_ROBOT, OBJECT, GRASP);
                               p3d_copy_config_into(hand_rob, p3d_get_robot_config(hand_rob), &hand_rob->ROBOT_POS);
			       
			        g3d_draw_allwin_active();
			       p3d_set_freeflyer_pose2(hand_rob,0,0,0,0,0,0);
			       
                               g3d_draw_allwin_active();

                              placeConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );

			       //*** END For visualization of the grasp to teste
			       
 
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {
				
			    //  gpSet_robot_hand_grasp_configuration(hand_rob, object, *igrasp);
//   gpSet_robot_hand_grasp_open_configuration(HAND_ROBOT, OBJECT, GRASP);
                             //  p3d_copy_config_into(hand_rob, p3d_get_robot_config(hand_rob), &hand_rob->ROBOT_POS);

                               printf(" No IK Found to place\n");
			      // g3d_draw_allwin_active();
			      // p3d_set_freeflyer_pose2(hand_rob,0,0,0,0,0,0);
                               continue;
                              }

                              
                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                               printf(" Place config IK found\n");
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                ////if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                if(task==SHOW_OBJECT||task==GIVE_OBJECT) //Need to set the robot at placement config to test the visibility of object while in robot's hand
                                {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                }
                                else
                                {
                                if(task==HIDE_OBJECT||task==MAKE_OBJECT_ACCESSIBLE) 
                                 {
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                 }
                                }
                                g3d_draw_allwin_active();

                               
                                
                                
                                
                                ////g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 1 );
                                
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
//                                int support_index=-1;
// 			       is_object_laying_on_a_support(obj_index, support_index);
// 				 
//                                //////////p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "IKEA_SHELF" );
// 			       p3d_rob* support= envPt_MM->robot[support_index];
// 			       printf(" >> Automatically detected support for object by HRI task planner is: %s\n",support->name);
			       
                               ////////p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               //////////p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);
/////Moved lift config test from here to put above
//                                p3d_mat4Copy(Tplacement0, Tplacement);
//                                Tplacement[2][3]+= 0.03;
//                                p3d_set_freeflyer_pose ( object, Tplacement );
//                                
// 			         ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
//          p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
// 	 manipulation->checkConfigForCartesianMode(graspConf, object);
// 	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
// 	 
// 	   manipulation->robot()->isCarryingObject = TRUE;
//                                   (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
// 				  
//                                liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
// //                                liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, p3d_mat4IDENTITY,  0, 0, armID, false );
//                                  g3d_draw_allwin_active();
//                                if ( liftConf==NULL )
//                                {
//                                 printf(" Fail to find liftConf\n");
//                                 continue;
//                                }
//                                printf(" LiftConf found\n");
//// End of moved lift config test
			       
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 ////////////////deactivateCcCntrts(manipulation->robot(), armID);

                                 
                                
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               ////if(liftConf!=NULL)
                               ////{
				 
				  ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                            gpDeactivate_object_collisions(manipulation->robot(), object->joints[1]->o, handProp, armID);
                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
			   
			       
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
	 manipulation->checkConfigForCartesianMode(graspConf, object);
	 gpSet_grasp_configuration(manipulation->robot(), *igrasp, armID);
	 
	  ///// manipulation->robot()->isCarryingObject = TRUE;
            /////                      (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                
                                 //// ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  ////////manipulation->robot()->isCarryingObject = TRUE;
                                  ////////(*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }                           
		  #ifdef PR2_EXISTS_FOR_MA
		    if(by_agent==PR2_MA)
		    {
		    fixAllJointsWithoutArm(manipulation->robot(),armID);
		    }
		  #endif
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                                  if(graspConf==NULL)
				  {
				    printf(" HRI TASK PLANNER ERROR: At this point graspConf should not be NULL. There are some memory issues or some other problems. Resolve it. \n");
				    p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
				    return 0;
				  }
				     manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
			    
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);
                                 
                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                 
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
/*
                                  manipulation->robot()->tcur=place_trajs[0];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);

                                  manipulation->robot()->tcur=place_trajs[1];

                                  g3d_show_tcur_rob(manipulation->robot(),default_drawtraj_fct);
*/
                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_TAKE;
                                  traj_sub_task.traj=take_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=REACH_TO_GRASP;
                                  traj_sub_task.traj=take_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=GRASP;
                                  traj_sub_task.traj=take_trajs[2];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=LIFT_OBJECT;
                                  traj_sub_task.traj=place_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);


                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=CARRY_OBJECT;
                                  traj_sub_task.traj=place_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task);

				 grasp_id_info_map.erase( grasp_id_info_map.find(igrasp->ID));
				  
                                  if(task==HIDE_OBJECT||task==MAKE_OBJECT_ACCESSIBLE) //Need to place and release the object
                                  {
                                  p3d_set_freeflyer_pose ( object, Tfinal_placement );
                                  //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                                  //printf(" >>> Obj final pos (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);

                                  manipulation->getManipulationData().clear();
                                  manipulation->getManipulationData().setAttachFrame(tAtt);
                                  manipulation->getManipulationData().setGraspConfig(placeConf);
                                  manipulation->getManipulationData().setGrasp(&(*igrasp));

                                  p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                  ////ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );

                                  g3d_draw_allwin_active();

                                  MANIPULATION_TASK_MESSAGE release_obj_status = manipulation->armEscapeObject(armID, placeConf,  object, release_obj_trajs);
                                  printf(" release_obj_status= %d \n",release_obj_status);
                                  ////return 0;
                                  if(release_obj_status==MANIPULATION_TASK_OK)
                                   {
                                  printf("release_obj_trajs.size()=%d\n",release_obj_trajs.size());
                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=RELEASE_OBJECT;
                                  traj_sub_task.traj=release_obj_trajs[0];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task); 

                                  traj_sub_task.armID=armID;
                                  traj_sub_task.sub_task_type=RETREAT_HAND;
                                  traj_sub_task.traj=release_obj_trajs[1];
				  traj_sub_task.traj_for_agent=by_agent;
                                  res_trajs.sub_task_traj.push_back(traj_sub_task); 
                                   }
                                  } 
/* Below is commented because no need to concatinate traj now and no need to generate soft motion
 
                                  manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
*/
				
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                     manipulation->robot()->isCarryingObject = FALSE;
                                   p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                   p3d_set_and_update_this_robot_conf ( object, obj_refConf );
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );

                                    p3d_destroy_config ( manipulation->robot(), refConf );
                                    p3d_destroy_config (object, obj_refConf );
                                    p3d_destroy_config ( manipulation->robot(), qcur );
                                   return 1;
                                  }
                                  else 
				  {
				    printf ( " no path found for place \n");
				    ////return 1;
				    
				  }
                               ////}

                              }
                           ////}
                        }
                     ////}

                     ////return 1;
                  ////}
                  ////else
                  ////{
                  ////   ManipulationUtils::printManipulationMessage ( status );
                  ////}
               ////}
            ////}
         ////}
      ////}
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
     //////// manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      

   }
   
  }
  
      manipulation->robot()->isCarryingObject = FALSE;
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
   result= 0;
  p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
  p3d_set_and_update_this_robot_conf ( object, obj_refConf );
				   
   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( object, obj_refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   //////////p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}


int JIDO_give_obj_to_human ( char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> &graspList, std::list<gpPlacement> &curr_placementList)
{
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

//    if ( manipulation== NULL )
//    {
//       initManipulation();
//    }

int PLAN_IN_CARTESIAN=1;
if(PLAN_IN_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   int armID= 0;
   p3d_matrix4 Tplacement0, T;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> place_trajs;
   double visibility, confCost;

   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );


   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL;
 
   refConf= p3d_get_robot_config ( manipulation->robot() );


////   ( *manipulation->robot()->armManipulationData ) [armID].setManipState ( handFree ) ;
   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );

   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );

   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      for ( int i=0; i<5; ++i )
      {
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );

         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {/*printf(" No IK Found to grasp\n");*/
          continue;
         }
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );

            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               g3d_draw_allwin_active();
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );

                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, trajs );

//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " Found for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
                     manipulation->concatTrajectories ( trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );

                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[indices_of_MA_agents[for_agent]], curr_placementList );

                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementList.begin(); iplacement!=curr_placementList.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if ( iplacement->stability<=0 ) //As the placement list is sorted
                           {
                              break;
                           }

                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;

                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                           p3d_set_freeflyer_pose ( object, Tplacement );
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );

                           //// p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );

                           if(task==GIVE_OBJECT||task==SHOW_OBJECT)
                           {
                           p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos) > 30*DEGTORAD)
                            { 
                            continue; 
                            }
                           }

                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);

                           for ( int j=0; j<5; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                              p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,testFrame);
                              
                              placeConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {/*printf(" No IK Found to place\n");*/
                               continue;
                              }

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                g3d_draw_allwin_active();

                                
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[for_agent]->perspective->fov, object, &visibility, 0 );

                                printf ( " mini_visibility_threshold_for_task[task]= %lf, maxi_visibility_threshold_for_task[task]=%lf, visibility=%lf\n",mini_visibility_threshold_for_task[task], maxi_visibility_threshold_for_task[task],visibility );

                                if(mini_visibility_threshold_for_task[task]>visibility||maxi_visibility_threshold_for_task[task]<visibility) 
                                {
                                printf(" Visibility NOT OK \n");
                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                continue;
                                } 

                                
                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                
                                
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );

                               p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "SHELF" );
                               //////////p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.1;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                               
                               if ( liftConf==NULL )
                               {
                                continue;
                               }

                              
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);

                                 //This test is to avoid the situations where robot should not make a big loop but it is doing so. Like between grasp and lift configs TODO: Do such tests between approach and open hand grasp configs also
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1)
                                 {
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   continue;
                                  }
                                
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               ////if(liftConf!=NULL)
                               ////{
                                  printf(" %d th IK Found to lift \n",j);
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);

                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
                                 manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                    
                                   manipulation->setSafetyDistanceValue ( orig_safety_dist );
                                  return 1;
                                  }////else {printf ( " no path found for place \n"); return 1;}
                               ////}

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }

   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}

int JIDO_give_obj_to_human_working_version_24_2_2011 ( char *obj_to_manipulate,  candidate_poins_for_task *curr_candidate_points)
{
  float clock0, elapsedTime;
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

  // get_set_of_points_to_give_object ( obj_to_manipulate );
  // reverse_sort_weighted_candidate_points_to_give_obj();

   printf ( " <<<<<< curr_candidate_points->no_points = %d >>>>>>>>\n", curr_candidate_points->no_points );
   if ( curr_candidate_points->no_points<=0 )
   {
      printf ( " AKP ERROR : No Candidate points\n" );
      return 0;
   }

   UPDATE_MIGHTABILITY_MAP_INFO=0;
   SHOW_MIGHTABILITY_MAP_INFO=0;
//SHOW_CURRENT_TASK_CANDIDATE_POINTS=0;
   configPt obj_actual_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_actual_pos );

   configPt obj_tmp_pos = MY_ALLOC ( double,envPt_MM->robot[obj_index]->nb_dof ); /* Allocation of temporary robot configuration */
   p3d_get_robot_config_into ( envPt_MM->robot[obj_index],&obj_tmp_pos );

   if ( manipulation== NULL )
   {
      initManipulation();
   }

int PLAN_IN_CARTESIAN=1;
if(PLAN_IN_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
    }

   clock0= clock();

   p3d_vector3 startPoint, endPoint, pointingDirection, headCenter, intersection1, intersection2;
   double headRadius;
   std::list<gpGrasp> graspList;
   std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
   std::vector <SM_TRAJ> smTrajs;
   ManipulationData configs ( manipulation->robot() );
   ArmManipulationData mData = ( *manipulation->robot()->armManipulationData ) [0];
   p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
   int armID= 0;
   p3d_matrix4 Tplacement0, T;
//   gpGrasp grasp;
   MANIPULATION_TASK_MESSAGE status;
   int result;
   gpHand_properties armHandProp = ( *manipulation->robot()->armManipulationData ) [0].getHandProperties();
   std::vector <double>  m_objStart ( 6 ), m_objGoto ( 6 );
   configPt q = NULL;
   gpHand_properties handProp = mData.getHandProperties();
   point_co_ordi goal_pos, point_to_give;
   int i1=0;
   double x, y, z, rx, ry, rz;
   MANIPULATION_TASK_MESSAGE message;
   configPt qcur= p3d_get_robot_config ( manipulation->robot() );

   bool quit= false;
   p3d_traj *traj = NULL;
   std::vector <p3d_traj*> trajs;
   std::vector <p3d_traj*> place_trajs;
   double visibility, confCost;


   gpGet_grasp_list ( object->name, armHandProp.type, graspList );
   p3d_get_freeflyer_pose ( object, Tplacement0 );
   p3d_get_freeflyer_pose2 ( object, &x, &y, &z, &rx, &ry, &rz );

//    ////gpReduce_grasp_list_size ( graspList, graspList, 30 );
    std::list<gpGrasp> graspList2;
    graspList2= graspList;
    gpGrasp_handover_filter(p3d_get_robot_by_name ( "SAHandRight" ), p3d_get_robot_by_name ( "SAHandRight2" ), object, graspList, graspList2);
    printf(" After gpGrasp_handover_filter()\n");
    printf(" graspList.size()=%d,graspList2.size()=%d\n",graspList.size(),graspList2.size());
   ////return 0;
//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
   p3d_matrix4 handFrame, tAtt, Tobject;
   configPt refConf= NULL, approachConf= NULL, graspConf= NULL, openConf= NULL, liftConf= NULL, placeConf= NULL;
   std::list<gpPlacement> curr_placementListOut;
   ////get_placements_in_3D ( object,  curr_placementListOut );
   get_placements_in_3D ( obj_to_manipulate,  curr_placementListOut );
   refConf= p3d_get_robot_config ( manipulation->robot() );


// while(graspList.size()>5)
// graspList.pop_back();

////   ( *manipulation->robot()->armManipulationData ) [armID].setManipState ( handFree ) ;
   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
   int grasp_ctr=0;
   double orig_safety_dist=manipulation->getSafetyDistanceValue();
   manipulation->setSafetyDistanceValue ( 0.0 );
   for ( std::list<gpGrasp>::iterator igrasp=graspList.begin(); igrasp!=graspList.end(); ++igrasp )
   {
      grasp_ctr++;
      p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
      printf ( "grasp id= %d\n",igrasp->ID );
      p3d_set_freeflyer_pose ( object, Tplacement0 );


      p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, manipulation->robot()->ccCntrts[armID]->Tatt2, tAtt );

      setMaxNumberOfTryForIK ( 3000 );
      p3d_get_freeflyer_pose ( object, Tobject );
      ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
      for ( int i=0; i<5; ++i )
      {
         p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );

         graspConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tobject, tAtt,  0, 0, armID, false );
         //graspConf= getGraspConf(object, armID, grasp, tAtt, confCost);


         if(graspConf==NULL)
         {/*printf(" No IK Found to grasp\n");*/}
         printf ( "graspConf= %p\n", graspConf );
         if ( graspConf!=NULL )
         {
            manipulation->getManipulationData().setAttachFrame(tAtt);
            manipulation->getManipulationData().setGrasp(&(*igrasp));
            p3d_set_and_update_this_robot_conf ( manipulation->robot(), graspConf );
            ManipulationUtils::unFixAllHands(manipulation->robot());
            gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
            p3d_get_robot_config_into ( manipulation->robot(), &graspConf );
            g3d_draw_allwin_active();

            approachConf= manipulation->getManipulationConfigs().getApproachFreeConf ( object, armID, *igrasp, graspConf, tAtt );
            if ( approachConf==NULL )
            {  
            p3d_destroy_config( manipulation->robot(), graspConf );  
            continue;
            } 
            ////else
            ////{
               p3d_set_and_update_this_robot_conf ( manipulation->robot(), approachConf );
               g3d_draw_allwin_active();
               openConf= manipulation->getManipulationConfigs().getOpenGraspConf ( object, armID, *igrasp, graspConf );
               printf ( "approachConf= %p\n", approachConf );
               if ( openConf==NULL )
               {  
                  p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  continue;
               } 
               ////else
               ////{
                  printf ( "openConf= %p\n", openConf );
                  p3d_set_collision_tolerance_inhibition ( object, TRUE );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), refConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), approachConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), openConf );
//                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );

                    //Added by Mokhtar to fix the issue of object colliding with environment in the generated plan
                   p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                   p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );

                  status= manipulation->armPickGoto ( armID, refConf, object, graspConf, openConf,  approachConf, trajs );

//                   p3d_destroy_config( manipulation->robot(), graspConf );
                  p3d_destroy_config( manipulation->robot(), approachConf );
                  p3d_destroy_config( manipulation->robot(), openConf );
                  ////return 0;         
                  // manipulation->cleanRoadmap();
                  if ( status==MANIPULATION_TASK_OK )
                  {
                     printf ( " Found for grasp_ctr=%d, and IK %d \n",grasp_ctr,i );
                     manipulation->concatTrajectories ( trajs, &traj );
                     MANPIPULATION_TRAJECTORY_CONF_STR conf;
                     SM_TRAJ smTraj_to_pick;
                     manipulation->computeSoftMotion ( traj, conf, smTraj_to_pick );

                     //g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 1 );
                     //printf ( " visibility=%lf\n",visibility );

                     for ( int i1=0;i1<curr_candidate_points->no_points;i1++ )
                     {
                        printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
                        goal_pos.x=curr_candidate_points->point[i1].x;
                        goal_pos.y=curr_candidate_points->point[i1].y;
                        goal_pos.z=curr_candidate_points->point[i1].z+0.01;

                        point_to_give.x=goal_pos.x;
                        point_to_give.y=goal_pos.y;
                        point_to_give.z=goal_pos.z;

                        obj_tmp_pos[6]=point_to_give.x;
                        obj_tmp_pos[7]=point_to_give.y;
                        obj_tmp_pos[8]=point_to_give.z;

                        get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );
                        int plac_ctr=0 ;
                        for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
                        {
                           plac_ctr++;
                           ////iter->draw(0.05);
                           if ( iplacement->stability<=0 ) //As the placement list is sorted
                              break;
                           p3d_mat4Copy ( Tplacement0, T );

                           iplacement->position[0]= point_to_give.x;
                           iplacement->position[1]= point_to_give.y;
                           iplacement->position[2]= point_to_give.z;
                           p3d_matrix4 Tplacement;
                           iplacement->computePoseMatrix ( Tplacement );
                            p3d_set_freeflyer_pose ( object, Tplacement );
                            //p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
                            //printf(" >>> (x, y, z, rx, ry, rz)=(%lf, %lf, %lf, %lf, %lf, %lf) \n",x, y, z, rx, ry, rz);
                           p3d_mat4Mult ( igrasp->frame, handProp.Tgrasp_frame_hand, handFrame );
                           p3d_mat4Mult ( handFrame, (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->Tatt2, tAtt );
                            p3d_mat4Mult (  Tplacement, tAtt, WRIST_FRAME );
//                             g3d_win *win= NULL;
//                             win= g3d_get_cur_win();
//                             win->fct_draw2= &(execute_Mightability_Map_functions);

//                             p3d_mat4ExtractTrans(FRAME, startPoint);
//                             p3d_mat4ExtractColumnZ(FRAME, pointingDirection);
//                             for(int ki=0; ki<3; ++ki)
//                             { endPoint[ki]= startPoint[ki] + 10*pointingDirection[ki]; }
//                             p3d_mat4ExtractTrans(HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, headCenter);
//                             headRadius= 0.5;
// p3d_vectCopy(headCenter, SPHERE_CENTER);
// p3d_vectCopy(startPoint, STARTPOINT);
// p3d_vectCopy(endPoint, ENDPOINT);
//                             p3d_vectSub(headCenter, wristPosition, )
// 
// 
//                           //  g3d_draw_allwin_active();
//                             if( gpLine_segment_sphere_intersection(startPoint, endPoint, headCenter, headRadius, intersection1, intersection2)==0 )
//                             { continue; }
                           if( get_wrist_head_alignment_angle(WRIST_FRAME, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos) > 30*DEGTORAD)
                            { continue; }

                           // it is reactivated in armPickGoTo, so we need to deactivate again:
                           gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                           for ( int j=0; j<5; ++j )
                           {
                              deactivateCcCntrts(manipulation->robot(), armID);
                              p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              ManipulationUtils::unFixAllHands(manipulation->robot());
                              gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );
                              p3d_set_freeflyer_pose ( object, Tplacement );
                              p3d_matrix4 testFrame;
                              p3d_mat4Copy(Tplacement,testFrame);
                              
                              placeConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                              ////ManipulationUtils::copyConfigToFORM ( object, placeConf ); 
                              ////pqp_print_colliding_pair();
                              if(placeConf==NULL)
                              {/*printf(" No IK Found to place\n");*/}

                              ////g3d_draw_allwin_active();
                              if(placeConf!=NULL)
                              {
                                manipulation->getManipulationData().setAttachFrame(tAtt);
                                if(PLAN_IN_CARTESIAN == 1)
                                {
                                manipulation->checkConfigForCartesianMode(placeConf, object);
                                }

                                p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
                                g3d_draw_allwin_active();
                                g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->fov, object, &visibility, 0 );
                                printf ( " visibility=%lf\n",visibility );
                                if(visibility<=0.1)
                                {
                                 printf(" IK found to place but the object's visibility %lf is not good\n",visibility);
                                 p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                                 continue;
                                }

                                 printf(" IK found to place and the object's visibility %lf is good\n",visibility); 
                                ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), refConf );
                              //ManipulationUtils::unFixAllHands(manipulation->robot());
                              //gpSet_grasp_configuration ( manipulation->robot(), *igrasp, armID );
                              //ManipulationUtils::fixAllHands (manipulation->robot(), NULL, false );

                               p3d_rob* support= ( p3d_rob* ) p3d_get_robot_by_name ( "SHELF" );
                               p3d_set_and_update_this_robot_conf ( manipulation->robot(), placeConf );
//                                MANIPULATION_TASK_MESSAGE place_status= manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, place_trajs);

                               p3d_set_freeflyer_pose ( object, Tplacement0 );
//                                liftConf=  manipulation->getApproachGraspConf(object, armID, *igrasp, graspConf, tAtt);

                               p3d_mat4Copy(Tplacement0, Tplacement);
                               Tplacement[2][3]+= 0.1;
                               p3d_set_freeflyer_pose ( object, Tplacement );
                               liftConf= sampleRobotGraspPosWithoutBase ( manipulation->robot(), Tplacement, tAtt,  0, 0, armID, false );
                               
                               if ( liftConf ){
                                  ////p3d_set_and_update_this_robot_conf ( manipulation->robot(), liftConf );
                                 //// g3d_draw_allwin_active();
                                 deactivateCcCntrts(manipulation->robot(), armID);
                                 if (optimizeRedundentJointConfigDist(manipulation->robot(), (*manipulation->robot()->armManipulationData)[armID].getCcCntrt()->argu_i[0], liftConf, object->joints[1]->abs_pos, tAtt, graspConf, armID, manipulation->getManipulationConfigs().getOptimizeRedundentSteps()) == -1){
                                   p3d_destroy_config(manipulation->robot(), liftConf);
                                   liftConf = NULL;
                                   continue;
                                  }
                                }
                                else
                                {
                                 
                                } 
                               p3d_set_freeflyer_pose ( object, Tplacement0);

                               ManipulationUtils::copyConfigToFORM ( manipulation->robot(), graspConf );
                               if(liftConf!=NULL)
                               {printf(" %d th IK Found to lift \n",j);
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), liftConf );
                                  ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf );
//                                   ManipulationUtils::fixAllHands ( manipulation->robot(), NULL, false );
                                  manipulation->robot()->isCarryingObject = TRUE;
                                  (*manipulation->robot()->armManipulationData)[armID].setCarriedObject(object);
                                   manipulation->cleanRoadmap();
                                  
                                 //Added by Mokhtar to avoid the object collision issue of planner
                                  p3d_multiLocalPath_disable_all_groupToPlan ( manipulation->robot() );
                                  p3d_multiLocalPath_set_groupToPlan ( manipulation->robot(), manipulation->getUpBodyMLP(), 1 );
                                  
//                                   if(PLAN_IN_CARTESIAN == 1) {
//     for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
//      manipulation->setArmCartesian(i,true);
//     }
//     }
                                  MANIPULATION_TASK_MESSAGE place_status = manipulation->armPickTakeToFree(armID, graspConf, placeConf, object, support, liftConf, *igrasp, place_trajs);

                                  if ( place_status==MANIPULATION_TASK_OK )
                                  {
                                  printf ( " Found for grasp_ctr=%d, and plac_ctr %d \n",grasp_ctr,plac_ctr );
                                 manipulation->concatTrajectories ( place_trajs, &traj );
                                  MANPIPULATION_TRAJECTORY_CONF_STR conf;
                                  SM_TRAJ smTraj_to_place;
                                  manipulation->computeSoftMotion ( traj, conf, smTraj_to_place );
//                                   ManipulationUtils::copyConfigToFORM ( manipulation->robot(), placeConf ); //If want to store the config for visualization
                                  p3d_set_freeflyer_pose ( object, Tplacement0); 
                                  gpDeactivate_object_fingertips_collisions( manipulation->robot(), object->joints[1]->o, armHandProp, armID);
                                   elapsedTime= ( clock()-clock0 ) /CLOCKS_PER_SEC;
                                   printf("Computation time: %2.1fs= %dmin%ds\n",elapsedTime, ( int ) ( elapsedTime/60.0 ), ( int ) ( elapsedTime - 60* ( ( int ) ( elapsedTime/60.0 ) ) ) );
                                  return 1;
                                  }else {printf ( " no path found for place \n"); return 1;}
                               }

                              }
                           }
                        }
                     }

                     ////return 1;
                  }
                  else
                  {
                     ManipulationUtils::printManipulationMessage ( status );
                  }
               ////}
            ////}
         }
      }
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs)
      manipulation->setSafetyDistanceValue ( orig_safety_dist );
// MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs)
      continue;
/*

      printf ( " >>>>> before armPlanTask(ARM_PICK_GOTO \n" );
      status = manipulation->armPlanTask ( ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "", *igrasp, confs, smTrajs );
      printf ( " >>>>>**** after armPlanTask(ARM_PICK_GOTO \n" );

      if ( qcur!=NULL )
         p3d_destroy_config ( manipulation->robot(), qcur );

      qcur= p3d_get_robot_config ( manipulation->robot() );
      p3d_copy_config_into ( manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur );

      if ( status==MANIPULATION_TASK_OK )
      {
         //grasp= manipulation->getCurrentGrasp();
         // grasp= *( manipulation->getManipulationData().getGrasp() );
         //return 0;
         remove ( "softMotion_Smoothed_Q_goto.traj" );
         remove ( "softMotion_Smoothed_Seg_goto.traj" );
         rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj" );
         rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj" );
      }
      else
      {
         printf ( "%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__ );
         continue;
      }
      //grasp.print();


      p3d_mat4Mult ( igrasp->frame, armHandProp.Tgrasp_frame_hand, handFrame );
      p3d_mat4Mult ( handFrame, mData.getCcCntrt()->Tatt2, tAtt );
      //Check if there is a valid configuration of the robot using this graspFrame

      // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
      // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);

      for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      {
         printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );
//          if(i1==0)
//     goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
         goal_pos.x=curr_candidate_points->point[i1].x;
         goal_pos.y=curr_candidate_points->point[i1].y;
         goal_pos.z=curr_candidate_points->point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
//    goal_pos.x=T0[0][3];
//    goal_pos.y=T0[1][3];
//    goal_pos.z=T0[2][3]+0.04;


         point_to_give.x=goal_pos.x;
         point_to_give.y=goal_pos.y;
         point_to_give.z=goal_pos.z;

         obj_tmp_pos[6]=point_to_give.x;
         obj_tmp_pos[7]=point_to_give.y;
         obj_tmp_pos[8]=point_to_give.z;

         get_ranking_based_on_view_point ( HRI_AGENTS_FOR_MA[HUMAN1_MA]->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut );

         for ( std::list<gpPlacement>::iterator iplacement=curr_placementListOut.begin(); iplacement!=curr_placementListOut.end(); ++iplacement )
         {
            ////iter->draw(0.05);
            if ( iplacement->stability<=0 ) //As the placement list is sorted
               break;
            p3d_mat4Copy ( Tplacement0, T );

            iplacement->position[0]= point_to_give.x;
            iplacement->position[1]= point_to_give.y;
            iplacement->position[2]= point_to_give.z;
            p3d_matrix4 Tplacement;
            iplacement->computePoseMatrix ( Tplacement );
            p3d_set_freeflyer_pose ( object, Tplacement );
            p3d_get_freeflyer_pose2 ( object,  &x, &y, &z, &rx, &ry, &rz );
// // //        T[0][3]=point_to_give.x;
// // //    T[1][3]=point_to_give.y;
// // //    T[2][3]=point_to_give.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_give.x, point_to_give.y, point_to_give.z, rx, ry, rz);

            double visibility_threshold=95.0;
            int is_visible=is_object_visible_for_agent ( HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1 );
            printf ( " is_visible for place %d= %d \n",i1,is_visible );
            if ( is_visible==0 )
               continue;

            configPt qobject= p3d_get_robot_config ( object );
            p3d_copy_config_into ( object, qobject, &object->ROBOT_GOTO );
            p3d_destroy_config ( object, qobject );


            g3d_draw_allwin_active();
            p3d_set_freeflyer_pose ( object, T0 );

            m_objStart.resize ( 6 );
            m_objGoto.resize ( 6 );
            m_objStart[0]= P3D_HUGE;
            m_objStart[1]= P3D_HUGE;
            m_objStart[2]= P3D_HUGE;
            m_objStart[3]= P3D_HUGE;
            m_objStart[4]= P3D_HUGE;
            m_objStart[5]= P3D_HUGE;

            m_objGoto[0]= point_to_give.x;
            m_objGoto[1]= point_to_give.y;
            m_objGoto[2]= point_to_give.z;
            m_objGoto[3]= rx;
            m_objGoto[4]= ry;
            m_objGoto[5]= rz;


            p3d_copy_config_into ( manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS );
            //switch to cartesian for the giving motion:
            // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

            status= manipulation->armPlanTask ( ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, ( char* ) obj_to_manipulate, ( char* ) "HRP2TABLE", confs, smTrajs );
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

            printf ( " After armPlanTask, result = %d \n",status );
            //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
            printf ( "checking for place %d and grasp id= %d\n",i1, igrasp->ID );

            if ( status==MANIPULATION_TASK_OK )
            {
               remove ( "softMotion_Smoothed_Q_place.traj" );
               remove ( "softMotion_Smoothed_Seg_place.traj" );
               rename ( "softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj" );
               rename ( "softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj" );
               printf ( ">>>>> Found for place %d and grasp id= %d\n",i1, igrasp->ID );
               result= 1;
               quit= true;

//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   // COMPUTE THE SOFTMOTION TRAJECTORY 
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
               break;
            }

         }//End of placement itr
         if ( quit==true )
            break;
      }//End for ( i1=0;i1<curr_candidate_points->no_points;i1++ )
      if ( quit==true )
      {
         break;
      }



      result= 0;*/
   }

   p3d_destroy_config ( manipulation->robot(), refConf );
   p3d_destroy_config ( manipulation->robot(), qcur );
   p3d_set_freeflyer_pose ( object, Tplacement0 );

//WARNING: TMP for Videos, Comment the line below
   p3d_col_deactivate_robot ( object );

// g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
// win->fct_draw2= & ( fct_draw_loop);

   return result;

}



int copy_HRI_task_candidate_points(candidate_poins_for_task *from_candidate_points, candidate_poins_for_task *to_candidate_points)
{
  for(int i=0;i<from_candidate_points->no_points;i++)
  {
    to_candidate_points->point[i].x=from_candidate_points->point[i].x;
    to_candidate_points->point[i].y=from_candidate_points->point[i].y;
    to_candidate_points->point[i].z=from_candidate_points->point[i].z;
    to_candidate_points->weight[i]=from_candidate_points->weight[i];
    to_candidate_points->horizontal_surface_of[i]=from_candidate_points->horizontal_surface_of[i];
    
  }
  
  to_candidate_points->curr_solution_point_index=from_candidate_points->curr_solution_point_index;
  to_candidate_points->no_points=from_candidate_points->no_points;
  return 1;
}

#ifdef JIDO_EXISTS_FOR_MA
int JIDO_find_HRI_task_solution(HRI_TASK_TYPE CURR_TASK, HRI_TASK_AGENT for_agent, char *obj_to_manipulate)
{
   
   
   int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
    
   candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
   ////find_candidate_points_for_current_HRI_task(CURR_TASK,  for_agent, JIDO_MA,curr_resultant_candidate_points);
   
   find_candidate_points_for_current_HRI_task(CURR_TASK,  JIDO_MA, for_agent,curr_resultant_candidate_points);
   
   switch (CURR_TASK)
   {
     case MAKE_OBJECT_ACCESSIBLE:
       assign_weights_on_candidte_points_to_put_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN);
     break;
     
     case SHOW_OBJECT:
	assign_weights_on_candidte_points_to_show_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT,rob_indx.HUMAN);
      break;
      
     case GIVE_OBJECT:
       assign_weights_on_candidte_points_to_give_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN, 2);
      break;
      
      case HIDE_OBJECT:
       assign_weights_on_candidte_points_to_hide_obj(obj_to_manipulate, curr_resultant_candidate_points, rob_indx.JIDO_ROBOT, rob_indx.HUMAN);
      break;
      
   }
  
   
   reverse_sort_HRI_task_weighted_candidate_points(curr_resultant_candidate_points);
   CANDIDATE_POINTS_FOR_TASK_FOUND=1;
   ////CANDIDATE_POINTS_FOR_CURRENT_TASK=curr_resultant_candidate_points;
   copy_HRI_task_candidate_points(curr_resultant_candidate_points,&resultant_current_candidate_point);
   CANDIDATE_POINTS_FOR_CURRENT_TASK=&resultant_current_candidate_point;
   MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);
}
#endif

#ifndef COMMENT_TMP

int JIDO_make_obj_accessible_to_human ( char *obj_to_manipulate )
{
   if(Affordances_Found==0)
    {
      printf(" >>>> MA ERROR : First initialize Mightability Maps. NOT moving ahead. \n");
      return 0;
    }
    
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );
 ////candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);
 ////find_candidate_points_for_current_HRI_task(CURRENT_HRI_MANIPULATION_TASK, JIDO, HUMAN1, curr_resultant_candidate_points);
 ////CANDIDATE_POINTS_FOR_TASK_FOUND=1;
 get_set_of_points_to_put_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_put_obj();

 printf ( " <<<<<< candidate_points_to_put.no_points = %d >>>>>>>>\n", candidate_points_to_put.no_points );
 ////return 0;
 if ( candidate_points_to_put.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_put;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;
  p3d_traj *traj = NULL;
  std::vector <p3d_traj*> trajs;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//    ////gpReduce_grasp_list_size ( graspList, graspList, 30 );
//    std::list<gpGrasp> graspList2;
//    graspList2= graspList;
//    gpGrasp_handover_filter(p3d_get_robot_by_name ( "SAHandRight" ), p3d_get_robot_by_name ( "SAHandRight2" ), object, graspList, graspList2);
//    printf(" After gpGrasp_handover_filter()\n");
//    printf(" graspList.size()=%d,graspList2.size()=%d\n",graspList.size(),graspList2.size());
   ////return 0;
//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
std::list<gpPlacement> curr_placementListOut;
std::list<gpPlacement> general_stable_configuration_list; 
std::list<gpPlacement> tmp_placement_list; 
  ////////get_placements_in_3D(object,  curr_placementListOut);
  gpCompute_stable_placements (object, general_stable_configuration_list );
  ////curr_placementListOut=general_stable_confi_list;
  
   int valid_grasp_ctr=0;

  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     ////if(iter->ID==11)
     ////continue;
     p3d_set_freeflyer_pose(object, T0);
////    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
    printf(" >>>>> before armPlanTask(ARM_PICK_GOTO \n");
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    printf(" >>>>>**** after armPlanTask(ARM_PICK_GOTO \n");

    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);
    printf(" >>>>>>> Status of manipulation->armPlanTask(ARM_PICK_GOTO) = %d\n",status);
    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
     valid_grasp_ctr++;
     remove("softMotion_Smoothed_Q_goto.traj");
     remove("softMotion_Smoothed_Seg_goto.traj");
     rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_goto.traj");
     rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_goto.traj");
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_put.point[i1].x;
	goal_pos.y=candidate_points_to_put.point[i1].y;
	goal_pos.z=candidate_points_to_put.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_put.x=goal_pos.x;
	point_to_put.y=goal_pos.y;
	point_to_put.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_put.x;
	obj_tmp_pos[7]=point_to_put.y;
	obj_tmp_pos[8]=point_to_put.z;
        
        curr_placementListOut.clear();
        get_placements_at_position(  object, goal_pos, general_stable_configuration_list, 10, curr_placementListOut );
        //printf("stable_placements_list.size()=%d, curr_placementListOut.size = %d \n", stable_placements_list.size(), curr_placementListOut.size());
        printf("curr_placementListOut.size = %d \n",  curr_placementListOut.size());
//         if(curr_placementListOut.size()==0)
//         {
//          continue;
//         ////return 0;
//         }
        tmp_placement_list.clear();
        tmp_placement_list= curr_placementListOut;
	gpPlacement_on_support_filter ( object, envPt_MM->robot[candidate_points_to_put.horizontal_surface_of[i1]],tmp_placement_list,curr_placementListOut );
        ////get_ranking_based_on_view_point(primary_human_MM->perspective->camjoint->abs_pos,goal_pos, object, envPt_MM->robot[rob_indx.HUMAN], curr_placementListOut);

        for(std::list<gpPlacement>::iterator iter_p=curr_placementListOut.begin(); iter_p!=curr_placementListOut.end(); ++iter_p)
        {
  ////iter->draw(0.05); 
         if(iter_p->stability<=0)//As the placement list is sorted
         break;
                p3d_mat4Copy(T0, T);
	
        iter_p->position[0]= point_to_put.x;
    	iter_p->position[1]= point_to_put.y;
    	iter_p->position[2]= point_to_put.z;
        p3d_matrix4 Tplacement;
        iter_p->computePoseMatrix(Tplacement);
        p3d_set_freeflyer_pose(object, Tplacement);
        p3d_get_freeflyer_pose2(object,  &x, &y, &z, &rx, &ry, &rz);
// // //        T[0][3]=point_to_put.x;
// // // 	T[1][3]=point_to_put.y;
// // // 	T[2][3]=point_to_put.z;

// // //        p3d_set_freeflyer_pose2(object, point_to_put.x, point_to_put.y, point_to_put.z, rx, ry, rz);

       double visibility_threshold=85.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 1);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==0)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);


        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);

        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_put.x;
        m_objGoto[1]= point_to_put.y;
        m_objGoto[2]= point_to_put.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;


       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
       //switch to cartesian for the giving motion:
      // (*manipulation->robot()->armManipulationData)[armID].setCartesian(true);

        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);
//         trajs.clear();
//         status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", trajs);

         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d, grasp id= %d and placement id = %d\n",i1, iter->ID,iter_p->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         remove("softMotion_Smoothed_Q_place.traj");
         remove("softMotion_Smoothed_Seg_place.traj");
         rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
         rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
         printf(">>>>> Found for place %d and grasp id= %d and placement id = %d\n",i1, iter->ID,iter_p->ID);
           result= 1;
           quit= true;
           
//             if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
//                   /* COMPUTE THE SOFTMOTION TRAJECTORY */
//                   MANPIPULATION_TRAJECTORY_CONF_STR conf;
//                   SM_TRAJ smTraj;
//                   manipulation->computeSoftMotion(traj, conf, smTraj);
//                   rename("softMotion_Smoothed_Q.traj", "softMotion_Smoothed_Q_place.traj");
//                   rename("softMotion_Smoothed_Seg.traj", "softMotion_Smoothed_Seg_place.traj");
//               }
//           manipulation->robot()->isCarryingObject = true;
//            manipulation->robot()->carriedObject = object;
//            (*manipulation->robot()->armManipulationData)[0].setCarriedObject(object);
           break;
        }
     
      }//End of placement itr
       if(quit==true)
      break;
     }//End for ( i1=0;i1<candidate_points_to_put.no_points;i1++ )
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);

//WARNING: TMP for Videos, Comment the line below
 p3d_col_deactivate_robot(object);

 // g3d_win *win= NULL;
//  win= g3d_get_win_by_name((char *)"Move3D");
 // win->fct_draw2= & ( fct_draw_loop);
 printf(" valid_grasp_ctr=%d\n",valid_grasp_ctr);
 return result;
 
}
#endif
#ifndef COMMENT_TMP
int JIDO_hide_obj_from_human ( char *obj_to_manipulate )
{
 int obj_index=get_index_of_robot_by_name ( obj_to_manipulate );

 get_set_of_points_to_hide_object ( obj_to_manipulate );
 reverse_sort_weighted_candidate_points_to_hide_obj();

 printf ( " <<<<<< candidate_points_to_hide.no_points = %d >>>>>>>>\n", candidate_points_to_hide.no_points );
 if ( candidate_points_to_hide.no_points<=0 )
 {
	printf ( " AKP ERROR : No Candidate points\n" );
	return 0;
 }

configPt obj_actual_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_actual_pos);

configPt obj_tmp_pos = MY_ALLOC(double,envPt_MM->robot[obj_index]->nb_dof); /* Allocation of temporary robot configuration */
p3d_get_robot_config_into(envPt_MM->robot[obj_index],&obj_tmp_pos);

 if (manipulation== NULL) {
	  initManipulation();
	}

	

//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//         manipulation->setSupport((char*)SUPPORT_NAME);
//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//         manipulation->setCameraFOV(CAMERA_FOV);
//         manipulation->setCameraImageSize(200, 200);
  std::list<gpGrasp> graspList;
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <p3d_traj*> trajs;
  ManipulationData configs(manipulation->robot());
  ArmManipulationData mData = (*manipulation->robot()->armManipulationData)[0];
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*)obj_to_manipulate);
  int armID= 0;
  p3d_matrix4 T0, T, tAtt;
//   gpGrasp grasp;
  p3d_matrix4 handFrame;
  MANIPULATION_TASK_MESSAGE status;
  int result;
  gpHand_properties armHandProp = (*manipulation->robot()->armManipulationData)[0].getHandProperties();
  std::vector <double>  m_objStart(6), m_objGoto(6);
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  point_co_ordi goal_pos, point_to_hide;
  int i1=0;
  double x, y, z, rx, ry, rz;
  MANIPULATION_TASK_MESSAGE message;
  configPt qcur= p3d_get_robot_config(manipulation->robot());
  configPt q0= p3d_get_robot_config(manipulation->robot());
  bool quit= false;


  gpGet_grasp_list(object->name, armHandProp.type, graspList);
  p3d_get_freeflyer_pose(object, T0);
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);

//   status= manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), (char*)obj_to_manipulate, (char*)"", confs, smTrajs);
  for(std::list<gpGrasp>::iterator iter=graspList.begin(); iter!=graspList.end(); ++iter)
  {
     p3d_set_and_update_this_robot_conf(manipulation->robot(), q0);
     p3d_copy_config_into(manipulation->robot(), q0, &manipulation->robot()->ROBOT_POS);

     printf("grasp id= %d\n",iter->ID);
     p3d_set_freeflyer_pose(object, T0);
////    (*manipulation->robot()->armManipulationData)[armID].setManipState(handFree) ;
    
//     status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);
    status = manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"", *iter, confs, smTrajs);



    if(qcur!=NULL)
      p3d_destroy_config(manipulation->robot(), qcur);

    qcur= p3d_get_robot_config(manipulation->robot());
    p3d_copy_config_into(manipulation->robot(), manipulation->robot()->ROBOT_GOTO, &qcur);

    if(status==MANIPULATION_TASK_OK)
    { //grasp= manipulation->getCurrentGrasp();
    // grasp= *( manipulation->getManipulationData().getGrasp() );
     //return 0;
    }
    else { 
      printf("%s: %d: ARM_PICK_GOTO failed \n",__FILE__,__LINE__);
      continue;
    }
  //grasp.print();

  
    p3d_mat4Mult(iter->frame, armHandProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    //Check if there is a valid configuration of the robot using this graspFrame

  // gpSet_grasp_configuration(manipulation->robot(), grasp, handProp.type);
  // gpFix_hand_configuration(manipulation->robot(), handProp, handProp.type);
  
    for ( i1=0;i1<candidate_points_to_hide.no_points;i1++ )
    {
       printf("checking for place %d and grasp id= %d\n",i1, iter->ID);
//          if(i1==0)
//  	goal_pos.x=candidate_points_to_put.point[i1].x +0.5;
//          else
        goal_pos.x=candidate_points_to_hide.point[i1].x;
	goal_pos.y=candidate_points_to_hide.point[i1].y;
	goal_pos.z=candidate_points_to_hide.point[i1].z+0.01;
//goal_pos.z= T0[2][3] + 0.15;
// 	goal_pos.x=T0[0][3];
// 	goal_pos.y=T0[1][3];
// 	goal_pos.z=T0[2][3]+0.04;


	point_to_hide.x=goal_pos.x;
	point_to_hide.y=goal_pos.y;
	point_to_hide.z=goal_pos.z;

	obj_tmp_pos[6]=point_to_hide.x;
	obj_tmp_pos[7]=point_to_hide.y;
	obj_tmp_pos[8]=point_to_hide.z;

        p3d_mat4Copy(T0, T);
	T[0][3]=point_to_hide.x;
	T[1][3]=point_to_hide.y;
	T[2][3]=point_to_hide.z;

        p3d_set_freeflyer_pose2(object, point_to_hide.x, point_to_hide.y, point_to_hide.z, rx, ry, rz);

       double visibility_threshold=5.0;
       int is_visible=is_object_visible_for_agent(HRI_AGENTS_FOR_MA[HUMAN1_MA], object, visibility_threshold, 0, 0);
       printf(" is_visible for place %d= %d \n",i1,is_visible);
       if(is_visible==1)
       continue;

        configPt qobject= p3d_get_robot_config(object);
        p3d_copy_config_into(object, qobject, &object->ROBOT_GOTO);
        p3d_destroy_config(object, qobject);



        g3d_draw_allwin_active();
        p3d_set_freeflyer_pose(object, T0);
        for(int i=0; i<1; ++i)
        {
          //q = sampleRobotGraspPosWithoutBase(manipulation->robot(), T, tAtt, FALSE, FALSE, armID, true);
         // message= manipulation->findArmGraspsConfigs(armID, object, grasp, configs);
          //if(q!=NULL) break;
        }
	if(message!=MANIPULATION_TASK_OK) 
        {
//         printf("findArmGraspsConfigs != MANIPULATION_TASK_OK\n"); 
//         return 1;
//         continue;
        }
// 	if(q==NULL) 
//         {
//         printf("q==NULL after sampleRobotGraspPosWithoutBase()\n"); 
//         continue;
//         }
        m_objStart.resize(6);
        m_objGoto.resize(6);
        m_objStart[0]= P3D_HUGE;
        m_objStart[1]= P3D_HUGE;
        m_objStart[2]= P3D_HUGE;
        m_objStart[3]= P3D_HUGE;
        m_objStart[4]= P3D_HUGE;
        m_objStart[5]= P3D_HUGE;

        m_objGoto[0]= point_to_hide.x;
        m_objGoto[1]= point_to_hide.y;
        m_objGoto[2]= point_to_hide.z;
        m_objGoto[3]= rx;
        m_objGoto[4]= ry;
        m_objGoto[5]= rz;
//         m_objGoto[3]= P3D_HUGE;
//         m_objGoto[4]= P3D_HUGE;
//         m_objGoto[5]= P3D_HUGE;

       p3d_copy_config_into(manipulation->robot(), qcur, &manipulation->robot()->ROBOT_POS);
        status= manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,qcur, manipulation->robotGoto(),  m_objStart, m_objGoto, (char*)obj_to_manipulate, (char*)"HRP2TABLE", confs, smTrajs);


         printf(" After armPlanTask, result = %d \n",status);
	 //p3d_set_object_to_carry_to_arm(manipulation->robot(), 0, object->name );
         printf("checking for place %d and grasp id= %d\n",i1, iter->ID);

        if(status==MANIPULATION_TASK_OK)
        { 
         printf(">>>>> Found for place %d and grasp id= %d\n",i1, iter->ID);
           result= 1;
           quit= true;
           break;
        }
     }
     if(quit==true)
     { 
       break;
     }
    

    
    result= 0;
  }

 p3d_destroy_config(manipulation->robot(), q0);
 p3d_destroy_config(manipulation->robot(), qcur);
 p3d_set_freeflyer_pose(object, T0);
 return result;
 
}
#endif

int get_placements_at_position(p3d_rob *object, point_co_ordi point, std::list<gpPlacement> placementList, int no_rot, std::list<gpPlacement> &placementListOut)
{
  int i, count;
  p3d_vector3 zAxis={0.0,0.0,1.0};
  p3d_matrix4 Tcur, T1, T2, Tplacement;
  gpPlacement placement;
  std::vector<double> thetas;
  double verticalOffset= 0.01;

  placementList.sort();
  placementList.reverse();

  placementListOut.clear();


  p3d_get_freeflyer_pose(object, Tcur);

  if(no_rot==0)
  {
    thetas.push_back(0);
  }
  else
  {
    for(i=0; i<no_rot; ++i)
    {
     thetas.push_back( i*2*M_PI/((double) no_rot) );
    }
    
  }

  count= 0;
  for(std::list<gpPlacement>::iterator iterP=placementList.begin(); iterP!=placementList.end(); iterP++)
  { 
     if(count > 10)
     { break; }
     count++;


     for(i=0; i<thetas.size(); ++i)
     {
        placement= (*iterP);
        placement.theta= thetas[i];
	placement.position[0]= point.x;
	placement.position[1]= point.y;
	placement.position[2]= point.z + verticalOffset;

        placement.computePoseMatrix(Tplacement);

        p3d_set_freeflyer_pose(object, Tplacement);
	
        if(p3d_col_test_robot(object, 0))
	{ continue; }

	placementListOut.push_back(placement);
     }
  }

   placementListOut.sort();
   placementListOut.reverse();

  ////for(std::list<gpPlacement>::iterator iterP=placementListOut.begin(); iterP!=placementListOut.end(); iterP++)
  ////{ 
    ////std::cout << "stability " << iterP->stability << std::endl;
  ////}

  return 0;
}

int get_placements_in_3D(char *obj_to_manipulate,  std::list<gpPlacement> &placementListOut)
{
p3d_rob* object= ( p3d_rob* ) p3d_get_robot_by_name ( ( char* ) obj_to_manipulate );
  int i, j, k, n;
  double dalpha;
  p3d_matrix4 Tcur, T;
  gpPlacement placement;

  placementListOut.clear();


 
  dalpha= 40*(M_PI/180.0);
  n= 2*M_PI/dalpha;


  for(i=0; i<n; ++i)
  { 
   for(j=0; j<n; ++j)
   { 
	for(k=0; k<n; ++k)
	{ 
		p3d_mat4Pos (T, 0, 0, 0, i*dalpha, j*dalpha, k*dalpha);
		
        placement.stability=i+j+k;//AKP: Tmp for displaying the comfigs with different colors
        placement.polyhedron= object->o[0]->pol[0]->poly;
        p3d_mat4Copy(T, placement.T);
	placementListOut.push_back(placement);
                   
	}
    }
  }


 

  return 0;
}

//! This function is used to know how much a robot (visually) sees an object from a specific viewpoint.
//! \param camera_frame the frame of the viewpoint (the looking direction is X, Y points downward and Z to the left)
//! \param camera_fov the field of view angle of the robot's camera (in degrees)
//! \param robot pointer to the robot
//! \param object pointer to the object
//! \param result return the ratio between the number of object's pixels that are visible
//! and the overall size (in pixels) of the image. So the value is between 0 (invisible object) and 1 (the object
//! occupies all the image).
//! \return 0 in case of success, 1 otherwise
int g3d_is_object_visible_from_robot(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *robot, p3d_rob *object, double *result)
{
  if(robot==NULL)
  {
    printf("%s: %d: g3d_is_object_visible_from_robot(): input robot is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
  if(object==NULL)
  {
    printf("%s: %d: g3d_is_object_visible_from_robot(): input object is NULL.\n",__FILE__,__LINE__);
    return 1;
  }
 
  int i, width, height;
  GLint viewport_original[4], viewport[4];
  int displayFrame, displayJoints, displayShadows, displayWalls, displayFloor, displayTiles, cullingEnabled;
  double fov;
  int draw_logo;
  int count;
  unsigned char *image= NULL;
  float red, green, blue;
  g3d_win *win= g3d_get_win_by_name((char*) "Move3D");

  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_UNLIT_GREEN_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    if(XYZ_ENV->robot[i]==robot) {
      p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_NO_DISPLAY);
    }
    else
     p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_UNLIT_GREEN_DISPLAY);
  }
  // display our robot and object with specific colors:
  p3d_set_robot_display_mode(object, P3D_ROB_UNLIT_RED_DISPLAY);

  glGetIntegerv(GL_VIEWPORT, viewport_original);
  //reduce the display size:
  glViewport(0,0,(GLint)(viewport_original[2]/2),(GLint)(viewport_original[3]/2));
  glGetIntegerv(GL_VIEWPORT, viewport);
  width = viewport[2];
  height= viewport[3];
 
  // save the current display options:
  g3d_save_win_camera(win->vs);
  fov            =  win->vs.fov;
  displayFrame   =  win->vs.displayFrame;
  displayJoints  =  win->vs.displayJoints;
  displayShadows =  win->vs.displayShadows;
  displayWalls   =  win->vs.displayWalls;
  displayFloor   =  win->vs.displayFloor;
  displayTiles   =  win->vs.displayTiles;
  cullingEnabled =  win->vs.cullingEnabled;
  red            =  win->vs.bg[0]; 
  green          =  win->vs.bg[1]; 
  blue           =  win->vs.bg[2]; 
  draw_logo      =  win->vs.enableLogo;

  // only keep what is necessary:
  win->vs.fov            = camera_fov;
  win->vs.displayFrame   = FALSE;
  win->vs.displayJoints  = FALSE;
  win->vs.displayShadows = FALSE;
  win->vs.displayWalls   = FALSE;
  win->vs.displayFloor   = FALSE;
  win->vs.displayTiles   = FALSE;
  win->vs.cullingEnabled=  1;
  win->vs.enableLogo = 0;
  //do not forget to set the backgroung to black:
  g3d_set_win_bgcolor(win->vs, 0, 0, 0);


  // move the camera to the desired pose and apply the new projection matrix:
  g3d_set_camera_parameters_from_frame(camera_frame, win->vs);
  g3d_set_projection_matrix(win->vs.projection_mode);


  ////g3d_draw_win(win);
  g3d_draw_win_back_buffer(win); //only the object should be drawn in red, everthing else is black

  // restore the display options:
  g3d_restore_win_camera(win->vs);
  win->vs.fov            = fov;
  win->vs.displayFrame   = displayFrame;
  win->vs.displayJoints  = displayJoints;
  win->vs.displayShadows = displayShadows;
  win->vs.displayWalls   = displayWalls;
  win->vs.displayFloor   = displayFloor;
  win->vs.displayTiles   = displayTiles;
  win->vs.cullingEnabled =  cullingEnabled;
  win->vs.enableLogo = draw_logo;
  g3d_set_win_bgcolor(win->vs, red, green, blue);
  g3d_set_projection_matrix(win->vs.projection_mode); // do this after restoring the camera fov

  // reset the display modes of everything
  for(i=0; i<XYZ_ENV->no; ++i) {
    p3d_set_obj_display_mode(XYZ_ENV->o[i], P3D_OBJ_DEFAULT_DISPLAY);
  }
  for(i=0; i<XYZ_ENV->nr; ++i) {
    p3d_set_robot_display_mode(XYZ_ENV->robot[i], P3D_ROB_DEFAULT_DISPLAY);
  }

//   static int cnt= 0;
//   char name[256];
//   sprintf(name, "/home/jpsaut/BioMove3Dgit/BioMove3D/screenshots/image%i.ppm", cnt++);
//   g3d_export_OpenGL_display(name);


  // get the OpenGL image buffer:
  image = (unsigned char*) malloc(3*width*height*sizeof(unsigned char));
  glReadBuffer(GL_BACK);  // use back buffer as we are in a double-buffered configuration

  // choose 1-byte alignment:
  glPixelStorei(GL_PACK_ALIGNMENT, 1);

  // get the image pixels (from (0,0) position):
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

  // count the pixels corresponding to the object's color:
  count= 0;
  for(i=0; i<width*height; i++)
  {
    if(image[3*i] > 200) {
      count++;
    }
  }

  *result= ((double) count)/((double) width*height);

  free(image);

  
  glViewport(0,0,(GLint)viewport_original[2],(GLint)viewport_original[3]);

  return 0;
}



int get_ranking_based_on_view_point(p3d_matrix4 view_frame,point_co_ordi point,p3d_rob *object, p3d_rob *human, std::list<gpPlacement> &placementList)
{
  
  double dotX, dotZ, alphaX, alphaZ, visibility;
  p3d_vector3 dir, xAxis, yAxis, zAxis= {0,0,1}, zAxis_view;
  p3d_matrix4 Tcur, T, Topt;
  std::list<gpPlacement>::iterator iplacement;
  GLfloat mat[16];

  dir[0]= view_frame[0][3] - point.x;
  dir[1]= view_frame[1][3] - point.y;
  dir[2]= view_frame[2][3] - point.z;
  p3d_vectNormalize(dir, dir);

  p3d_mat4ExtractColumnZ(view_frame, zAxis_view);

  
  p3d_vectXprod(dir, zAxis, yAxis);
  p3d_vectNormalize(yAxis, yAxis);
  p3d_vectXprod(dir, yAxis, zAxis);

  p3d_mat4Copy(p3d_mat4IDENTITY, Topt);

  Topt[0][0]= dir[0];
  Topt[1][0]= dir[1];
  Topt[2][0]= dir[2];

  Topt[0][1]= -yAxis[0];
  Topt[1][1]= -yAxis[1];
  Topt[2][1]= -yAxis[2];

  Topt[0][2]= -zAxis[0];
  Topt[1][2]= -zAxis[1];
  Topt[2][2]= -zAxis[2];

  p3d_get_freeflyer_pose(object, Tcur);

  for(iplacement=placementList.begin(); iplacement!=placementList.end(); ++iplacement)
  {
    iplacement->position[0]= point.x;
    iplacement->position[1]= point.y;
    iplacement->position[2]= point.z;

    iplacement->polyhedron= object->o[0]->pol[0]->poly;
    iplacement->computePoseMatrix(T);
    p3d_set_freeflyer_pose(object, T);

    if( p3d_col_test_robot(object , 0) )
    {
       iplacement->stability= 0;
    }
    else
    {
       p3d_mat4ExtractColumnX(T, xAxis);
       p3d_mat4ExtractColumnZ(T, zAxis);
//        iplacement->stability= -p3d_vectDotProd(dir, xAxis);
       dotX= p3d_vectDotProd(dir, xAxis);
//        dotZ= zAxis[2];
       dotZ= p3d_vectDotProd(zAxis, zAxis_view);

       alphaX= fabs(acos(dotX));
       alphaZ= fabs(acos(dotZ));

       if( dotX<0  || alphaX > 75*M_PI/180.0 || dotZ<0 || alphaZ > 60*M_PI/180.0 )
       { 
          iplacement->stability= 0;
       }
       else
       {
         ////set_object_at_placement ( object, *iplacement );
         g3d_is_object_visible_from_robot(view_frame, 120, human, object, &visibility);
         
         iplacement->stability= visibility;
         ////////iplacement->stability=0.5;
	 p3d_polyhedre *poly= NULL;
////p3d_rob *horse= p3d_get_robot_by_name("Horse");


// printf(" Drawing object\n");
// 
//  p3d_to_gl_matrix(T, mat);
//  glPushMatrix();
//   glMultMatrixf(mat);
//   g3d_draw_p3d_polyhedre(poly);
//  glPopMatrix();


        }
    }
  }
  ////fl_check_forms();
  ////g3d_draw_allwin_active();
  placementList.sort();
  placementList.reverse();


  p3d_set_freeflyer_pose(object, Tcur);
////////exit(0);
  return 0;
}

////// related to execution of a sequence of tasks //////
int show_desired_HRI_task_plan()
{
 if(SHOW_HRI_TASK_TRAJ_TYPE==0)//Show complete plan
 {
  show_world_state_of_entire_plan(HRI_task_list, SHOW_HRI_PLAN_TYPE);
  return 1;
 }
 
 if(SHOW_HRI_TASK_TRAJ_TYPE==1)//Show plan for selected task
 {
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==CURRENT_HRI_TASK_PLAN_ID_TO_SHOW)
   {
   show_traj_for_this_HRI_task(HRI_task_list[i], SHOW_HRI_PLAN_TYPE);
   return 1;
   }
  }
 }
 
 if(SHOW_HRI_TASK_TRAJ_TYPE==2)//Show plan for selected sub task of selected task
 {
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==CURRENT_HRI_TASK_PLAN_ID_TO_SHOW)
   {
   show_plan_for_this_sub_task(HRI_task_list[i], HRI_task_list[i].traj.sub_task_traj[INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW], INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW, SHOW_HRI_PLAN_TYPE);
   
   return 1;
   }
  }
  
 } 
 
// SHOW_HRI_PLAN_TYPE
// INDEX_CURRENT_HRI_TASK_SUB_PLAN_TO_SHOW
// CURRENT_HRI_TASK_PLAN_ID_TO_SHOW
//  SHOW_HRI_TASK_TRAJ_TYPE
}

int show_traj_for_this_HRI_task(HRI_task_node &for_task, int show_traj)
{
 for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.before_task.robot_config[j]);
   
   }
g3d_draw_allwin_active();
for(int i=0;i<for_task.traj.sub_task_traj.size();i++)
  {

  if(show_traj==1)
   {
  ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=for_task.traj.sub_task_traj[i].traj;
 envPt_MM->robot[indices_of_MA_agents[for_task.traj.sub_task_traj[i].traj_for_agent]]->tcur=for_task.traj.sub_task_traj[i].traj;

  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);

  if(for_task.traj.sub_task_traj[i].sub_task_type==LIFT_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==CARRY_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==PUT_DOWN_OBJECT)
    {
    ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = TRUE;
   envPt_MM->robot[indices_of_MA_agents[for_task.traj.sub_task_traj[i].traj_for_agent]]->isCarryingObject = TRUE;
    }

  
  ////g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);
  g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.traj.sub_task_traj[i].traj_for_agent]],default_drawtraj_fct_ptr);

    if(for_task.traj.sub_task_traj[i].sub_task_type==LIFT_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==CARRY_OBJECT||for_task.traj.sub_task_traj[i].sub_task_type==PUT_DOWN_OBJECT)
    {
    ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
    envPt_MM->robot[indices_of_MA_agents[for_task.traj.sub_task_traj[i].traj_for_agent]]->isCarryingObject = FALSE;
    }
   }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.traj.sub_task_traj[i].config_after_sub_task.robot_config[j]);
  
   }
  g3d_draw_allwin_active();
  }

 for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.after_task.robot_config[j]);
   
   }
g3d_draw_allwin_active();
return 1;
}

int show_traj_for_this_HRI_sub_task(HRI_task_node &for_task,traj_for_HRI_sub_task &sub_task_traj)
{
  ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=sub_task_traj.traj;
  envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->tcur=sub_task_traj.traj;

  if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   {
     ////p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]], sub_task_traj.armID, for_task.hri_task.for_object.c_str());
     p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]], sub_task_traj.armID, for_task.hri_task.for_object.c_str());
     printf(" setting object to carry %s \n",for_task.hri_task.for_object.c_str());
     ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = TRUE;
     envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->isCarryingObject = TRUE;
   }
  
  
  ////g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);
  g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]],default_drawtraj_fct_ptr);

  if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   {
    ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
    envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->isCarryingObject = FALSE;
   }
 
}

int free_final_configs_stored()
{
 for(int i=envPt_MM->nr-1;i>=0;i--)
  {
   if(final_configs_after_a_task.robot_config[i]!=NULL)
   {
    MY_FREE(final_configs_after_a_task.robot_config[i], double, envPt_MM->robot[i]->nb_dof);

   }
   final_configs_after_a_task.robot_config.pop_back();
  }
  
}

//It will modify the envPt_MM->robots to refect the world state as it would be after executing the sub task 
int get_final_configs_for_this_HRI_sub_task_traj(HRI_task_node &for_task, traj_for_HRI_sub_task &sub_task_traj )
{
////for(int i=0;i<for_task.traj.sub_task_traj.size();i++)
 //// {
 
  ////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->tcur=sub_task_traj.traj;//for_task.traj.sub_task_traj[i].traj;

  envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->tcur=sub_task_traj.traj;//for_task.traj.sub_task_traj[i].traj;

  printf(" Sub traj type=%d\n",sub_task_traj.sub_task_type);

  p3d_localpath* lp = NULL;
  for(lp=sub_task_traj.traj->courbePt;lp->next_lp;lp=lp->next_lp)
  {//Go to the last lp of the trajectory
  }
  

//   if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
//    {}
    if(lp->isCarryingObject)
    {
      ////p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]], sub_task_traj.armID, lp->carriedObject[sub_task_traj.armID]->name);
      p3d_set_object_to_carry_to_arm(envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]], sub_task_traj.armID, lp->carriedObject[sub_task_traj.armID]->name);
    }

    final_configs_after_a_task.robot_config.resize(envPt_MM->nr);

    ////final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]= p3d_config_at_param_along_traj(sub_task_traj.traj,sub_task_traj.traj->range_param);
    final_configs_after_a_task.robot_config[indices_of_MA_agents[sub_task_traj.traj_for_agent]]= p3d_config_at_param_along_traj(sub_task_traj.traj,sub_task_traj.traj->range_param);
    
    ////p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]);
    p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]],final_configs_after_a_task.robot_config[indices_of_MA_agents[sub_task_traj.traj_for_agent]]);
    ////if(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject == TRUE)
    if(lp->isCarryingObject)
    {
      p3d_matrix4 objPos;
     ////ArmManipulationData& mData = (*envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->armManipulationData)[sub_task_traj.armID];
      ArmManipulationData& mData = (*envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->armManipulationData)[sub_task_traj.armID];
     p3d_mat4Copy(mData.getManipulationJnt()->abs_pos , objPos);
     ////p3d_set_freeflyer_pose ( envPt_MM->robot[get_index_of_robot_by_name((char*)for_task.hri_task.for_object.c_str())], objPos );
     p3d_set_freeflyer_pose ( envPt_MM->robot[get_index_of_robot_by_name((char*)for_task.hri_task.for_object.c_str())], objPos );
     //////envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = false;
     envPt_MM->robot[indices_of_MA_agents[sub_task_traj.traj_for_agent]]->isCarryingObject = false;
     //final_configs_after_a_task.robot_config[get_index_of_robot_by_name(for_task.for_object.c_str())]=
    }
    ////p3d_set_and_update_this_robot_conf(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],final_configs_after_a_task.robot_config[indices_of_MA_agents[for_task.hri_task.by_agent]]);
    
  /////g3d_show_tcur_rob(envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]],default_drawtraj_fct);

  //if(sub_task_traj.sub_task_type==LIFT_OBJECT||sub_task_traj.sub_task_type==CARRY_OBJECT||sub_task_traj.sub_task_type==PUT_DOWN_OBJECT)
   //{
   // envPt_MM->robot[indices_of_MA_agents[for_task.hri_task.by_agent]]->isCarryingObject = FALSE;
   //}
  
}


int init_tested_cell_info()
{
  
  for(int i=0;i<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;i++)
  {
    for(int j=0;j<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->ny;j++)
    {
      for(int k=0;k<grid_around_HRP2.GRID_SET->bitmap[HRP2_GIK_MANIP]->nx;k++)
      {
      cells_tested_for_current_task[i][j][k]=0;
      }
    }
  }
}

int validate_HRI_task(HRI_task_desc curr_task, int task_plan_id, int for_proactive_info)
{
 HRI_task_node curr_task_to_validate;
 curr_task_to_validate.task_plan_id=task_plan_id;
 curr_task_to_validate.hri_task.task_type=curr_task.task_type;
 curr_task_to_validate.hri_task.for_object=curr_task.for_object;
 printf(">>> Inside validate_HRI_task(), curr_task_to_validate.hri_task.for_object =%s curr_task.for_object = %s\n",curr_task_to_validate.hri_task.for_object.c_str(), curr_task.for_object.c_str());
 curr_task_to_validate.hri_task.by_agent=curr_task.by_agent;
 curr_task_to_validate.hri_task.for_agent=curr_task.for_agent;

 //Tmp for testing
 /*double visibility;
 g3d_is_object_visible_from_viewpoint ( HRI_AGENTS_FOR_MA[curr_task_to_validate.hri_task.for_agent]->perspective->camjoint->abs_pos, HRI_AGENTS_FOR_MA[curr_task_to_validate.hri_task.for_agent]->perspective->fov, p3d_get_robot_by_name(curr_task_to_validate.hri_task.for_object.c_str()), &visibility, 1 );
 printf(" visibility = %lf\n",visibility);
 return 0;
 */
 
 configPt tmp_config;

 for(int i=0;i<envPt_MM->nr;i++)
  {
  tmp_config=MY_ALLOC(double,envPt_MM->robot[i]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[i],&tmp_config);
  
  curr_task_to_validate.before_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
  }

  ////traj_for_HRI_task res_trajs;
  
  ////if(find_current_HRI_manip_task_solution(curr_task, curr_task_to_validate.traj)==0)
  ////if(get_robot_proactive_solution_info( curr_task, curr_task_to_validate.traj)==0)
  ////if(for_proactive_info==1)
  ////{
    
    /*
    HRI_task_agent_effort_level desired_level;
     desired_level.performing_agent=curr_task.by_agent;
  desired_level.target_agent=curr_task.for_agent;
  
  if(curr_task.for_agent==HUMAN1_MA)
    desired_level.effort_for_agent=curr_task.for_agent;
  if(curr_task.by_agent==HUMAN1_MA)
   desired_level.effort_for_agent=curr_task.by_agent;
  
  desired_level.task=curr_task.task_type;
  */
    
    int find_solution_curr_res=0;
    int cur_vis_effort=MA_NO_VIS_EFFORT;
    int cur_reach_effort=MA_NO_REACH_EFFORT;
    int maxi_vis_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
    int maxi_reach_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
    
    init_take_lift_traj_info();
    init_tested_cell_info();
    
    int effort_for=1;// 1 for target agent, 2 for performing agent
    
    //In both cases currently we increase the effort level of human only
    if(for_proactive_info==1)
    {
      effort_for=2; // For the performing agent. Here we assume that the task's performing agent is human and robot has to find the places where the human can perform the task to show proactive behvior
    }
    else
    {
      effort_for=1;//assuming the target agent is human and robot has to find the places where it can perform the task 
    }
    
    
    while(find_solution_curr_res==0&&cur_vis_effort<=maxi_vis_effort_trans_available&&cur_reach_effort<=maxi_reach_effort_trans_available)
    {
 /*
  desired_level.maxi_reach_accept=(MA_transition_reach_effort_type)cur_reach_effort;//MA_ARM_EFFORT;//MA_ARM_EFFORT;//MA_ARM_TORSO_EFFORT;//MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
  desired_level.maxi_vis_accept=(MA_transition_vis_effort_type)cur_vis_effort;//MA_HEAD_EFFORT;//MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
  
  desired_level.vis_relevent=1;
  desired_level.reach_relevant=1;
  
   if(curr_task.task_type==SHOW_OBJECT||curr_task.task_type==HIDE_OBJECT)
    {
    desired_level.reach_relevant=0;
    }
    */
    //Below is tmp to avoid resetting the no_non_accepted_reach_states and vis states in the function called
    //TODO: Adapt the function to include the non acceptable states also for the task of HIDE and PUT_AWAY etc.
    if(curr_task.task_type!=HIDE_OBJECT)
    {
      
      update_effort_levels_for_HRI_Tasks(curr_task, effort_for, cur_reach_effort, cur_vis_effort);
  //set_accepted_effort_level_for_HRI_task(desired_level);
    }
 
  if(for_proactive_info==1)
  {
  find_solution_curr_res=get_robot_proactive_solution_info(curr_task, curr_task_to_validate.traj);
  }
  else
  {
  
    find_solution_curr_res=find_current_HRI_manip_task_solution(curr_task, curr_task_to_validate.traj);
    
//     if(AT_LEAST_1_GRASP_LIFT_FOUND==0)
//     {
//      printf(" >>>> HRI Task Planner FAIL to pick the object \n");  
//      break;
//     }
  }
  
  if(find_solution_curr_res==0)
     {
   printf(" Fail to validate current task  for the effort levels to Reach: %d, to see: %d \n",cur_reach_effort,cur_vis_effort);
   ////return 0;
//   printf("Increasing the effort levels\n");
//   cur_reach_effort++;
//   cur_vis_effort++;
   if(for_proactive_info!=1&&AT_LEAST_1_GRASP_LIFT_FOUND==0)//If no grasp lift found for current task then don't test with higher effort level 
      {
     printf(" >>>> HRI Task Planner FAIL to pick the object \n");  
     break;
      }
     else
      {
        printf("Increasing the effort levels\n");
        cur_reach_effort++;
        cur_vis_effort++;
      }
     }
     else
     {
       printf(" Found solution for current task for the effort levels to Reach: %d, to see: %d \n",cur_reach_effort, cur_vis_effort);
       break;
     }
     
     
     
    }//End while
    
     if(find_solution_curr_res==0)
     {
       printf(" HRI Task Planner Fail to validate current task for all effort level \n");
       return 0;
     }
  ////}
  /*else
  {
   if(find_current_HRI_manip_task_solution(curr_task, curr_task_to_validate.traj)==0)
   {
   printf(" Fail to validate current task \n");
   return 0;
   }
  }*/
  ////curr_task_to_validate.traj=res_trajs;

  printf(" Num of sub task traj for the current task = %d \n", curr_task_to_validate.traj.sub_task_traj.size());

  //To get the world state if the task will be executed
  // p3d_config_at_param_along_traj(..,..);
  int show_traj=0;
  if(show_traj==1)
  {
   show_traj_for_this_HRI_task(curr_task_to_validate, show_traj);
  }
  
  ////p3d_config_at_param_along_traj(curr_task_to_validate.traj.sub_task_traj.[curr_task_to_validate.traj.sub_task_traj.size()-1]
  for(int i=0;i<curr_task_to_validate.traj.sub_task_traj.size();i++)
  {
  ////get_final_configs_for_this_HRI_sub_task_traj(curr_task_to_validate, curr_task_to_validate.traj.sub_task_traj[curr_task_to_validate.traj.sub_task_traj.size()-1]);
  get_final_configs_for_this_HRI_sub_task_traj(curr_task_to_validate, curr_task_to_validate.traj.sub_task_traj[i]);
  for(int j=0;j<envPt_MM->nr;j++)
   {
  
  tmp_config=MY_ALLOC(double,envPt_MM->robot[j]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[j],&tmp_config);
  
  curr_task_to_validate.traj.sub_task_traj[i].config_after_sub_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
   }
  free_final_configs_stored();
  }
                                  
  //Store the last sub task final traj as the final traj of the task itself
  for(int i=0;i<envPt_MM->nr;i++)
  {
  
  tmp_config=MY_ALLOC(double,envPt_MM->robot[i]->nb_dof);
  p3d_get_robot_config_into(envPt_MM->robot[i],&tmp_config);
  
  curr_task_to_validate.after_task.robot_config.push_back(tmp_config);
  ////MY_FREE(tmp_config,double,envPt_MM->robot[i]->nb_dof);//DONOT FREE it because configs are stored 
  }

  HRI_task_list.push_back(curr_task_to_validate);

  char task_plan_id_str[20];
  std::string task_plan_desc;
  sprintf (task_plan_id_str, "%d_",curr_task_to_validate.task_plan_id);
  task_plan_desc=task_plan_id_str;
  task_plan_desc+=envPt_MM->robot[indices_of_MA_agents[curr_task_to_validate.hri_task.by_agent]]->name;
  task_plan_desc+='_';
  task_plan_desc+=HRI_task_NAME_ID_map.find(curr_task_to_validate.hri_task.task_type)->second;
  task_plan_desc+='_';
  ////printf("task_plan_desc before adding obj name = %s \n",task_plan_desc.c_str());
 //// printf(" curr_task.for_object.c_str() = %s\n", curr_task.for_object.c_str());
  task_plan_desc+=curr_task_to_validate.hri_task.for_object.c_str();
 ////  printf("task_plan_desc after adding obj name = %s \n",task_plan_desc.c_str());
  task_plan_desc+='_';
  task_plan_desc+=envPt_MM->robot[indices_of_MA_agents[curr_task_to_validate.hri_task.for_agent]]->name;
  
  HRI_task_plan_DESC_ID_map[task_plan_desc]=curr_task_to_validate.task_plan_id;
  printf(" Inserted into HRI_task_plan_DESC_ID_map key= %s and task_plan_id = %d\n",task_plan_desc.c_str(), curr_task_to_validate.task_plan_id);
  ////show_traj_for_this_HRI_task(curr_task_to_validate[0]);
  printf(" Restoring the actual environment before this task. \n");
  for(int i=0;i<envPt_MM->nr;i++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[i],HRI_task_list[HRI_task_list.size()-1].before_task.robot_config[i]);
   
  }

 ////free_final_configs_stored();
 g3d_draw_allwin_active();
 printf(" returning form validate_HRI_task()\n");
 return 1;
  ////robots_status_for_Mightability_Maps[r_ctr].has_moved
  
}

int show_world_state_of_entire_plan(std::vector<HRI_task_node> &hri_task_list, int exec_path_configs)
{
 for(int i=0;i<hri_task_list.size();i++)
 {
   for(int j=0;j<envPt_MM->nr;j++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].before_task.robot_config[j]);
   
  }
  g3d_draw_allwin_active();
    #if defined(WITH_XFORMS)
  fl_check_forms();
#endif  
  for(int k=0;k<hri_task_list[i].traj.sub_task_traj.size();k++)
  {
 
  
   if(exec_path_configs==1)
    {
    show_traj_for_this_HRI_sub_task(hri_task_list[i],hri_task_list[i].traj.sub_task_traj[k]);
    }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].traj.sub_task_traj[k].config_after_sub_task.robot_config[j]);

   }

  g3d_draw_allwin_active();
    #if defined(WITH_XFORMS)
  fl_check_forms();
#endif  
//  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);
  }

 
  
  for(int j=0;j<envPt_MM->nr;j++)
  {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],hri_task_list[i].after_task.robot_config[j]);
   
  }
g3d_draw_allwin_active();
  #if defined(WITH_XFORMS)

  fl_check_forms();
#endif

 }

}

int show_plan_for_this_sub_task(HRI_task_node &for_task, traj_for_HRI_sub_task &sub_task_traj, int sub_task_index, int show_traj)
{
  if(sub_task_index==0)
  {
  
  for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.before_task.robot_config[j]);
   
   }
  }
  
  if(sub_task_index>0)
  {
   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],for_task.traj.sub_task_traj[sub_task_index-1].config_after_sub_task.robot_config[j]);

   }
  }
 
  g3d_draw_allwin_active();
    #if defined(WITH_XFORMS)

  fl_check_forms();
#endif  
   if(show_traj==1)
    {
    show_traj_for_this_HRI_sub_task(for_task,sub_task_traj);
    }

   for(int j=0;j<envPt_MM->nr;j++)
   {
   ////printf(" Restoring robot %s \n",envPt_MM->robot[i]->name);
   
  p3d_set_and_update_this_robot_conf(envPt_MM->robot[j],sub_task_traj.config_after_sub_task.robot_config[j]);

   }

  g3d_draw_allwin_active();
    #if defined(WITH_XFORMS)

  fl_check_forms();
#endif  
//  printf(" Sub traj type=%d\n",for_task.traj.sub_task_traj[i].sub_task_type);
  

 
 

}

int get_soft_motion_trajectory_for_p3d_traj_vector(std::vector <p3d_traj*> trajs, std::vector <SM_TRAJ> &smTrajs)
{
  ////std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs
    p3d_traj* traj = NULL;
    
   if (manipulation->concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) 
   {
        smTrajs.clear();
//         for(unsigned i = 0; i < trajs.size(); i++){
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
          MANPIPULATION_TRAJECTORY_CONF_STR conf;
          SM_TRAJ smTraj;
          manipulation->computeSoftMotion(traj/*s.at(i)*/, conf, smTraj);
         //// confs.push_back(conf);
          smTrajs.push_back(smTraj);
//         }
    } 
    else 
    {
     printf("Fail to concatanate trajectories \n");
     return 0;
    }	 
    
    return 1;
}

int get_soft_motion_trajectory_for_p3d_traj(p3d_traj* traj, SM_TRAJ &smTraj)
{
    
  //// std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs
         ////smTrajs.clear();
//         for(unsigned i = 0; i < trajs.size(); i++){
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
          MANPIPULATION_TRAJECTORY_CONF_STR conf;
         //// SM_TRAJ smTraj;
          manipulation->computeSoftMotion(traj/*s.at(i)*/, conf, smTraj);
     ////     confs.push_back(conf);
         //// smTrajs.push_back(smTraj);
//         }
	  
	  return 1;
}

int get_single_soft_motion_traj_for_SHARY(int HRI_task_plan_id, int sub_traj_st_index, int sub_traj_end_index, SM_TRAJ &smTraj )
{
  
  p3d_traj* traj;
  ////SM_TRAJ smTraj;
  ////smTrajs.clear();
  std::vector <p3d_traj*> trajs;
  
                     
                     
		     
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==HRI_task_plan_id)
   {
     if(sub_traj_st_index<0||sub_traj_end_index>HRI_task_list[i].traj.sub_task_traj.size())
     {
       printf(" HRI_TASKS ERROR: Sub trajectory index range is not valid\n");
       return 0;
     }
       
     for(int j=sub_traj_st_index;j<=sub_traj_end_index;j++)
     {
       trajs.push_back(HRI_task_list[i].traj.sub_task_traj[j].traj);
      
     }
     
      manipulation->concatTrajectories ( trajs, &traj );
       get_soft_motion_trajectory_for_p3d_traj(traj, smTraj);
       
       ////smTrajs.push_back(smTraj);
  
   
   return 1;
   }
  }
  
  printf(" The task plan ID has not been found\n");
  return 0;
  
}


int get_soft_motion_trajectories_for_plan_ID(int HRI_task_plan_id, std::vector <SM_TRAJ> &smTrajs)
{
  p3d_traj* traj;
  SM_TRAJ smTraj;
  ////smTraj.clear();
  ////smTrajs.clear();
  
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==HRI_task_plan_id)
   {
     for(int j=0;j<HRI_task_list[i].traj.sub_task_traj.size();j++)
     {
       traj=HRI_task_list[i].traj.sub_task_traj[j].traj;
       get_soft_motion_trajectory_for_p3d_traj(traj, smTraj);
       
       smTrajs.push_back(smTraj);
     }
  
   
   return 1;
   }
  }
  
  printf(" The task plan ID has not been found\n");
  return 0;
  
}

int ececute_this_HRI_task_SM_Traj_in_simu(char *for_robot, SM_TRAJ &smTraj)
{
   
   //envPt_MM->robot[get_index_of_robot_by_name(for_robot)]->tcur=;
  
  //g3d_show_tcur_rob(envPt_MM->robot[get_index_of_robot_by_name(for_robot)],default_drawtraj_fct);
  return 1;
}

int ececute_this_HRI_task_p3d_Traj_in_simu(char *for_robot, p3d_traj *traj)
{
   
   envPt_MM->robot[get_index_of_robot_by_name(for_robot)]->tcur=traj;
  
  g3d_show_tcur_rob(envPt_MM->robot[get_index_of_robot_by_name(for_robot)],default_drawtraj_fct_ptr);
}

int show_p3d_trajectories_for_plan_ID(int HRI_task_plan_id)
{
  
  
  for(int i=0;i<HRI_task_list.size();i++)
  {
  if(HRI_task_list[i].task_plan_id==HRI_task_plan_id)
   {
     show_traj_for_this_HRI_task(HRI_task_list[i],1);
  
   
   return 1;
   }
  }
  
  printf(" The task plan ID has not been found\n");
  return 0;
  
}

int get_candidate_points_for_HRI_task(HRI_task_desc curr_task, int is_performing_agent_master, int consider_object_dimension)
{
//TODO: Setting below global variables just for precaution, check if really required or needed by planner or to display
CURRENT_HRI_MANIPULATION_TASK=curr_task.task_type;
CURRENT_TASK_PERFORMED_BY=curr_task.by_agent;
CURRENT_TASK_PERFORMED_FOR=curr_task.for_agent;
strcpy(CURRENT_OBJECT_TO_MANIPULATE,curr_task.for_object.c_str());

printf("<<<< CURRENT_HRI_MANIPULATION_TASK=%d for object = %s, TASK_PERFORMED_BY=%d, TASK_PERFORMED_FOR=%d\n",CURRENT_HRI_MANIPULATION_TASK, CURRENT_OBJECT_TO_MANIPULATE, CURRENT_TASK_PERFORMED_BY, CURRENT_TASK_PERFORMED_FOR);

candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);

 int performing_agent_rank;
printf(" >> is_performing_agent_master=%d\n",is_performing_agent_master);
if(is_performing_agent_master==1)
performing_agent_rank=1;//Master
else
performing_agent_rank=0;//Slave


  find_HRI_task_candidate_points(curr_task.task_type,(char*)curr_task.for_object.c_str(),curr_task.by_agent,curr_task.for_agent,performing_agent_rank,curr_resultant_candidate_points, consider_object_dimension);//It stores the candidate points in a global variable for displaying purpose, so we FREE below
 
  MY_FREE(curr_resultant_candidate_points, candidate_poins_for_task,1);

}

int get_object_list_on_object(char* supporting_obj_name, std::vector<std::string> &ON_object_list)
{
  int support_indx=get_index_of_robot_by_name(supporting_obj_name);
  if(support_indx<0)
  {
    printf(" ERROR : Support name is not a valid 'Robot' \n");
    return -1;
  }
  int nr=envPt_MM->nr;
  
  ON_object_list.clear();
  
  for(int i=0; i<nr; i++)
  {
    int supp_index;
    int is_on_any_support=is_object_laying_on_a_support(i,supp_index);
    if(is_on_any_support==1&&supp_index==support_indx)
    {
      ////printf(" Obj %s is on the support %s \n",envPt_MM->robot[i]->name,envPt_MM->robot[supp_index]->name);
      ON_object_list.push_back(envPt_MM->robot[i]->name);
      
    }
  }
}

int print_this_string_list(std::vector<std::string> &str_list)
{
  ////std::list<std::string>::iterator it;
 //
  ////for(it=str_list.begin();it!=str_list.end();++it)
  for(int i=0;i<str_list.size();i++)
 {
   printf(" %s \n",str_list.at(i).c_str());
 }
  
}

int get_clean_the_table_plan(char *Table_name)
{
  
  std::vector<std::string> ON_object_list;
  std::vector<int> ON_object_indices;
  
  get_object_list_on_object(Table_name, ON_object_list);
  printf(" Objects on the %s are: \n", Table_name);
  print_this_string_list(ON_object_list);
   
  ON_object_indices.clear();
  
  for(int i=0;i<ON_object_list.size();i++)
  {
    int obj_indx= get_index_of_robot_by_name((char*)ON_object_list.at(i).c_str());
    
    if(obj_indx<0)
    {
   printf(" >>> HRI TASK Planner WARNING: Object %s could not be found, so will be ignored \n",ON_object_list.at(i).c_str());
    }
    else
    {
      ON_object_indices.push_back(obj_indx);
    }
  }
  
  for(int i=0;i<ON_object_list.size();i++)
  {
  int cur_obj_indx=ON_object_indices.at(i);
  
  printf(" Testing for object %s \n",envPt_MM->robot[cur_obj_indx]->name);
  
  int i;
#ifdef PR2_EXISTS_FOR_MA
  i=PR2_MA;
#endif

#ifdef JIDO_EXISTS_FOR_MA
  i=JIDO_MA;
#endif
  
  
  ////int nr_ctr=1;
  for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_vis_states;j++)
		{
		    if(object_MM.object[cur_obj_indx].geo_MM.visible[i][j]>0)
		    {
		      //printf(" Drawing disc\n"); 
		      printf(" It is visible by %d state of %s \n", j, envPt_MM->robot[indices_of_MA_agents[i]]->name);
		      
		    }
		 
		}
		
	      for(int j=0;j<agents_for_MA_obj.for_agent[i].maxi_num_reach_states;j++)
		{
		for(int k=0;k<agents_for_MA_obj.for_agent[i].no_of_arms;k++)
		  {
		  
		     if(object_MM.object[cur_obj_indx].geo_MM.reachable[i][j][k]>0)
		    {
		      //printf(" Drawing disc\n"); 
		      ////g3d_drawDisc(cell_x_world, cell_y_world, cell_z_world, radius, curr_flags_show_Mightability_Maps.show_reachability[i][j][k], NULL);
		      printf(" It is reachable by %d state of %s \n", j, envPt_MM->robot[indices_of_MA_agents[i]]->name);
		      break;
		   
		    }
		   
		  }
		}
  }
}

int find_agent_object_affordance(HRI_task_desc curr_task, int obj_index, taskability_node &res_node )
{
  int task=curr_task.task_type;
  ////int obj_index=get_index_of_robot_by_name((char*)curr_task.for_object.c_str());
  int performing_agent=curr_task.by_agent;
  int agent_posture;
  int agent_is_human=0;
  int agent_supported=0;
  
  if(performing_agent==HUMAN1_MA)
  {
    agent_posture=HUMAN1_CURRENT_STATE_MM;
    agent_is_human=1;
    agent_supported=1;
  }
#ifdef HUMAN2_EXISTS_FOR_MA
  if(performing_agent==HUMAN2_MA)
  {
    agent_posture=HUMAN2_CURRENT_STATE_MM;
    agent_is_human=1;
        agent_supported=1;

  }
#endif
 
#ifdef PR2_EXISTS_FOR_MA
  if(performing_agent==PR2_MA)
  {
    agent_posture=PR2_ARBITRARY_MA;
    agent_is_human=0;
        agent_supported=1;

  }
#endif

  
  int agent_cur_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  int agent_maxi_allowed_effort[MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
  
  agent_cur_effort[VIS_ABILITY]=MA_NO_VIS_EFFORT;
  agent_cur_effort[REACH_ABILITY]=MA_NO_REACH_EFFORT;
  
  int per_ag_hum=0;
  
  if(performing_agent==HUMAN1_MA)
  {
    per_ag_hum=1;
  }
  #ifdef HUMAN2_EXISTS_FOR_MA
  {
    if(performing_agent==HUMAN2_MA)
    {
      per_ag_hum=1;
    }
  }
  #endif
  
  if(per_ag_hum==1)
  {
    agent_maxi_allowed_effort[VIS_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
    agent_maxi_allowed_effort[REACH_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
  }
  
  #ifdef PR2_EXISTS_FOR_MA
  if(performing_agent==PR2_MA)
  {
    agent_maxi_allowed_effort[VIS_ABILITY]=MA_HEAD_EFFORT;
    agent_maxi_allowed_effort[REACH_ABILITY]=MA_ARM_EFFORT;
  }
  
  #endif
  
  
  
  if(task==TAKE_OBJECT)
  { 
    
    #ifdef JIDO_EXISTS_FOR_MA

  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[JIDO_MA]]->name);

#endif

#ifdef PR2_EXISTS_FOR_MA
 
  init_manipulation_planner(envPt_MM->robot[indices_of_MA_agents[PR2_MA]]->name);

#endif
  
    char by_hand[50];
    #ifdef JIDO_EXISTS_FOR_MA
    if(performing_agent==JIDO_MA)
    {
      if(JIDO_HAND_TYPE==1)
      {
	strcpy(by_hand,"JIDO_GRIPPER");
      }
      if(JIDO_HAND_TYPE==2)
      {
	strcpy(by_hand,"SAHandRight");
      }
    }
    #endif
    
    #ifdef PR2_EXISTS_FOR_MA
    if(performing_agent==PR2_MA)
    {
      strcpy(by_hand,"PR2_GRIPPER");
    }
    #endif
    
    if(per_ag_hum==1)
    {
      ////strcpy(by_hand,"SAHandRight2");
      strcpy(by_hand,"PR2_GRIPPER");//NOTE: TMP using gripper for human as SAHandRight2 is not working
    }
    
    std::list<gpGrasp> grasps_for_object;
    
    get_grasp_list_for_object(envPt_MM->robot[obj_index]->name, grasps_for_object);
    
    p3d_rob* hand_rob= ( p3d_rob* ) p3d_get_robot_by_name ( by_hand );
    printf(" Total no. of initail grasps for agent %s = %d\n", envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name, grasps_for_object.size());
    int ctr=0;
    
     p3d_col_deactivate_rob_rob(hand_rob, envPt_MM->robot[obj_index]);
   for ( std::list<gpGrasp>::iterator igrasp=grasps_for_object.begin(); igrasp!=grasps_for_object.end(); ++igrasp )
   {
    ////printf(" Setting grasp %d\n",ctr); 
    gpSet_robot_hand_grasp_configuration(hand_rob, envPt_MM->robot[obj_index], *igrasp);
    
   
     ////p3d_col_deactivate_robot(hand_rob);
   
	  int kcd_with_report=0;
      int res = p3d_col_test_robot ( hand_rob,kcd_with_report );
     if ( res>0 ) ////if(p3d_col_test_robot(bitmapset->visball,FALSE))
    {
    ////printf(" Collision \n");
     ////g3d_draw_allwin_active();
    }
    else
    {
    ctr++;
    }
   }
   p3d_col_activate_rob_rob(hand_rob, envPt_MM->robot[obj_index]);
   printf(" Total no. of non collision grasps = %d\n",ctr);
   
   if(ctr<=0)
   {
     printf(" No valid grasp... \n");
     return 0;
   }
   
   int sol_found=0;
   
   while(sol_found==0)
   {
     int at_least_one_analysis_to_test=0;
     
     update_analysis_type_effort_level_group(performing_agent, agent_posture);
     
    ////update_effort_levels_for_HRI_Tasks(curr_task, 2, agent_cur_effort[REACH_ABILITY], agent_cur_effort[VIS_ABILITY]);
 
    int is_visible=0;
    
   for(int i=0;i<Analysis_type_Effort_level[performing_agent][VIS_ABILITY][agent_cur_effort[VIS_ABILITY]].num_analysis_types;i++)
   {
     int curr_analysis_state=Analysis_type_Effort_level[performing_agent][VIS_ABILITY][agent_cur_effort[VIS_ABILITY]].analysis_types[i];
     
     if(object_MM.object[obj_index].geo_MM.visible[performing_agent][i]>0)
     {
	
	printf(" Object is visible by %d state of %s \n", i, envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name);
	is_visible=1;
	break;	      
     }
   }
   
   if(is_visible==0)
   {
     printf(" Object is NOT visible by agent %s \n", envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name);
     
     if(agent_cur_effort[VIS_ABILITY]<agent_maxi_allowed_effort[VIS_ABILITY])
     {
       agent_cur_effort[VIS_ABILITY]++;
       at_least_one_analysis_to_test=1;
     }
   }
   
   int is_reachable=0;
    
   for(int i=0;i<Analysis_type_Effort_level[performing_agent][REACH_ABILITY][agent_cur_effort[REACH_ABILITY]].num_analysis_types;i++)
   {
     int curr_analysis_state=Analysis_type_Effort_level[performing_agent][REACH_ABILITY][agent_cur_effort[REACH_ABILITY]].analysis_types[i];
    
    for(int k=0;k<agents_for_MA_obj.for_agent[performing_agent].no_of_arms;k++)
    {
     if(object_MM.object[obj_index].geo_MM.reachable[performing_agent][i][k]>0)
     {
	
	printf(" Object is reachable by %d state of %s \n", i, envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name);
	is_reachable=1;
	break;	      
     }
    }
   }
   
   if(is_reachable==0)
   {
     printf(" Object is NOT reachable by agent %s \n", envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name);
     if(agent_cur_effort[REACH_ABILITY]<agent_maxi_allowed_effort[REACH_ABILITY])
     {
       agent_cur_effort[REACH_ABILITY]++;
       at_least_one_analysis_to_test=1;
     }
    }
   
   if(is_reachable==1&&is_visible==1)
    {
      printf(" The agent %s could take the object %s with effort levels for vis= %d and for reach= %d \n", envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name, envPt_MM->robot[obj_index]->name, agent_cur_effort[VIS_ABILITY], agent_cur_effort[REACH_ABILITY]);
      
      res_node.task=curr_task.task_type;
      res_node.performing_agent=performing_agent;
      res_node.target_object=obj_index;
      res_node.performing_ag_effort[VIS_ABILITY]=agent_cur_effort[VIS_ABILITY];
      res_node.performing_ag_effort[REACH_ABILITY]=agent_cur_effort[REACH_ABILITY];
      
      sol_found=1;
      return 1;
    }
    
    if(at_least_one_analysis_to_test==0)
    {
      printf(" The agent %s could not take the object %s with its maximum allowed effort level \n", envPt_MM->robot[indices_of_MA_agents[performing_agent]]->name, envPt_MM->robot[obj_index]->name);
      
      return 0;
    }
   }
  }//End if(task==TAKE_OBJECT)
  
  
}


int init_grasp_exists_for_object()
{
  printf(" ====== LIST of Valid graspable and manipulable Objects ======\n");
  int ctr=0;
  for(int j=0;j<envPt_MM->nr;j++)
    {
      for(int k=0;k<NUM_VALID_GRASPABLE_OBJ;k++)
      {
	GRASP_EXISTS_FOR_OBJECT[j]=0;
      if(strcasestr(envPt_MM->robot[j]->name,MANIPULABLE_OBJECTS[k]))
       {
       GRASP_EXISTS_FOR_OBJECT[j]=1;
       printf(" * %d : %s \n",ctr, envPt_MM->robot[j]->name);
      ctr++;
       break;
       }
      }
    }
    printf(" >>**> MA - HRI Task Planner WARNING : Cross verify if any manipulable object is missing. Put it in the HRI_task.cpp file in MANIPULABLE_OBJECTS[] list and change in the value of NUM_VALID_GRASPABLE_OBJ.\n");
    
}
  
std::vector<taskability_node> manipulability_graph;
std::map<std::string, int > manipulability_node_DESC_ID_map;
  
int find_manipulability_graph()
{
  manipulability_graph.clear();
  manipulability_node_DESC_ID_map.clear();
  
  HRI_task_desc curr_task;
  curr_task.task_type=TAKE_OBJECT;
  
  int ctr=0;
  
  for(int i=0; i<MAXI_NUM_OF_AGENT_FOR_HRI_TASK;i++)
  {
    for(int j=0;j<envPt_MM->nr;j++)
    {
      if(GRASP_EXISTS_FOR_OBJECT[j]==1)
      {
	curr_task.by_agent=(HRI_TASK_AGENT)i;
	curr_task.for_object=envPt_MM->robot[j]->name;
       taskability_node res_node;
       int res=find_agent_object_affordance(curr_task, j, res_node );
       if(res==1)
       {
	 char manipulability_node_id_str[20];
  std::string manipulability_node_desc;
  sprintf (manipulability_node_id_str, "%d_",ctr);
  manipulability_node_desc=manipulability_node_id_str;
  manipulability_node_desc+=envPt_MM->robot[indices_of_MA_agents[res_node.performing_agent]]->name;
  manipulability_node_desc+='_';
  manipulability_node_desc+=HRI_task_NAME_ID_map.find(curr_task.task_type)->second;
  manipulability_node_desc+='_';
  
  manipulability_node_desc+=envPt_MM->robot[res_node.target_object]->name;
  
  manipulability_node_DESC_ID_map[manipulability_node_desc]=ctr;
  printf(" Inserted into manipulability_node_DESC_ID_map key= %s \n",manipulability_node_desc.c_str());
  
  strcpy(res_node.desc,manipulability_node_desc.c_str());
  
  res_node.node_id=ctr;
  
  manipulability_graph.push_back(res_node);
  
  ctr++;
  
       }
      }
    }
  }
}
/////// Functions related to taskability graph //////////
int find_taskability_link_between_two_agents_for_task(HRI_task_desc curr_task, taskability_node &res_node )
{
  int performing_agent=curr_task.by_agent;
  int target_agent=curr_task.for_agent;
  int task=curr_task.task_type;
 
  int per_ag_hum=0;
  int tar_ag_hum=0;
  if(performing_agent==HUMAN1_MA)
  {
     per_ag_hum=1;
  }
#ifdef HUMAN2_EXISTS_FOR_MA
  {
    if(performing_agent==HUMAN2_MA)
   {
     per_ag_hum=1;
   }
  }
#endif

if(target_agent==HUMAN1_MA)
  {
     tar_ag_hum=1;
  }
#ifdef HUMAN2_EXISTS_FOR_MA
  {
    if(target_agent==HUMAN2_MA)
   {
     tar_ag_hum=1;
   }
  }
#endif

  candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);

  curr_resultant_candidate_points->no_points=0;
  
int performing_agent_rank=0;//Slave
int consider_object_dimension=0;

int find_solution_curr_res=0;

int agent_cur_effort[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
int agent_maxi_allowed_effort[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
/*
    int performing_ag_cur_vis_effort=MA_NO_VIS_EFFORT;
    int performing_ag_cur_reach_effort=MA_NO_REACH_EFFORT;
    int target_ag_cur_vis_effort=MA_NO_VIS_EFFORT;
    int target_ag_cur_reach_effort=MA_NO_REACH_EFFORT;
    int maxi_vis_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
    int maxi_reach_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
   */
agent_cur_effort[performing_agent][VIS_ABILITY]=MA_NO_VIS_EFFORT;
agent_cur_effort[performing_agent][REACH_ABILITY]=MA_NO_REACH_EFFORT;
agent_cur_effort[target_agent][VIS_ABILITY]=MA_NO_VIS_EFFORT;
agent_cur_effort[target_agent][REACH_ABILITY]=MA_NO_REACH_EFFORT;

if(per_ag_hum==1)
{
  agent_maxi_allowed_effort[performing_agent][VIS_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
  agent_maxi_allowed_effort[performing_agent][REACH_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
}
if(tar_ag_hum==1)
{
  agent_maxi_allowed_effort[target_agent][VIS_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
  agent_maxi_allowed_effort[target_agent][REACH_ABILITY]=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
}

#ifdef PR2_EXISTS_FOR_MA
    if(performing_agent==PR2_MA)
    {
       agent_maxi_allowed_effort[performing_agent][VIS_ABILITY]=MA_HEAD_EFFORT;
       agent_maxi_allowed_effort[performing_agent][REACH_ABILITY]=MA_ARM_EFFORT;
    }
    
     if(target_agent==PR2_MA)
    {
       agent_maxi_allowed_effort[target_agent][VIS_ABILITY]=MA_HEAD_EFFORT;
       agent_maxi_allowed_effort[target_agent][REACH_ABILITY]=MA_ARM_EFFORT;
    }
#endif
   
    int set_effort_for=2;// 1 for target agent, 2 for performing agent
    
    int sol_found=0;
    
    update_effort_levels_for_HRI_Tasks(curr_task, 2, agent_cur_effort[performing_agent][REACH_ABILITY], agent_cur_effort[performing_agent][VIS_ABILITY]);
     
    //Below is tmp to avoid resetting the no_non_accepted_reach_states and vis states in the function called
    //TODO: Adapt the function to include the non acceptable states also for the task of HIDE and PUT_AWAY etc.
    if(task!=HIDE_OBJECT)
    {
     
      update_effort_levels_for_HRI_Tasks(curr_task, 1, agent_cur_effort[target_agent][REACH_ABILITY], agent_cur_effort[target_agent][VIS_ABILITY]);
      
  //set_accepted_effort_level_for_HRI_task(desired_level);
    }
    
    
int res=find_HRI_task_candidate_points((HRI_TASK_TYPE_ENUM)task,  NULL, (HRI_TASK_AGENT_ENUM)performing_agent, (HRI_TASK_AGENT_ENUM)target_agent,performing_agent_rank, curr_resultant_candidate_points,  consider_object_dimension);

    if(curr_resultant_candidate_points->no_points<=0)
     {
     printf(" >>> Fail to find candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", agent_cur_effort[performing_agent][VIS_ABILITY], agent_cur_effort[performing_agent][REACH_ABILITY], performing_agent, agent_cur_effort[target_agent][VIS_ABILITY], agent_cur_effort[target_agent][REACH_ABILITY], target_agent);
    
    
     }
     else
     {
         printf(" >>>** Found candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", agent_cur_effort[performing_agent][VIS_ABILITY], agent_cur_effort[performing_agent][REACH_ABILITY], performing_agent, agent_cur_effort[target_agent][VIS_ABILITY], agent_cur_effort[target_agent][REACH_ABILITY], target_agent);
    
	 
       sol_found=1;
       printf(" curr_resultant_candidate_points->no_points=%d\n",curr_resultant_candidate_points->no_points);
     }

    //int performing_ag_maxi_effort_reached=0;
     //int target_ag_maxi_effort_reached=0;
     int agent_maxi_effort_reached[MAXI_NUM_OF_AGENT_FOR_HRI_TASK][MAXI_NUM_ABILITY_TYPE_FOR_EFFORT];
     
     for(int i=0;i<MAXI_NUM_ABILITY_TYPE_FOR_EFFORT;i++)
     {
       agent_maxi_effort_reached[performing_agent][i]=0;
       agent_maxi_effort_reached[target_agent][i]=0;
     }
    /* if(per_ag_hum==0)
     {
     performing_ag_maxi_effort_reached=1;//Assuming that for non human already maximum effort has been set
       
     }
     
     if(tar_ag_hum==0)
     {
     target_ag_maxi_effort_reached=1;//Assuming that for non human already maximum effort has been set
       
     }*/
     
    while(sol_found==0)
    {
      printf(" Inside while(sol_found==0)\n");
      int effort_increased=0;
      if(set_effort_for==2)
      {
	
       if(agent_cur_effort[performing_agent][VIS_ABILITY]<agent_maxi_allowed_effort[performing_agent][VIS_ABILITY])
       {
	 printf(" Increasing vis effort level of performing agent\n");
	agent_cur_effort[performing_agent][VIS_ABILITY]++;
	effort_increased=1;
       }
       else
       {
       agent_maxi_effort_reached[performing_agent][VIS_ABILITY]=1; 
       }
	
      if(agent_cur_effort[performing_agent][REACH_ABILITY]<agent_maxi_allowed_effort[performing_agent][REACH_ABILITY])
       {
	 printf(" Increasing Reach effort level of performing agent\n");
	agent_cur_effort[performing_agent][REACH_ABILITY]++;
	effort_increased=1;
       }
       else
       {
       agent_maxi_effort_reached[performing_agent][REACH_ABILITY]=1; 
       }
	
       set_effort_for=1;
      }
     else
      {
     if(set_effort_for==1)
       {
	  if(agent_cur_effort[target_agent][VIS_ABILITY]<agent_maxi_allowed_effort[target_agent][VIS_ABILITY])
       {
	 printf(" Increasing vis effort level of target_agent\n");
	agent_cur_effort[target_agent][VIS_ABILITY]++;
	effort_increased=1;
       }
       else
       {
       agent_maxi_effort_reached[target_agent][VIS_ABILITY]=1; 
       }
	
      if(agent_cur_effort[target_agent][REACH_ABILITY]<agent_maxi_allowed_effort[target_agent][REACH_ABILITY])
       {
	 printf(" Increasing Reach effort level of target_agent\n");
	agent_cur_effort[target_agent][REACH_ABILITY]++;
	effort_increased=1;
       }
       else
       {
       agent_maxi_effort_reached[target_agent][REACH_ABILITY]=1; 
       }
	
       set_effort_for=2;
       }
     }
      
     if(agent_maxi_effort_reached[performing_agent][VIS_ABILITY]==1&&agent_maxi_effort_reached[performing_agent][REACH_ABILITY]==1&&agent_maxi_effort_reached[target_agent][VIS_ABILITY]==1&&agent_maxi_effort_reached[target_agent][REACH_ABILITY]==1)
     {
       printf(" Maximum effort levels of both the agents have been tested so, stopping further tests\n");
       break;
     }
     
     if(effort_increased==0)
     {
       continue;
     }
     
      update_effort_levels_for_HRI_Tasks(curr_task, 2, agent_cur_effort[performing_agent][REACH_ABILITY], agent_cur_effort[performing_agent][VIS_ABILITY]);
    
	  //Below is tmp to avoid resetting the no_non_accepted_reach_states and vis states in the function called
    //TODO: Adapt the function to include the non acceptable states also for the task of HIDE and PUT_AWAY etc.
      if(task!=HIDE_OBJECT)
       {
      update_effort_levels_for_HRI_Tasks(curr_task, 1, agent_cur_effort[target_agent][REACH_ABILITY], agent_cur_effort[target_agent][VIS_ABILITY]);
     
  //set_accepted_effort_level_for_HRI_task(desired_level);
       }
	
	res=find_HRI_task_candidate_points((HRI_TASK_TYPE_ENUM)task,   NULL, (HRI_TASK_AGENT_ENUM)performing_agent, (HRI_TASK_AGENT_ENUM)target_agent,performing_agent_rank, curr_resultant_candidate_points, consider_object_dimension);

    if(curr_resultant_candidate_points->no_points<=0)
     {
     printf(" >>> Fail to find candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", agent_cur_effort[performing_agent][VIS_ABILITY], agent_cur_effort[performing_agent][REACH_ABILITY], performing_agent, agent_cur_effort[target_agent][VIS_ABILITY], agent_cur_effort[target_agent][REACH_ABILITY], target_agent);
    
    
     }
     else
     {
         printf(" >>>** Found candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", agent_cur_effort[performing_agent][VIS_ABILITY], agent_cur_effort[performing_agent][REACH_ABILITY], performing_agent, agent_cur_effort[target_agent][VIS_ABILITY], agent_cur_effort[target_agent][REACH_ABILITY], target_agent);
    
	 
       sol_found=1;
       printf(" curr_resultant_candidate_points->no_points=%d\n",curr_resultant_candidate_points->no_points);
     }
      
      
      
     
    }// END while(sol_found==0)

 if(sol_found==0)
   {
     printf(" Fail to find the solution \n");
     return 0;
     
   }
   
  if(sol_found==1)
   {
     printf(" Found the solution \n");
     res_node.performing_agent=performing_agent;
     res_node.target_agent=target_agent;
     res_node.task=task;
     res_node.performing_ag_effort[REACH_ABILITY]=agent_cur_effort[performing_agent][REACH_ABILITY];
     res_node.performing_ag_effort[VIS_ABILITY]=agent_cur_effort[performing_agent][VIS_ABILITY];
     res_node.target_ag_effort[REACH_ABILITY]=agent_cur_effort[target_agent][REACH_ABILITY];
     res_node.target_ag_effort[VIS_ABILITY]=agent_cur_effort[target_agent][VIS_ABILITY];
     res_node.candidate_points=curr_resultant_candidate_points;
     
     return 1;
     
   }
   
}

int find_taskability_link_between_two_agents_for_task_old(HRI_task_desc curr_task, taskability_node &res_node )
{
  int performing_agent=curr_task.by_agent;
  int target_agent=curr_task.for_agent;
  int task=curr_task.task_type;
 
  int per_ag_hum=0;
  int tar_ag_hum=0;
  if(performing_agent==HUMAN1_MA)
  {
     per_ag_hum=1;
  }
#ifdef HUMAN2_EXISTS_FOR_MA
  {
    if(performing_agent==HUMAN2_MA)
   {
     per_ag_hum=1;
   }
  }
#endif

if(target_agent==HUMAN1_MA)
  {
     tar_ag_hum=1;
  }
#ifdef HUMAN2_EXISTS_FOR_MA
  {
    if(target_agent==HUMAN2_MA)
   {
     tar_ag_hum=1;
   }
  }
#endif

  candidate_poins_for_task *curr_resultant_candidate_points=MY_ALLOC(candidate_poins_for_task,1);

  curr_resultant_candidate_points->no_points=0;
  
int performing_agent_rank=0;//Slave
int consider_object_dimension=0;

int find_solution_curr_res=0;
    int performing_ag_cur_vis_effort=MA_NO_VIS_EFFORT;
    int performing_ag_cur_reach_effort=MA_NO_REACH_EFFORT;
    int target_ag_cur_vis_effort=MA_NO_VIS_EFFORT;
    int target_ag_cur_reach_effort=MA_NO_REACH_EFFORT;
    int maxi_vis_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_VIS;
    int maxi_reach_effort_trans_available=MA_WHOLE_BODY_CURR_POS_EFFORT_REACH;
    
   
    int set_effort_for=2;// 1 for target agent, 2 for performing agent
    
    int sol_found=0;
    
    
    //Below is tmp to avoid resetting the no_non_accepted_reach_states and vis states in the function called
    //TODO: Adapt the function to include the non acceptable states also for the task of HIDE and PUT_AWAY etc.
    if(task!=HIDE_OBJECT)
    {
      if(tar_ag_hum==1)
      {
      update_effort_levels_for_HRI_Tasks(curr_task, 1, target_ag_cur_reach_effort, target_ag_cur_vis_effort);
      }
      if(per_ag_hum==1)
      {
      update_effort_levels_for_HRI_Tasks(curr_task, 2, performing_ag_cur_reach_effort, performing_ag_cur_vis_effort);
      }
  //set_accepted_effort_level_for_HRI_task(desired_level);
    }
    
    
int res=find_HRI_task_candidate_points((HRI_TASK_TYPE_ENUM)task,  NULL, (HRI_TASK_AGENT_ENUM)performing_agent, (HRI_TASK_AGENT_ENUM)target_agent,performing_agent_rank, curr_resultant_candidate_points,  consider_object_dimension);

    if(curr_resultant_candidate_points->no_points<=0)
     {
     printf(" >>> Fail to find candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", performing_ag_cur_vis_effort, performing_ag_cur_reach_effort, performing_agent, target_ag_cur_vis_effort, target_ag_cur_reach_effort, target_agent);
    
    
     }
     else
     {
        printf(" >>>** Found candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", performing_ag_cur_vis_effort, performing_ag_cur_reach_effort, performing_agent, target_ag_cur_vis_effort, target_ag_cur_reach_effort, target_agent);
    
       sol_found=1;
       printf(" curr_resultant_candidate_points->no_points=%d\n",curr_resultant_candidate_points->no_points);
     }

    int performing_ag_maxi_effort_reached=0;
     int target_ag_maxi_effort_reached=0;
     
     if(per_ag_hum==0)
     {
     performing_ag_maxi_effort_reached=1;//Assuming that for non human already maximum effort has been set
       
     }
     
     if(tar_ag_hum==0)
     {
     target_ag_maxi_effort_reached=1;//Assuming that for non human already maximum effort has been set
       
     }
     
    while(sol_found==0)
    {
      printf(" Inside while(sol_found==0)\n");
      int effort_increased=0;
      if(set_effort_for==2)
      {
	if(per_ag_hum==1)
	{
       if(performing_ag_cur_vis_effort<maxi_vis_effort_trans_available&&performing_ag_cur_reach_effort<maxi_reach_effort_trans_available)
       {
	 printf(" Increasing effort level of performing agent\n");
	performing_ag_cur_vis_effort++;
        performing_ag_cur_reach_effort++;
	effort_increased=1;
       }
       else
       {
       performing_ag_maxi_effort_reached=1; 
       }
	}
       set_effort_for=1;
      }
      else
      {
     if(set_effort_for==1)
       {
	 if(tar_ag_hum==1)
	 {
       if(target_ag_cur_vis_effort<maxi_vis_effort_trans_available&&target_ag_cur_reach_effort<maxi_reach_effort_trans_available)
        {
	 printf(" Increasing effort level of target agent\n");
	target_ag_cur_vis_effort++;
       target_ag_cur_reach_effort++;
     effort_increased=1;
        }
       else
        {
       target_ag_maxi_effort_reached=1; 
        }
	 }
       set_effort_for=2;
       }
      }
      
     if(performing_ag_maxi_effort_reached==1&&target_ag_maxi_effort_reached==1)
     {
       printf(" Maximum effort levels of both the agents have been tested so, stopping further tests\n");
       break;
     }
     
     if(effort_increased==0)
     {
       continue;
     }
	  //Below is tmp to avoid resetting the no_non_accepted_reach_states and vis states in the function called
    //TODO: Adapt the function to include the non acceptable states also for the task of HIDE and PUT_AWAY etc.
      if(task!=HIDE_OBJECT)
       {
      if(tar_ag_hum==1)
      {
      update_effort_levels_for_HRI_Tasks(curr_task, 1, target_ag_cur_reach_effort, target_ag_cur_vis_effort);
      }
      if(per_ag_hum==1)
      {
      update_effort_levels_for_HRI_Tasks(curr_task, 2, performing_ag_cur_reach_effort, performing_ag_cur_vis_effort);
      }
    
  //set_accepted_effort_level_for_HRI_task(desired_level);
       }
	
	res=find_HRI_task_candidate_points((HRI_TASK_TYPE_ENUM)task,   NULL, (HRI_TASK_AGENT_ENUM)performing_agent, (HRI_TASK_AGENT_ENUM)target_agent,performing_agent_rank, curr_resultant_candidate_points, consider_object_dimension);

    if(curr_resultant_candidate_points->no_points<=0)
     {
     printf(" >>> Fail to find candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", performing_ag_cur_vis_effort, performing_ag_cur_reach_effort, performing_agent, target_ag_cur_vis_effort, target_ag_cur_reach_effort, target_agent);
    
    
     }
     else
     {
        printf(" >>>** Found candidate points for effort levels vis:%d, reach: %d, for agent %d and for effort levels vis:%d, reach: %d, for agent %d\n", performing_ag_cur_vis_effort, performing_ag_cur_reach_effort, performing_agent, target_ag_cur_vis_effort, target_ag_cur_reach_effort, target_agent);
    
       sol_found=1;
       printf(" curr_resultant_candidate_points->no_points=%d\n",curr_resultant_candidate_points->no_points);
     }
      
      
      
     
    }// END while(sol_found==0)

 if(sol_found==0)
   {
     printf(" Fail to find the solution \n");
     return 0;
     
   }
   
  if(sol_found==1)
   {
     printf(" Found the solution \n");
     res_node.performing_agent=performing_agent;
     res_node.target_agent=target_agent;
     res_node.task=task;
     res_node.performing_ag_effort[REACH_ABILITY]=performing_ag_cur_reach_effort;
     res_node.performing_ag_effort[VIS_ABILITY]=performing_ag_cur_vis_effort;
     res_node.target_ag_effort[REACH_ABILITY]=target_ag_cur_reach_effort;
     res_node.target_ag_effort[VIS_ABILITY]=target_ag_cur_vis_effort;
     res_node.candidate_points=curr_resultant_candidate_points;
     
     return 1;
     
   }
   
}

std::vector<taskability_node> taskability_graph;
std::map<std::string, int > taskability_node_DESC_ID_map;

int find_taskability_graph()
{
  taskability_node_DESC_ID_map.clear();
HRI_task_desc curr_task;
std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=taskability_graph.begin();it!=taskability_graph.end();it++)
 {
   printf(" freeing memory for candidate points for %d th node of taskability graph\n", ctr);
   MY_FREE(it->candidate_points, candidate_poins_for_task,1);
   ctr++;
 }
 
taskability_graph.clear();
 
ctr=0;
  for(int i=0; i< MAXI_NUM_OF_AGENT_FOR_HRI_TASK; i++)
  {
    for(int j=0; j< MAXI_NUM_OF_AGENT_FOR_HRI_TASK; j++)
    {
      if(i==j)
      {
	continue;
      }
      
      curr_task.for_object='\0';
      curr_task.by_agent=(HRI_TASK_AGENT)i;
      curr_task.for_agent=(HRI_TASK_AGENT)j;
      
      for(int k=0; k<MAXI_NUM_OF_HRI_TASKS; k++)
      {
	if(k==SHOW_OBJECT||k==HIDE_OBJECT||k==MAKE_OBJECT_ACCESSIBLE||k==GIVE_OBJECT)// NOTE: CURRENTLY these tasks are fully implemented
	{
	curr_task.task_type=(HRI_TASK_TYPE)k;
	taskability_node res_node;
	int res=find_taskability_link_between_two_agents_for_task(curr_task, res_node);
	if(res==1)
	 {
	  printf(">>> Adding to the taskability graph, for performing agent %d, for target agent %d, for task %d, no_candidate_poins %d\n",res_node.performing_agent, res_node.target_agent, res_node.task, res_node.candidate_points->no_points);
	  
	  
	  
  char taskability_node_id_str[20];
  std::string taskability_node_desc;
  sprintf (taskability_node_id_str, "%d_",ctr);
  taskability_node_desc=taskability_node_id_str;
  taskability_node_desc+=envPt_MM->robot[indices_of_MA_agents[res_node.performing_agent]]->name;
  taskability_node_desc+='_';
  taskability_node_desc+=HRI_task_NAME_ID_map.find(curr_task.task_type)->second;
  taskability_node_desc+='_';
  ////printf("task_plan_desc before adding obj name = %s \n",task_plan_desc.c_str());
 //// printf(" curr_task.for_object.c_str() = %s\n", curr_task.for_object.c_str());
  ////////taskability_node_desc+=curr_task_to_validate.hri_task.for_object.c_str();
 ////  printf("task_plan_desc after adding obj name = %s \n",task_plan_desc.c_str());
  ////////task_plan_desc+='_';
  taskability_node_desc+=envPt_MM->robot[indices_of_MA_agents[res_node.target_agent]]->name;
  
  taskability_node_DESC_ID_map[taskability_node_desc]=ctr;
  printf(" Inserted into taskability_node_DESC_ID_map key= %s \n",taskability_node_desc.c_str());
  
  strcpy(res_node.desc,taskability_node_desc.c_str());
  
  res_node.node_id=ctr;
  
  taskability_graph.push_back(res_node);
  
  ctr++;
  
	 }
	}
      }
      
    }
  }


}

int print_manipulability_graph()
{
 
  
  printf(" ========================= MANIPULABILITY GRAPH =================\n");
std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=manipulability_graph.begin();it!=manipulability_graph.end();it++)
 {
   ///printf(">>>  for performing agent %d, for target agent %d, for task %d, no_candidate_poins %d\n**",it->performing_agent, it->target_agent, it->task, it->candidate_points->no_points);
   
   printf("-> Manipulability node id= %d, descriptor: %s\n",it->node_id, it->desc);
   printf("     Performing ag effort: (vis: %d, reach: %d), \n",    it->performing_ag_effort[VIS_ABILITY],it->performing_ag_effort[REACH_ABILITY]);
   
   ctr++;
 }
}

int print_taskability_graph()
{
printf(" ========================= TASKABILITY GRAPH =================\n");
std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=taskability_graph.begin();it!=taskability_graph.end();it++)
 {
   ///printf(">>>  for performing agent %d, for target agent %d, for task %d, no_candidate_poins %d\n**",it->performing_agent, it->target_agent, it->task, it->candidate_points->no_points);
   
   printf("-> Taskability node id= %d, descriptor: %s\n",it->node_id, it->desc);
   printf("    Candidate no. of points = %d, Performing ag effort: (vis: %d, reach: %d), Target ag effort: (vis: %d, reach: %d)\n",   it->candidate_points->no_points, it->performing_ag_effort[VIS_ABILITY],it->performing_ag_effort[REACH_ABILITY], it->target_ag_effort[VIS_ABILITY],it->target_ag_effort[REACH_ABILITY]);
   
   ctr++;
 }


}

int show_these_candidate_points(candidate_poins_for_task *candidate_points)
{
  int show_weight_by_color=1;
int show_weight_by_length=0;
int i=0;
  double min, max, w;
  double color[4];
   double radius=grid_around_HRP2.GRID_SET->pace/3.0;
  
for(i=0;i<candidate_points->no_points;i++)
    {

     if(show_weight_by_color==1||show_weight_by_length==1)
	{
	 
	  AKP_rgb_from_hue2(candidate_points->weight[i], color);
	}
	
      if(show_weight_by_color==1)
	{
	  g3d_drawDisc(candidate_points->point[i].x, candidate_points->point[i].y, candidate_points->point[i].z+0.01, radius, Any, color);
	}
      else
	{
	  g3d_drawDisc(candidate_points->point[i].x, candidate_points->point[i].y, candidate_points->point[i].z+0.01, radius, 4, NULL);
	}
    
      if(show_weight_by_length==1)
	{
	  g3d_drawOneLine(candidate_points->point[i].x, candidate_points->point[i].y,candidate_points->point[i].z, candidate_points->point[i].x, candidate_points->point[i].y, candidate_points->point[i].z+w/10.0, Any, color);
	 
	    }
      
  
    }
  
}


int init_currently_supported_tasks()
{
for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
 {
   if(i==GIVE_OBJECT||i==MAKE_OBJECT_ACCESSIBLE||i==HIDE_OBJECT||i==SHOW_OBJECT)
   {
    TASK_CURRENTLY_SUPPORTED[i]=1;
   }
   else
   {
    TASK_CURRENTLY_SUPPORTED[i]=0;
   }
 }  
}

int show_all_taskability_graph(int show_edge, int show_candidates)
{
int show_all_candidates=0;
std::vector<taskability_node>::iterator it; 
double x1, y1, z1, x2, y2, z2;
double x_c, y_c, z_c;
////double interval=grid_around_HRP2.GRID_SET->pace/25.0;

double z_shift[MAXI_NUM_OF_HRI_TASKS];
double increment=0.1;

double min_effort=0;
double max_effort=10;//Assuming that the maximum effort level for visibility or reachability will be less than 10   
double weight;
double color[MAXI_NUM_OF_HRI_TASKS][4];
double red, green, blue;
double color_hue;

p3d_vector3 p1, p2;

char task_name[50];

int total_no_supported_task=0;

for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
{
  if(TASK_CURRENTLY_SUPPORTED[i]==1)
  {
   total_no_supported_task++;
  }
}

int supp_task_ctr=0;
for(int i=0; i<MAXI_NUM_OF_HRI_TASKS; i++)
{
  if(TASK_CURRENTLY_SUPPORTED[i]==1)
  {
  strcpy(task_name,HRI_task_NAME_ID_map.find(i)->second.c_str());
  
 z_shift[i]=supp_task_ctr*increment; 
 
 color_hue=((double)supp_task_ctr/total_no_supported_task);
     AKP_rgb_from_hue2(color_hue, color[i]);
     color[i][3]=1;
     
      g3d_draw_text_at(task_name,10, 10+15*supp_task_ctr, color[i][0], color[i][1], color[i][2] );
      supp_task_ctr++;
  }
}

int ctr=0;
 for(it=taskability_graph.begin();it!=taskability_graph.end();it++)
 {
   ////printf(">>>  for performing agent %d, for target agent %d, for task %d, no_candidate_poins %d\n**",it->performing_agent, it->target_agent, it->task, it->candidate_points->no_points);
   
   if(show_candidates==1)
   {
  show_these_candidate_points(it->candidate_points);
   }
   
    if(show_edge==1)
    {
     
     //x1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmax)/2.0;
     //y1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymax)/2.0;
     
     x1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->joints[1]->abs_pos[0][3];
     y1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->joints[1]->abs_pos[1][3];
     z1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.zmax;
     
     //x2=(envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.xmax)/2.0;
     //y2=(envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.ymax)/2.0;
     x2=envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->joints[1]->abs_pos[0][3];
     y2=envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->joints[1]->abs_pos[1][3];
     z2=envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.zmax;
     
     
     
     z1+=z_shift[it->task];
     z2+=z_shift[it->task];
     
     p1[0]=x1;
     p1[1]=y1;
     p1[2]=z1;
     
     p2[0]=x2;
     p2[1]=y2;
     p2[2]=z2;
     
     //g3d_drawOneLine(x1, y1,z1, x2, y2, z2, it->task, NULL);
     /* 
     color_hue=((double)it->task/MAXI_NUM_OF_HRI_TASKS);
     AKP_rgb_from_hue2(color_hue, color);
     color[3]=1;
     */
     //////g3d_draw_arrow(p1, p2, color[0], color[1], color[2]);
     g3d_draw_arrow_with_width(p1, p2, 2, 0.07, color[it->task][0], color[it->task][1], color[it->task][2]);
     g3d_drawColorSphere(x1, y1, z1, .015, Any, color[it->task]);
     g3d_drawColorSphere(x2, y2, z2, .015, Any, color[it->task]);
     
     weight= ((double)it->performing_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     double t2=0.25;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)it->performing_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.30;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
      ////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
     /////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)it->target_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.35;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Red, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)it->target_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.40;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Yellow, NULL);
     
   ctr++;
    }
 }
return 1;

}



int draw_taskability_edge(taskability_node &node)
{

double x1, y1, z1, x2, y2, z2;
double x_c, y_c, z_c;
////double interval=grid_around_HRP2.GRID_SET->pace/25.0;


double increment=0.1;

double min_effort=0;
double max_effort=10;//Assuming that the maximum effort level for visibility or reachability will be will be less than 10   
double weight;
double color[MAXI_NUM_OF_HRI_TASKS][4];
double red, green, blue;
double color_hue;

p3d_vector3 p1, p2;

char task_name[50];


int ctr=0;

     //x1=(envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.xmax)/2.0;
     //y1=(envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.ymax)/2.0;
     x1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->joints[1]->abs_pos[0][3];
     y1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->joints[1]->abs_pos[1][3];
     z1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.zmax;
     
     //x2=(envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.xmax)/2.0;
     //y2=(envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[it->target_agent]]->BB.ymax)/2.0;
     x2=envPt_MM->robot[indices_of_MA_agents[node.target_agent]]->joints[1]->abs_pos[0][3];
     y2=envPt_MM->robot[indices_of_MA_agents[node.target_agent]]->joints[1]->abs_pos[1][3];
     z2=envPt_MM->robot[indices_of_MA_agents[node.target_agent]]->BB.zmax;
     
     
   
     
     p1[0]=x1;
     p1[1]=y1;
     p1[2]=z1;
     
     p2[0]=x2;
     p2[1]=y2;
     p2[2]=z2;
     
     //g3d_drawOneLine(x1, y1,z1, x2, y2, z2, it->task, NULL);
     /* 
     color_hue=((double)it->task/MAXI_NUM_OF_HRI_TASKS);
     AKP_rgb_from_hue2(color_hue, color);
     color[3]=1;
     */
     //////g3d_draw_arrow(p1, p2, color[0], color[1], color[2]);
     g3d_draw_arrow_with_width(p1, p2, 2, 0.07, color[node.task][0], color[node.task][1], color[node.task][2]);
     g3d_drawColorSphere(x1, y1, z1, .015, Any, color[node.task]);
     g3d_drawColorSphere(x2, y2, z2, .015, Any, color[node.task]);
     
     weight= ((double)node.performing_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     double t2=0.25;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)node.performing_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.30;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
      ////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
     /////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)node.target_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.35;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Red, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)node.target_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.40;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Yellow, NULL);
     
  
 
return 1;
}

int show_this_taskability_node(int node_id, int show_edge, int show_candidates)
{
  std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=taskability_graph.begin();it!=taskability_graph.end();it++)
 {
    if(it->node_id==node_id)
   {
   if(show_candidates==1)
     {
     show_these_candidate_points(it->candidate_points);
     }
     if(show_edge==1)
     {
       //taskability_node node=it;
      draw_taskability_edge(*it);
     }
  
     break;
   }
 }
  
}

int show_Ag_Ag_taskability_node(int performing_agent, int target_agent, int task, int show_edge, int show_candidates)
{
  std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=taskability_graph.begin();it!=taskability_graph.end();it++)
 {
   
   if(it->performing_agent==performing_agent&&it->target_agent==target_agent&&it->task==task)
   {
     if(show_candidates==1)
     {
     show_these_candidate_points(it->candidate_points);
     }
     if(show_edge==1)
     {
       //taskability_node node=it;
      draw_taskability_edge(*it);
     }
     
     break;
   }
 }
  
}



int show_all_manipulability_graph()
{

std::vector<taskability_node>::iterator it; 
double x1, y1, z1, x2, y2, z2;
double x_c, y_c, z_c;
////double interval=grid_around_HRP2.GRID_SET->pace/25.0;


double min_effort=0;
double max_effort=10;//Assuming that the maximum effort level for visibility or reachability will be will be less than 10   
double weight;
double color[4]={1,1,0,1};
double red, green, blue;
double color_hue;

p3d_vector3 p1, p2;

char task_name[50];


int ctr=0;
 for(it=manipulability_graph.begin();it!=manipulability_graph.end();it++)
 {
   ////printf(">>>  for performing agent %d, for target agent %d, for task %d, no_candidate_poins %d\n**",it->performing_agent, it->target_agent, it->task, it->candidate_points->no_points);
   
     //x1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmax)/2.0;
     //y1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymax)/2.0;
     x1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->joints[1]->abs_pos[0][3];
     y1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->joints[1]->abs_pos[1][3];
     z1=envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.zmax;
     
     //x2=(envPt_MM->robot[it->target_object]->BB.xmin+envPt_MM->robot[it->target_object]->BB.xmax)/2.0;
     //y2=(envPt_MM->robot[it->target_object]->BB.ymin+envPt_MM->robot[it->target_object]->BB.ymax)/2.0;
     x2=envPt_MM->robot[it->target_object]->joints[1]->abs_pos[0][3];
     y2=envPt_MM->robot[it->target_object]->joints[1]->abs_pos[1][3];
     z2=envPt_MM->robot[it->target_object]->BB.zmax;
     
     p1[0]=x1;
     p1[1]=y1;
     p1[2]=z1;
     
     p2[0]=x2;
     p2[1]=y2;
     p2[2]=z2;
     
     //g3d_drawOneLine(x1, y1,z1, x2, y2, z2, it->task, NULL);
     /* 
     color_hue=((double)it->task/MAXI_NUM_OF_HRI_TASKS);
     AKP_rgb_from_hue2(color_hue, color);
     color[3]=1;
     */
     //////g3d_draw_arrow(p1, p2, color[0], color[1], color[2]);
     g3d_draw_arrow_with_width(p1, p2, 2, 0.07, color[0], color[1], color[2]);
     g3d_drawColorSphere(x1, y1, z1, .015, Any, color);
     g3d_drawColorSphere(x2, y2, z2, .015, Any, color);
     
     weight= ((double)it->performing_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     double t2=0.25;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)it->performing_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.30;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
      ////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
     /////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     
     
   ctr++;
 }
return 1;

}

int draw_manipulability_edge(taskability_node &node)
{
  double x1, y1, z1, x2, y2, z2;
double x_c, y_c, z_c;
////double interval=grid_around_HRP2.GRID_SET->pace/25.0;

double min_effort=0;
double max_effort=10;//Assuming that the maximum effort level for visibility or reachability will be will be less than 10   
double weight;
double color[4]={1,1,0,1};
double red, green, blue;
double color_hue;

p3d_vector3 p1, p2;

char task_name[50];

  
     //x1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.xmax)/2.0;
     //y1=(envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymin+envPt_MM->robot[indices_of_MA_agents[it->performing_agent]]->BB.ymax)/2.0;
     x1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->joints[1]->abs_pos[0][3];
     y1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->joints[1]->abs_pos[1][3];
     z1=envPt_MM->robot[indices_of_MA_agents[node.performing_agent]]->BB.zmax;
     
     //x2=(envPt_MM->robot[node.target_object]->BB.xmin+envPt_MM->robot[node.target_object]->BB.xmax)/2.0;
     //y2=(envPt_MM->robot[node.target_object]->BB.ymin+envPt_MM->robot[node.target_object]->BB.ymax)/2.0;
     x2=envPt_MM->robot[node.target_object]->joints[1]->abs_pos[0][3];
     y2=envPt_MM->robot[node.target_object]->joints[1]->abs_pos[1][3];
     z2=envPt_MM->robot[node.target_object]->BB.zmax;
     
     p1[0]=x1;
     p1[1]=y1;
     p1[2]=z1;
     
     p2[0]=x2;
     p2[1]=y2;
     p2[2]=z2;
     
     //g3d_drawOneLine(x1, y1,z1, x2, y2, z2, node.task, NULL);
     /* 
     color_hue=((double)node.task/MAXI_NUM_OF_HRI_TASKS);
     AKP_rgb_from_hue2(color_hue, color);
     color[3]=1;
     */
     //////g3d_draw_arrow(p1, p2, color[0], color[1], color[2]);
     g3d_draw_arrow_with_width(p1, p2, 2, 0.07, color[0], color[1], color[2]);
     g3d_drawColorSphere(x1, y1, z1, .015, Any, color);
     g3d_drawColorSphere(x2, y2, z2, .015, Any, color);
     
     weight= ((double)node.performing_ag_effort[VIS_ABILITY] - min_effort )/(max_effort-min_effort);
     
     ////////// AKP_rgb_from_hue2(weight, color);
     
     double t2=0.25;//interval;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
	  
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Green, NULL);
     //////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
     weight= ((double)node.performing_ag_effort[REACH_ABILITY] - min_effort )/(max_effort-min_effort);
     
    ////////// AKP_rgb_from_hue2(weight, color);
     
     t2=0.30;
     
     x_c=(1-t2)*x1+t2*x2;
     y_c=(1-t2)*y1+t2*y2;
     z_c=(1-t2)*z1+t2*z2;
     g3d_drawColorSphere(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
      ////////////g3d_drawDisc(x_c, y_c, z_c, (weight/10.0)+0.02, Blue, NULL);
     /////////g3d_drawDisc(x_c, y_c, z_c, .02, Any, color);
     
}

int show_this_manipulability_node(int node_id)
{
  std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=manipulability_graph.begin();it!=manipulability_graph.end();it++)
 {
   if(it->node_id==node_id)
   {
     
     draw_manipulability_edge(*it);
     
     return 1;
   
   }
 }
  
}

int show_Ag_Obj_manipulability_node(int performing_agent, int target_object)
{
 std::vector<taskability_node>::iterator it; 

int ctr=0;
 for(it=manipulability_graph.begin();it!=manipulability_graph.end();it++)
 {
   
   if(it->performing_agent==performing_agent&&it->target_object==target_object)
   {
      draw_manipulability_edge(*it);
   }
 }
}
/////// ////////////////////////////////////// //////////
