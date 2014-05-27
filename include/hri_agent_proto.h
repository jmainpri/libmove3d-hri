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
extern int hri_assign_global_agents(HRI_AGENTS *agents);
extern HRI_AGENT *  hri_assign_source_agent(char *agent_name, HRI_AGENTS *agents);
extern HRI_AGENTS * hri_create_agents(void);
extern int hri_destroy_agents(HRI_AGENTS *agents);
extern HRI_AGENT * hri_create_agent(p3d_rob * robot);
extern int hri_destroy_agent(HRI_AGENT *agent);
extern HRI_MANIP * hri_create_empty_agent_manip();
extern HRI_MANIP * hri_create_agent_manip(HRI_AGENT * agent);
extern int hri_destroy_agent_manip(HRI_MANIP *manip);
extern HRI_NAVIG * hri_create_agent_navig(HRI_AGENT * agent);
extern int hri_destroy_agent_navig(HRI_NAVIG *navig);
extern HRI_PERSP * hri_create_agent_perspective(HRI_AGENT * agent, p3d_env *env);
extern int hri_destroy_agent_perspective(HRI_PERSP *persp);
extern int hri_fill_all_agents_default_tasks(HRI_AGENTS * agents);
extern int hri_create_fill_agent_default_manip_tasks(HRI_MANIP * manip, GIK_TASK ** tasklist, int * tasklist_no, HRI_AGENT_TYPE type, HRI_AGENT * agent);
extern int hri_create_assign_default_manipulation(HRI_AGENTS * agents);
extern int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_vector3 * goalCoord, double approach_distance, configPt *q);
extern int hri_agent_single_task_manip_move(HRI_AGENT * agent, HRI_GIK_TASK_TYPE type, p3d_rob * object, configPt *q);
extern int g3d_hri_display_shared_zone();
extern int hri_agent_load_default_arm_posture(HRI_AGENT * agent, configPt q);
extern int hri_agent_get_posture(HRI_AGENT * agent);
extern int hri_agent_compute_posture(HRI_AGENT * agent, double neck_height, int state, configPt q);
extern int hri_agent_set_human_standing_posture(HRI_AGENT * agent,configPt q);
extern int hri_agent_set_human_seated_posture(HRI_AGENT * agent, configPt q);
extern int hri_agent_compute_state_posture(HRI_AGENT * agent, int state, configPt q);
extern int hri_agent_set_human_t_shirt_color(HRI_AGENT * agent,int color);
extern int hri_agent_is_grasping_obj(HRI_AGENT* agent, bool is_grasping , const char* OBJECT , int armId);
extern int hri_agent_is_grasping_obj_at_center(HRI_AGENT* agent, const char* OBJECT , int armId , p3d_matrix4 t);
extern void hri_agent_print_config_for_softmotion(HRI_AGENT* agent);
extern int hri_is_robot_an_agent(p3d_rob * robot, HRI_AGENTS * agents, int * is_human, int * agent_idx);
extern HRI_AGENT* hri_get_one_agent_of_type(HRI_AGENTS * agents, HRI_AGENT_TYPE agentType);
extern HRI_AGENT* hri_get_agent_by_name( HRI_AGENTS* agents, const char* name );
