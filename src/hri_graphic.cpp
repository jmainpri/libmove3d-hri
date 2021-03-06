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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "hri.h"

#include "move3d-gui.h"

int HRI_DRAW_TRAJ;
// Hri distance draw
extern std::string hri_text_to_display;
extern bool hri_draw_distance;
extern std::vector<double> hri_disp_dist;

int STOP_AGENT_STATE_ANALYSIS=0;//Will be used by spark to set/reset
int MA_ASA_WITHOUT_SPARK=0; // WARNING: It should be always set to 0 when the libhri is used with Spark or mhp. Set it to 1 only to execute the MA and ASA functions, when libhri is used without spark, because of the synchronization problem
int SHOW_OBJECT_FACTS=0;
int MA_DISPLAY_FLAG=1;//Used to set the flag to display MA related results

void g3d_hri_main()
{
  //hri_hri_inter_point_test();
  g3d_hri_bt_draw_active_bitmaps(BTSET);
  g3d_hri_bt_draw_active_3dbitmaps(INTERPOINT);
  g3d_hri_bt_draw_active_3dbitmaps(OBJSET);
  g3d_hri_bt_draw_targets(BTSET);
  hri_exp_draw_ordered_points();
  //g3d_hri_display_test();
  //g3d_draw_all_agents_fovs(GLOBAL_AGENTS);
  g3d_hri_display_all_agents_sees(GLOBAL_AGENTS);

  if(HRI_DRAW_TRAJ){g3d_draw_all_tcur();}
  
#ifdef USE_MIGHTABILITY_MAPS
  //AKP: Function to display reachability, visibility of the object
  //if(SHOW_OBJECT_FACTS==1)
  //show_object_facts();
  
  if(MA_ASA_WITHOUT_SPARK==1)
  {
    ////printf("Inside g3d_draw_env_custom() \n");
    execute_Mightability_Map_functions();
    
    if(STOP_AGENT_STATE_ANALYSIS!=1)
    {
		  hri_execute_Agent_State_Analysis_functions();
    }
  }
  
  if(MA_DISPLAY_FLAG==1)
  {
    MA_Display_Functions();
  }
#endif
  
  // Displaying things (Debug information) that are not part of the scene
  // such as writing text or minial distance vectors will break agent visibility computations. 
  // They should be enabled/disabled inside win->vs.enableLogo which is turned off when drawing in the backbuffer
  if(G3D_WIN->vs.enableLogo==1) 
  {
  #ifdef USE_MIGHTABILITY_MAPS
  //AKP: Function to display reachability, visibility of the object
  if(SHOW_OBJECT_FACTS==1)
  show_object_facts();
  #endif
    
    hri_draw_kinect_points();
    hri_draw_kinect_human_arms( GLOBAL_AGENTS );
  
    hri_draw_kinect_points();
    hri_draw_kinect_human_arms( GLOBAL_AGENTS );
  
    g3d_draw_all_agents_fovs(GLOBAL_AGENTS);
    
    hri_draw_kinect_state(G3D_WIN->vs, 0.90, 5, 0.07); 
    hri_draw_mindist();
    hri_draw_action_monitoring_spheres();
    hri_draw_divergent_positions();
    hri_draw_situation_assessment_computation_status(0.20, 5, 0.07);

    if(false)
    {
      // Display a string with text
      char string[150]; 
      //sprintf(string, "HRI cost = %2.2f", hri_cost_to_display );
      strcpy(string,hri_text_to_display.c_str());
      glColor3f(0.0,0.0,0.0);
      g3d_draw_text(string);
    }
  }
}

void g3d_hri_display_visible_objects(HRI_AGENT *agent)
{
  p3d_env *env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int i;

  agent->perspective->enable_vision_draw = TRUE;

  for(i=0; i<agent->perspective->currently_sees.vis_nb; i++) {
    if(agent->perspective->currently_sees.vis[i] == HRI_VISIBLE) {
      if(agent->perspective->currently_sees.vispl[i] == HRI_FOA) {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_GREEN_DISPLAY);
      }
      //else {
      //  p3d_set_robot_display_mode(env->robot[i], P3D_ROB_BLUE_DISPLAY);
      //}
    }
    else {
      if (agent->perspective->currently_sees.vispl[i] == HRI_FOA || 
          agent->perspective->currently_sees.vispl[i] == HRI_FOV) 
      {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_RED_DISPLAY);
      }
      else {
        p3d_set_robot_display_mode(env->robot[i], P3D_ROB_DEFAULT_DISPLAY);
      }
    }
  }
}


void g3d_hri_display_all_agents_sees(HRI_AGENTS *agents)
{
  int i;

  if(agents != NULL){
    for (i=0; i<agents->all_agents_no; i++) {
      if(agents->all_agents[i]->perspective->enable_visible_objects_draw)
        g3d_hri_display_visible_objects(agents->all_agents[i]);
    }
  }
}



