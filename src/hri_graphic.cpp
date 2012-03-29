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
int MA_ASA_WITHOUT_SPARK=0; // It will be used to execute the MA and ASA functions, when libhri is used without spark, because of the synchronization problem 
int SHOW_OBJECT_FACTS=0;

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



