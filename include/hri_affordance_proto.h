/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue May 12 14:41:50 2009
 */
#ifndef __CEXTRACT__

#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include "../lightPlanner/proto/ManipulationUtils.hpp"
#include <boost/graph/graph_concepts.hpp>

extern int find_affordance();
extern int show_affordance();
extern int create_3d_grid_for_HRP2_GIK(point_co_ordi grid_center);
extern int show_3d_grid_for_HRP2_GIK();
extern int Find_AStar_Path(double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip);
extern int show_HRP2_gik_sol();
extern void update_human_state_old(int state); //1 means sitting 0 means standing
extern int show_affordance_new();
extern int show_3d_grid_affordances();
extern int create_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int expansion, int bt_type);
extern int show_3d_grid_affordances_new();
extern int put_object_for_human_to_take();
extern int HRP2_find_collision_free_path_to_take_object();
extern int HRP2_put_object_for_human_to_take();
extern int show_exact_obstacles_for_HRP2_GIK_manip(hri_bitmapset * bitmapset, int bt_type);
extern int HRP2_return_hand_to_rest_position();
extern int HRP2_grasp_object(int for_hand,double hand_clench_val);
extern int HRP2_release_object(int for_hand,double hand_clench_val);
extern int HRP2_look_at_bottle();
extern int execute_current_HRP2_GIK_solution(int with_bottle);
extern int show_weighted_candidate_points_to_put_obj(int show_weight);
extern int update_human_state(int state); //1 means sitting 0 means standing
extern int get_cubic_spline_by_Hermite_polynomial(point_co_ordi point[100], int n, point_co_ordi init_vel, point_co_ordi final_vel, int continuity_constraint_type, double sampling_period, double total_time, point_co_ordi *resultant_spline);
extern int get_Hermite_polynomial_points(point_co_ordi st_point, point_co_ordi end_point, point_co_ordi st_vel, point_co_ordi end_vel, double sampling_rate, point_co_ordi *resultant_points);
extern int find_spline_path_for_HRP2_hand(hri_bitmapset * btset, hri_bitmap* bitmap, int hand_by_reach);
extern int show_spline_path_for_HRP2_hand();
extern int find_HRP2_GIK_sol_for_spline_path(int hand_by_reach, int state, int use_body_part, int maintain_hand_orientation);
extern int get_AStar_path(hri_bitmapset * btset, hri_bitmap* bitmap);
extern int find_spline_path_for_via_points(point_co_ordi via_points[500], int no_via_points);
extern int HRP2_find_collision_free_path_to_take_object_new();//In this version, the entire path is divided into three phases, first reach near to the bottle, then orient the hand then again plan a path from the new hand position to the bottle while maintaining the orientation
extern int find_HRP2_GIK_sol_for_hand_orientation(p3d_vector3 req_hand_orientation_in_global_frame, int hand_by_reach, int state, int use_body_part);
extern double* get_HRP2_hand_x_axis_orientation_in_global_frame(int for_hand);//1 for left, 2 for right hand
extern int find_affordance_new();
extern int find_reachable_sphere_surface(int for_hand, HRI_TASK_AGENT for_agent);
extern int show_weighted_candidate_points_to_show_obj(int show_weight);
extern int HRP2_show_object_to_human();
extern int show_weighted_candidate_points_to_hide_obj();
extern int HRP2_hide_object_from_human();
extern int HRP2_put_object_for_human_to_take_new();
extern int HRP2_hide_object_from_human_new();
extern int HRP2_show_object_to_human_new();
extern int execute_Mightability_Map_functions();
extern int HRP2_look_at_point(p3d_vector3 point_to_look, int use_body_part);//use_body_part=0 for heand only, 1 for upper body, 2 for whole body. option 0 is not implemented yet
extern int update_robots_and_objects_status();
extern int update_3D_grid_for_Mightability_Maps(hri_bitmapset * bitmapset, int expansion, int bt_type);
extern int update_3D_grid_for_Mightability_Maps_new(hri_bitmapset * bitmapset, int expansion, int bt_type);
extern int update_Mightability_Maps();
extern int show_3D_workspace_Bounding_Box();
extern int make_cells_around_point_obstacle_free(double hand_pos[3], int expansion);
extern int make_cells_around_point_as_obstacle(hri_bitmapset *btset, int bt_type, point_co_ordi point, int extension);
extern int find_candidate_points_on_plane_to_put_obj_new();
extern int assign_weights_on_candidte_points_to_put_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent);
extern int reverse_sort_weighted_candidate_points_to_put_obj();
extern int assign_weights_on_candidte_points_to_show_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent);


extern int assign_weights_on_candidte_points_to_hide_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent, int performing_agent_rank);

extern int reverse_sort_weighted_candidate_points_to_show_obj();
extern int reverse_sort_weighted_candidate_points_to_hide_obj();
extern int reverse_sort_weighted_candidate_points_to_give_obj();
extern int find_candidate_points_on_plane_to_hide_obj_new();
extern int find_candidate_points_to_show_obj_new();
extern int find_candidate_points_on_plane_to_put_obj();
extern int find_candidate_points_on_plane_to_hide_away_obj();
extern int find_candidate_points_on_plane_to_put_away_obj();
extern int find_candidate_points_to_give_obj();
extern int Create_and_init_Mightability_Maps(char *around_object);
extern int get_set_of_points_to_put_object(char *object_name);
extern int get_set_of_points_to_show_object(char *object_name);
extern int get_set_of_points_to_hide_object(char *object_name);
extern int get_set_of_points_to_give_object(char *object_name);
extern int JIDO_find_candidate_points_on_plane_to_put_obj();
extern int find_symbolic_Mightability_Map();
extern int show_symbolic_Mightability_Map_Relations();
extern int is_object_graspable();
extern int show_candidate_points_for_curr_geo_node();
extern int find_symbolic_Mightability_Map_new();
extern int make_cells_around_point_as_near_to_obstacle(double hand_pos[3], int expansion);
extern int show_weighted_candidate_points_for_putinto_obj(int show_weight);
extern int reverse_sort_weighted_candidate_points_to_putinto_obj();
extern int get_object_mightabilities();
extern int show_object_Mightabilities();
extern int show_first_non_visible_cells(int obj_index);
extern int get_index_of_robot_by_name(char *rob_name);
extern int test_jido_grasp_traj();
extern int JIDO_put_obj_in_hand_into_trashbin(char trashbin_name[50], char obj_to_manipulate[50]);
extern int execute_JIDO_trajectory();
extern int simulate_object_falling(char obj_name[50]);
extern int play_all_JIDO_trajectories();
extern int move_object_on_a_path();
extern int update_object_pos_from_mocap(point_co_ordi *mrkrs_pos, int obj_index, char object_name[30]);
extern int read_update_object_pos_from_mocap_data_file();
extern int update_human_pos_from_mocap_eye_glasses(point_co_ordi *mrkrs_pos, int obj_index, char object_name[30], int is_primary_human);
extern int update_human_pos_from_mocap_rigid_hat(point_co_ordi *mrkrs_pos, int obj_index, char object_name[30], int is_primary_human);
extern int init_mocap_data_run_file();
extern int show_axis_of_FOV_from_mocap_eye_glass_data();
extern int virtually_update_human_state_new(p3d_rob* human_Pt,int state); //1 means sitting 0 means standing
extern int virtually_update_non_primary_human_state(int state, int hum_index); //1 means sitting 0 means standing, hum_index is the index of robot in environment (envPt)
extern int JIDO_make_obj_accessible_to_human ( char obj_to_manipulate[50] );
extern int show_world_state_of_entire_plan(std::vector<HRI_task_node> &hri_task_list, int exec_path_configs);
extern int JIDO_show_obj_to_human ( char obj_to_manipulate[50] );
extern int JIDO_find_candidate_points_to_show_obj();
/*extern int test_geometric_plan_creation_for_JIDO();*/
extern int JIDO_give_obj_to_human (char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> &graspList, std::list<gpPlacement> &curr_placementList);
extern int JIDO_find_candidate_points_to_give_obj();
extern int assign_weights_on_candidte_points_to_give_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent, int performing_agent_rank);

extern int show_weighted_candidate_points_to_give_obj(int show_weight);
extern int JIDO_find_candidate_points_on_plane_to_hide_obj();
extern int JIDO_hide_obj_from_human ( char obj_to_manipulate[50] );
extern int JIDO_find_candidate_points_to_hide_obj();
extern int assign_weights_on_candidte_points_to_hide_away_obj(char *object_name);
extern int JIDO_find_candidate_points_on_plane_to_hide_away_obj();
extern int JIDO_hide_away_obj_from_human ( char obj_to_manipulate[50] );
extern int JIDO_find_candidate_points_to_hide_away_obj();
extern int show_weighted_candidate_points_to_hide_away_obj(int show_weight);
extern int assign_weights_on_candidte_points_to_put_away_obj(char *object_name);
extern int JIDO_find_candidate_points_on_plane_to_put_away_obj();
extern int JIDO_put_away_obj_from_human ( char obj_to_manipulate[50] );
extern int JIDO_find_candidate_points_to_put_away_obj();
extern int show_weighted_candidate_points_to_put_away_obj(int show_weight);
extern int show_humans_perspective(HRI_AGENT * agent, int save);//AKP WARNING: FOV is not taken from agent->perspective->fov; It is set as 120 degree in this function itself
extern int restore_previous_win_state();
extern int is_object_visible_for_agent(HRI_AGENT * agent, p3d_rob *object, double threshold, int save, int draw_at_end);
////extern int get_placements_in_3D(p3d_rob *object,  std::list<gpPlacement> &placementListOut);
extern int show_all_how_to_placements_in_3D(point_co_ordi at_place,int use_random_colors, int skip, int semi_transparent);
extern int initialize_MM_resultant_set();
extern int caculate_and_show_resultant_MM();
extern int show_current_ranked_candidate_placements();
extern int show_current_task_candidate_points(int show_weight_by_color, int show_weight_by_length, candidate_poins_for_task *candidate_points);
extern int show_candidate_points_for_current_task(int show_weight_by_color, int show_weight_by_length);

extern int assign_weights_on_candidate_points_to_displace_obj(char *object_name);
extern int reverse_sort_weighted_candidate_points_to_displace_obj();
extern int find_candidate_points_on_plane_to_displace_obj();
extern int show_weighted_candidate_points_to_displace_obj(int show_weight);
// extern int test_geometric_plan_creation_new();
extern int HRP2_make_obj_accessible_to_human( char obj_to_manipulate[50] );
extern int HRP2_show_obj_to_human( char obj_to_manipulate[50] );
extern int HRP2_give_obj_to_human( char obj_to_manipulate[50] );
extern int HRP2_hide_obj_from_human( char obj_to_manipulate[50] );
extern int make_cells_corresponding_to_object_obstacle_free(char *object_name);
extern int HRP2_take_object(char obj_to_manipulate[50]);
extern int set_current_HRI_manipulation_task(int arg);
extern int find_current_HRI_manip_task_solution(HRI_task_desc curr_task, traj_for_HRI_task &res_traj);
extern int find_Mightability_Maps(char *around_object);
extern int JIDO_give_obj_to_human( char *obj_to_manipulate );
extern int find_candidate_points_for_current_HRI_task(HRI_TASK_TYPE curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, candidate_poins_for_task *resultant_candidate_point);
extern int reverse_sort_HRI_task_weighted_candidate_points(candidate_poins_for_task *candidate_points);
extern int JIDO_find_HRI_task_solution(HRI_TASK_TYPE CURR_TASK, HRI_TASK_AGENT for_agent, char *obj_to_manipulate);
extern int find_HRI_task_candidate_points(HRI_TASK_TYPE CURR_TASK, char *obj_to_manipulate, HRI_TASK_AGENT performed_by,  HRI_TASK_AGENT performed_for, int performing_agent_rank, candidate_poins_for_task *curr_resultant_candidate_points, int consider_obj_dimension);
extern object_Symbolic_Mightability_Maps_Relation* create_object_oriented_Mightability_obj();
extern int delete_object_oriented_Mightability_obj(object_Symbolic_Mightability_Maps_Relation *OOM);
extern int copy_current_object_oriented_Mightability_into(object_Symbolic_Mightability_Maps_Relation* target);
extern int print_object_oriented_Mightability(object_Symbolic_Mightability_Maps_Relation* OOM);
extern int store_OOM_before_task();
extern int print_object_oriented_Mightability_for_object(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index);
extern int print_object_oriented_Mightability_for_object_by_agent(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index, HRI_TASK_AGENT agent);

extern int init_visibility_acceptance_for_tasks();
extern int JIDO_perform_task (char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> curr_placementList, traj_for_HRI_task &res_trajs);
extern int validate_HRI_task(HRI_task_desc curr_task, int task_plan_id, int for_proactive_info, double timeout);
extern int show_traj_for_this_HRI_task(HRI_task_node &for_task,int show_traj);
extern int show_desired_HRI_task_plan();
extern int show_plan_for_this_sub_task(HRI_task_node &for_task, traj_for_HRI_sub_task &sub_task_traj, int sub_task_index, int show_traj);
extern int convert_symbolic_HRI_task_desc_into(symbolic_HRI_task_desc &HRI_task_ip, HRI_task_desc &task_to_validate);
extern int get_soft_motion_trajectories_for_plan_ID(int HRI_task_plan_id, std::vector <SM_TRAJ> &smTrajs);
extern int ececute_this_HRI_task_p3d_Traj_in_simu(char *for_robot, p3d_traj *traj);
extern int ececute_this_HRI_task_SM_Traj_in_simu(char *for_robot, SM_TRAJ &smTraj);
extern int show_p3d_trajectories_for_plan_ID(int HRI_task_plan_id);
extern int get_single_soft_motion_traj_for_SHARY(int HRI_task_plan_id, int sub_traj_st_index, int sub_traj_end_index, SM_TRAJ &smTraj );
extern int g3d_is_object_visible_from_robot(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *robot, p3d_rob *object, double *result);
extern int init_manipulation_planner(char robot_name[100]);
extern int update_3d_grid_reachability_for_agent_MM(HRI_TASK_AGENT for_agent, MA_agent_hand_name for_hand, int for_state);
extern int get_human_head_relative_yaw_pitch_for(HRI_TASK_AGENT for_agent, point_co_ordi for_point, double &relative_yaw, double &relative_pitch);
extern int find_MA_Agent_visibility(HRI_TASK_AGENT for_agent, char *obj_name, double &visibility_val);
extern int JIDO_find_solution_to_take(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs);
extern int find_human_give_candidate_points(HRI_TASK_AGENT performed_by, int curr_reach_state);
extern int get_robot_proactive_solution_info( HRI_task_desc curr_task, traj_for_HRI_task &res_traj);
extern int show_hand_grasps_of_list(p3d_rob *hand, p3d_rob *object, std::list<gpGrasp> *graspList);
extern int get_grasp_list_for_object(char *obj_to_manipulate, std::list<gpGrasp> &graspList);
extern int show_all_how_to_placements_in_3D(point_co_ordi at_place, int use_random_colors, int skip, int semi_transparent, std::list<gpPlacement> *placement_config_list);
extern int get_placements_in_3D(char *obj_to_manipulate,  std::list<gpPlacement> &placementListOut);
extern int find_candidate_points_for_current_HRI_task_for_object(HRI_TASK_TYPE_ENUM curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, int performing_agent_rank, candidate_poins_for_task *resultant_candidate_point, char *object, int consider_obj_dimension);
extern int get_indices_for_MA_agents();
extern int show_all_grasps_for_this_placement_list_at_place(p3d_rob *hand, p3d_rob *object, std::list<gpGrasp> *graspList, std::list<gpPlacement> *placement_list, point_co_ordi at_point);
extern int show_all_grasps_for_this_placement_at_place(p3d_rob *hand, p3d_rob *object, std::list<gpGrasp> *graspList, gpPlacement placement, point_co_ordi at_point);
extern int init_agents_for_MA_and_ASA();
extern int set_all_Mightability_Analyses_to_update();
extern int get_candidate_points_for_HRI_task(HRI_task_desc curr_task, int is_performing_agent_master, int consider_object_dimension);
extern int set_accepted_effort_level_for_HRI_task(HRI_task_agent_effort_level desired_level);
extern int get_HRI_task_id_type_by_name(char task_name[50], int &task_type);
extern int JIDO_find_solution_to_take_new(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs);
extern int JIDO_find_solution_to_take_new2(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs);
extern int JIDO_find_solution_to_take_new3(char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs);
extern int assign_weights_on_candidte_points_to_show_obj_new(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent, int performing_agent_rank);
extern int robot_perform_task (char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, char by_hand[50], HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> curr_placementList, int filter_contact_polygon_inside_support, int object_in_hand, traj_for_HRI_task &res_trajs);
extern int get_object_list_on_object(char* supporting_obj_name, std::vector<std::string> &ON_object_list, std::vector<int> &ON_object_indices);
extern int print_this_string_list(std::vector<std::string> &str_list);
extern int get_clean_the_table_plan(char *Table_name);
extern int update_effort_levels_for_HRI_Tasks(HRI_task_desc &curr_task, int effort_for, int desired_reach_effort, int desired_vis_effort);//effort_for=1 means for target agent, 2 means for performing agent
extern int show_object_facts();
extern int init_object_facts_data();
extern int get_effort_based_object_facts(int for_agent, int for_effort_level, int for_ability_type);
extern int show_agent_ability_effort_points();
extern int get_effort_based_places_facts(agent_ability_effort_set &ag_ab_eff);
extern int find_taskability_link_between_two_agents_for_task(HRI_task_desc curr_task, taskability_node &res_node );
extern int find_taskability_graph();
extern int print_taskability_graph();
extern void AKP_rgb_from_hue2(double x, double color[4]);
extern int show_all_taskability_graph(int show_edge, int show_candidates);
extern int show_this_taskability_node(int node_id, int show_edge, int show_candidates);
extern void g3d_draw_arrow_with_width(p3d_vector3 p1, p3d_vector3 p2, double line_width, double Cone_height, double red, double green, double blue);
extern void g3d_draw_text_at(char* string, double x_shift, double y_shift, double r, double g, double b);
extern int init_currently_supported_tasks();
extern int find_agent_object_affordance(HRI_task_desc curr_task, taskability_node &res_node );
extern int get_maximum_possible_lean_angle_for_agent(HRI_TASK_AGENT for_agent, double &res_lean_forward_ang);
extern int find_manipulability_graph(std::vector<taskability_node> &manipulability_graph);
extern int init_grasp_exists_for_object();
extern int update_analysis_type_effort_level_group(int agent, int agent_posture);
extern int print_manipulability_graph(std::vector<taskability_node> &manipulability_graph);
extern int show_all_manipulability_graph(std::vector<taskability_node> &manipulability_graph);
extern int show_taskabilities(show_taskability_params &curr_params, std::vector<taskability_node> curr_manipulability_graph );
extern int show_Ag_Ag_taskability_node(int performing_agent, int target_agent, int task, int show_edge, int show_candidates);
extern int show_Ag_Obj_manipulability_node(int performing_agent, int target_object, std::vector<taskability_node> &manipulability_graph);
extern int show_this_manipulability_node(int node_id,std::vector<taskability_node> &manipulability_graph);
extern int print_this_taskability_params(show_taskability_params &curr_params );
extern int show_agent_state_configs(int for_agent, int for_ability_type, int for_state) ;
extern int free_state_configs(int for_agent, int for_state, int for_ability_type);
extern int find_least_effort_state_for_agent_ability_for_obj(int for_agent, int for_ability, int for_object);
extern int init_Ag_Obj_Ab_mini_effort_states();
extern int show_Ag_Ab_Obj_least_effort_states(int for_agent, int for_ability, int for_obj_index);
extern int init_and_allocate_OOM_data_field();
extern int init_Agent_State_configs_data();
extern int check_config_for_cell_already_pushed(int cell_x, int cell_y, int cell_z, int for_agent, int for_ability, int for_state, int for_hand, configPt ptr);//for_hand is not used in case of VIS_ABILITY
extern int find_current_hri_goal_solution();
extern int find_putinto_points_for_object(int container_index);
extern int show_put_into_points();
extern int update_put_into_points();
extern int init_Container_Oriented_put_into_points();
extern int check_obj_is_container(char *obj_name);
extern int get_index_for_obj_in_putintolist(int container_index);
extern int find_agent_container_putinto_points(int agent_type, int container_obj_indx, int agent_cur_effort[2], candidate_poins_for_task *resultant_candidate_point);
extern int find_put_into_ability_graph();
extern int show_all_put_into_ability_graph();
extern int get_src_targ_vertex_pair_for_task(HRI_task_desc for_task, MY_GRAPH G, MY_VERTEX_DESC &src, MY_VERTEX_DESC &targ);
extern int get_shortest_path_for_this_pair(MY_GRAPH &G, std::vector<MY_VERTEX_DESC> &predecessors, std::vector<double> &weights, MY_VERTEX_DESC &src,MY_VERTEX_DESC &targ);
extern int get_reachable_config(int for_agent, p3d_rob* agent_Pt,p3d_rob* obj_Pt,configPt ag_curr_config, configPt &ag_res_config, int *by_hand);
extern int get_agent_object_affordance_reach_disp_effort(p3d_rob * agent_Pt, p3d_rob * obj_Pt, int for_MA_agent, int only_first_solution);
extern int modify_graph_for_agent_busy(MY_GRAPH &G,int for_agent_type, int agent_busy);
extern int get_shortest_path_for_this_pair_new(MY_GRAPH &G, std::vector<MY_VERTEX_DESC> &predecessors, std::vector<double> &weights, MY_VERTEX_DESC &src,MY_VERTEX_DESC &targ,  std::vector<MY_EDGE_DESC> &path);
extern int print_path_of_graph(MY_GRAPH &G, std::vector<MY_EDGE_DESC> &path, std::string &path_desc);
extern int find_give_task_link_between_two_agents_for_displacement_effort(p3d_rob* performing_agent, p3d_rob* target_agent, int mutual_effort_criteria, bool is_human_standing);
extern int test_to_extract_candidate_paces();
extern int get_places_based_on_this_fact(agent_ability_effort_tuple &ag_ab_eff, std::set <cell_X_Y_Z> &curr_places, int operation_type, int on_plane);
extern int show_this_agent_ability_effort_points(std::set <cell_X_Y_Z> &curr_places);
extern int store_current_world_state_physical_position(world_state_configs &curr_WS);
extern int compare_two_world_states_physical_positions(world_state_configs &WS1,world_state_configs &WS2, std::vector<int> &moved_objects , std::vector<int> &lost_objects, std::vector<int> &new_objects, std::vector<std::string> &physical_changes);
extern int find_world_state_with_id(world_state_configs &WS, int WS_id);
extern int find_ability_graph(HRI_TASK_TYPE ability, std::vector<taskability_node> &ability_graph);
extern int find_ability_graph_with_id(std::vector<ability_graph> ab_graphs, int ab_gr_id, ability_graph &res_ab_graph);
extern int compare_two_world_states_ability_graphs(MY_GRAPH &ab_graph_1,MY_GRAPH &ab_graph_2, std::vector<std::string> &changes);
extern int insert_object_to_exclude_from_MA(std::string obj_name);
extern int find_potential_actions_for_this_source_target_env_pair(world_state_configs &WS1,world_state_configs &WS2, std::vector<World_state_change_explanation> &explanations);
extern int find_potential_actions_for_this_source_target_env_pair_id(int WS1_id,int WS2_id, std::vector<World_state_change_explanation> &explanations);
extern int compare_two_world_states_id_physical_positions(int WS1_id,int WS2_id, std::vector<std::string> &physical_changes);
extern int compare_two_world_states_id_ability_graphs(int WS1_id,int WS2_id, std::vector<std::string> &ability_changes);
extern int compare_two_world_states_ids(int ws1_id, int ws2_id, World_State_Changes &res_changes);
extern int print_this_world_state_change(World_State_Changes &WS_changes);
extern int filter_world_state_based_on_agent_perspective(int MA_agent_type);
extern int addCurrentRobotGraspToList(p3d_rob *robot, p3d_rob *object, gpHand_properties &handProp, std::list<gpGrasp> &graspList);
extern int remove_this_object_out_of_scene_and_update_MA(char* obj_name, std::vector<double> &act_pos);
extern int put_this_object_here_and_update_MA(char* obj_name, std::vector<double> &act_pos);
extern int get_sub_trajectory_names_for_plan_ID(int HRI_task_plan_id, std::vector<std::string> &traj_names);
extern int delete_this_world_state(world_state_configs &curr_WS);
extern int MA_Display_Functions();

#endif /* __CEXTRACT__ */

