/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue May 12 14:41:50 2009
 */
#ifndef __CEXTRACT__

#include "../lightPlanner/proto/ManipulationPlanner.hpp"
#include "../lightPlanner/proto/ManipulationUtils.hpp"

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

extern int assign_weights_on_candidte_points_to_hide_obj(char *object_name, candidate_poins_for_task *candidate_points, int indx_by_agent, int indx_for_agent);

extern int reverse_sort_weighted_candidate_points_to_show_obj();
extern int reverse_sort_weighted_candidate_points_to_hide_obj();
extern int reverse_sort_weighted_candidate_points_to_give_obj();
extern int find_candidate_points_on_plane_to_hide_obj_new();
extern int find_candidate_points_to_show_obj_new();
extern int find_candidate_points_on_plane_to_put_obj();
extern int find_candidate_points_on_plane_to_hide_away_obj();
extern int find_candidate_points_on_plane_to_put_away_obj();
extern int find_candidate_points_to_give_obj();
extern int Create_and_init_Mightability_Maps();
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
extern int find_Mightability_Maps();
extern int JIDO_give_obj_to_human( char *obj_to_manipulate );
extern int find_candidate_points_for_current_HRI_task(HRI_TASK_TYPE curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, candidate_poins_for_task *resultant_candidate_point);
extern int reverse_sort_HRI_task_weighted_candidate_points(candidate_poins_for_task *candidate_points);
extern int JIDO_find_HRI_task_solution(HRI_TASK_TYPE CURR_TASK, HRI_TASK_AGENT for_agent, char *obj_to_manipulate);
extern int find_HRI_task_candidate_points(HRI_TASK_TYPE CURR_TASK, char *obj_to_manipulate, HRI_TASK_AGENT performed_by,  HRI_TASK_AGENT performed_for, int performing_agent_rank, candidate_poins_for_task *curr_resultant_candidate_points);
extern object_Symbolic_Mightability_Maps_Relation* create_object_oriented_Mightability_obj();
extern int delete_object_oriented_Mightability_obj(object_Symbolic_Mightability_Maps_Relation *OOM);
extern int copy_current_object_oriented_Mightability_into(object_Symbolic_Mightability_Maps_Relation* target);
extern int print_object_oriented_Mightability(object_Symbolic_Mightability_Maps_Relation* OOM);
extern int store_OOM_before_task();
extern int print_object_oriented_Mightability_for_object(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index);
extern int print_object_oriented_Mightability_for_object_by_agent(object_Symbolic_Mightability_Maps_Relation* OOM, int obj_index, HRI_TASK_AGENT agent);

extern int init_visibility_acceptance_for_tasks();
extern int JIDO_perform_task (char *obj_to_manipulate, HRI_TASK_TYPE task, HRI_TASK_AGENT by_agent, HRI_TASK_AGENT for_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> curr_placementList, traj_for_HRI_task &res_trajs);
extern int validate_HRI_task(HRI_task_desc curr_task, int task_plan_id, int for_proactive_info);
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
extern int init_manipulation_planner();
extern int update_3d_grid_reachability_for_agent_MM(HRI_TASK_AGENT for_agent, MA_agent_hand_name for_hand, int for_state);
extern int get_human_head_relative_yaw_pitch_for(HRI_TASK_AGENT for_agent, point_co_ordi for_point, double &relative_yaw, double &relative_pitch);
extern int find_MA_Agent_visibility(HRI_TASK_AGENT for_agent, char *obj_name, double &visibility_val);
extern int JIDO_find_solution_to_take(char *obj_to_manipulate, HRI_TASK_TYPE task,  HRI_TASK_AGENT from_agent, candidate_poins_for_task *curr_candidate_points, std::list<gpGrasp> graspList, std::list<gpPlacement> placementList, traj_for_HRI_task &res_trajs);
extern int find_human_give_candidate_points(HRI_TASK_AGENT performed_by);
extern int get_robot_proactive_solution_info( HRI_task_desc curr_task, traj_for_HRI_task &res_traj);
extern int show_hand_grasps_of_list(p3d_rob *hand, p3d_rob *object, std::list<gpGrasp> *graspList);
extern int get_grasp_list_for_object(char *obj_to_manipulate, std::list<gpGrasp> &graspList);
extern int show_all_how_to_placements_in_3D(point_co_ordi at_place, int use_random_colors, int skip, int semi_transparent, std::list<gpPlacement> *placement_config_list);
extern int get_placements_in_3D(char *obj_to_manipulate,  std::list<gpPlacement> &placementListOut);
extern int find_candidate_points_for_current_HRI_task_for_object(HRI_TASK_TYPE_ENUM curr_task, HRI_TASK_AGENT_ENUM performed_by, HRI_TASK_AGENT_ENUM performed_for, int performing_agent_rank, candidate_poins_for_task *resultant_candidate_point, char *object);
#endif /* __CEXTRACT__ */

