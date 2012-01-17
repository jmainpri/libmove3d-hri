/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.
 *
 *   Created: Tue May 27 14:48:29 2008
 */
#ifndef __CEXTRACT__
extern int hri_exp_get_robot_joint_object();
extern void hri_exp_set_exp_from_config(hri_bitmapset* btset, configPt q);
extern double hri_exp_combined_val ( hri_bitmapset* btset, int x, int y, int z );
extern double hri_exp_path_val ( hri_bitmapset* btset, int x, int y, int z );
extern double hri_exp_distance_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_vision_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_rreach_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_hcomfort_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_hlreach_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_hrreach_val ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_hlreach_val2 ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_hrreach_val2 ( hri_bitmapset * btset, int x, int y, int z );
extern double hri_exp_obstacle_val ( hri_bitmapset * btset, int x, int y, int z );
extern int hri_exp_fill_obstacles ( hri_bitmapset * btset );
extern int hri_exp_find_manip_path ( hri_bitmapset * btset );
extern int hri_exp_find_10_exchange_point ( hri_bitmapset * btset );
extern int hri_exp_find_exchange_point ( void );
extern hri_bitmapset* hri_exp_init ( void );
extern void hri_exp_save ( hri_bitmapset* btset, hri_bitmap * bitmap, char * name, double excld );
extern void hri_exp_save_npoint ( hri_bitmapset* btset, hri_bitmap * bitmap, char * name, int incl_zeros, int n );
extern void hri_exp_save_table ( char * name, double * val, int n );
extern void hri_exp_save_4tables ( char * name, int *x, int *y, int *z, double * val, int n );
extern void hri_exp_draw_ordered_points ( void );
extern int hri_exp_rrt_path ( double *qs, int *iksols, int *iksolg, int (*fct_stop)(void), void (*fct_draw)(void) );
extern int hri_expand_start_hri_rrt ( p3d_graph *G, hri_bitmapset * btset, int inode, p3d_node ** added_node, int *reached, int (*fct_stop)(void) );
extern int hri_shoot_with_btset ( p3d_rob *robotPt, hri_bitmapset * btset, int inode, configPt q );
extern int hri_expand_one_hri_rrt ( p3d_graph *G, p3d_compco **CompPt, p3d_node **node, int inode, configPt q );
extern int hri_shoot ( p3d_rob *robotPt, configPt q );
extern p3d_node *hri_nearest_neighbor ( p3d_rob *rob, configPt q, p3d_compco *comp );
extern int hri_expand_prm ( p3d_graph *G, hri_bitmapset * btset, int inode, p3d_node ** added_node, int *reached, int (*fct_stop)(void) );
extern int hri_add_basic_node ( p3d_graph *G, hri_bitmapset * btset, int inode, p3d_node ** added_node, int * fail, configPt q );
extern int hri_link_node_graph ( p3d_node* Node, p3d_graph* Graph );
extern int hri_link_node_comp ( p3d_graph *G, p3d_node *N, p3d_compco **compPt );
extern int hri_APInode_linked ( p3d_graph *graphPt, p3d_node *N1, p3d_node *N2, double *dist );
extern hri_bitmapset* hri_object_reach_init ( double objx, double objy, double objz );
extern int hri_exp_find_obj_reach_path ( hri_bitmapset * btset );
extern double hri_obj_reach_path_val ( hri_bitmapset* btset, int x, int y, int z );

extern void g3d_hri_display_surfaces( void );
extern void g3d_hri_display_test( void );

//OTP
extern bool hri_get_best_otp_position( p3d_rob* rob, p3d_rob* human, p3d_vector3 otp );

#endif /* __CEXTRACT__ */
