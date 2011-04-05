extern void hri_initialize_visibility();
extern int hri_is_object_visible(HRI_AGENT * agent,p3d_rob *object, int threshold, int save, int draw_at_end);
extern int g3d_is_object_visible_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *object, double *result, int save);
extern int g3d_get_given_entities_pixelpresence_from_viewpoint(p3d_matrix4 camera_frame, double camera_fov, HRI_ENTITY **objects, int objects_nb, bool display_others_in_blue,double *results, int save);
extern int g3d_is_object_visible_from_current_viewpoint(g3d_win* win, p3d_rob *object, double *result, int save, char *path);
extern int g3d_get_given_entities_pixelpresence_in_current_viewpoint(g3d_win* win, HRI_ENTITY **objects, int objects_nb, bool display_others_in_blue,double *vis_results, int save, char *path);
extern int g3d_visibility_for_given_objects_in_current_viewpoint_pixelpercentage(g3d_win* win, p3d_rob **objects, int objects_nb, double *res, int save, char *path);
extern int hri_object_visibility_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth);
extern int hri_entity_visibility_placement(HRI_AGENT *agent, HRI_ENTITY *ent, int *result, double *elevation, double *azimuth);
extern int hri_object_pointing_placement(HRI_AGENT *agent, p3d_rob *object, int *result, double *elevation, double *azimuth);
extern int hri_entity_pointing_placement(HRI_AGENT *agent, HRI_ENTITY *ent, int *result, double *elevation, double *azimuth);
extern int g3d_object_visibility_placement(p3d_matrix4 camera_frame, p3d_vector4 objectCenter, double Hfov, double Vfov, double Hfoa, double Vfoa, int *result, double *phi_result, double *theta_result);
extern int g3d_draw_agent_fov(HRI_AGENT *agent);
extern int g3d_draw_visibility_by_frame(p3d_matrix4 camera_frame, double Hfov, double Vfov, double max_dist, GLdouble source_color[],GLdouble dest_color[]);
extern int g3d_draw_agent_pointing(HRI_AGENT *agent);
extern int hri_is_object_pointed(HRI_AGENT * agent, p3d_rob *object, int threshold, int save);
extern void g3d_draw_all_agents_fovs(HRI_AGENTS *agents);
extern void p3d_cartesian2spherical(double x, double y, double z, double *rho, double *phi, double *theta);
extern void p3d_cartesian2spherical(double x, double y, double z, double originx, double originy, double originz, double *phi, double *theta);
extern int g3d_is_object_visible_from_current_viewpoint2(g3d_win* win, p3d_rob *object, double *result, int save, char *path);
extern int hri_compute_agent_sees(HRI_AGENT * agent, int threshold, int save, int draw_at_end);
extern int hri_turn_agent_head_direction(HRI_AGENT *agent, double elevation, double azimuth);
extern int g3d_compute_visibility_for_given_entities(HRI_ENTITY ** ents, HRI_AGENT * agent, HRI_VISIBILITY * res, int res_nb);
extern int g3d_compute_visibility_in_fov_for_given_entities(HRI_ENTITY ** ents,HRI_ENTITY * agent_entity , HRI_AGENT * agent, HRI_VISIBILITY * res, int res_nb);
extern int g3d_compute_visibility_in_fov_for_suspect_undetected_entity(HRI_ENTITIES * ents, int suspect_undetected_entity_index,HRI_AGENT * agent,HRI_AGENTS * agents);

extern double hri_simple_is_point_visible_by_robot(p3d_vector3 point,p3d_rob* robotPt);

