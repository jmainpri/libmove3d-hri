extern HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent);
extern HRI_ENTITIES * hri_create_entities();
extern int hri_link_agents_with_entities(HRI_ENTITIES * entities, HRI_AGENTS * agents);
extern int hriSetEntitiesPresenceFromAgentPresence(HRI_ENTITIES * entities,HRI_AGENT * agent , int presence);
extern HRI_REACHABILITY hri_is_reachable_single_arm(HRI_ENTITY * object, HRI_AGENT *agent,HRI_GIK_TASK_TYPE task_type,p3d_vector3* Tcoord);
extern HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent);
extern HRI_PLACEMENT_RELATION hri_placement_relation(p3d_rob *sourceObj, p3d_rob *targetObj);
extern int hri_is_on(p3d_vector3 topObjC, p3d_BB *topObjBB, p3d_BB *bottomObjBB);
extern int hri_is_in(p3d_vector3 insideObjC, p3d_BB *outsideObjBB);
extern int hri_is_nexto(p3d_vector3 sourceC, p3d_BB *sourceBB, p3d_vector3 targetC, p3d_BB *targetBB);
extern HRI_SPATIAL_RELATION hri_spatial_relation(p3d_rob * object, p3d_rob * robot);
extern HRI_SPATIAL_RELATION hri_spatial_relation(HRI_ENTITY * object, HRI_AGENT * agent);
extern HRI_SPATIAL_RELATION hri_spatial_relation_new(HRI_ENTITY * object, HRI_AGENT * agent, HRI_SPATIAL_RELATION * front_behind, HRI_SPATIAL_RELATION * left_right , HRI_SPATIAL_RELATION * far_near, HRI_SPATIAL_RELATION  front_behind_old, HRI_SPATIAL_RELATION  left_right_old , HRI_SPATIAL_RELATION  far_near_old);
extern int hri_set_XYZ_of_entity_at_center_of_other_entity(HRI_ENTITY *firstEntity, HRI_ENTITY *otherEntity);
extern int hri_assess_perception_inferrence_conflict(HRI_ENTITY *firstEntity, HRI_ENTITY *otherEntity,double xPerception,double yPerception,double zPerception);
extern void hri_manage_object_disappearance_and_move(HRI_AGENTS * agents, HRI_ENTITIES * ents,int robotMyselfIndex);
extern int hri_compute_geometric_facts(HRI_AGENTS * agents, HRI_ENTITIES * ents, int robotMyselfIndex);
extern int hri_delete_all_facts_for_disappeared_entity(HRI_AGENTS * agents, HRI_ENTITIES * ents,int disappearedEntityIndex);
extern int hri_initialize_agent_knowledge(HRI_KNOWLEDGE * knowledge, HRI_ENTITIES * entities, HRI_AGENTS * agents);
extern int hri_initialize_all_agents_knowledge(HRI_ENTITIES * entities, HRI_AGENTS * agents);
extern void hri_display_agent_knowledge(HRI_AGENT * agent);
extern void hri_display_entities(HRI_ENTITIES * ents);


