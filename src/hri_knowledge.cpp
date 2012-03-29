#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "hri.h"

HRI_ENTITIES * GLOBAL_ENTITIES = NULL;



// hacked value used to compute reachability for humans because we currently have problem to use exact GIK
// for herakles. Work around could be to replace HERAKLES by ACHILE while doing the computation. :
double HRI_KNOWLEDGE_HUMAN_REACHABILITY_DISTANCE = 0.8;
/// object is considered to have disappear if it is undetected a numer of time equual to this parameter without any reason :
int HRI_KNOWLEDGE_MAX_UNEXPLAINED_DETECTION_VALUE = 7;

// to change threshold that will decide wether object is on furniture to account
// for localization issues
double HRI_KNOWLEDGE_IS_ON_MAX_DIST = 0.05;

HRI_KNOWLEDGE * hri_create_empty_agent_knowledge(HRI_AGENT * hri_agent)
{
    HRI_KNOWLEDGE * kn;

    kn = MY_ALLOC(HRI_KNOWLEDGE, 1);

    /* kn->points_at = 0; */
    /* kn->points_at_nb = 0; */
    /* kn->looks_at = NULL; */
    /* kn->looks_at_nb = 0; */
    kn->entities = NULL;
    kn->entities_nb = 0;

    return kn;
}

// Entities is a list of things that have some interest for spatial reasoning. For ex. objects, human body parts, surfaces, etc
// We consider all move3d robots except virtual ones (VISBALL for ex) and all move3d objects containing the words "surface, hand, head, camera"
// TODO: The cool thing would be to give a list of important things in the p3d file
//       and make this function take that list
HRI_ENTITIES * hri_create_entities()
{
    p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
    int i, j;
    int ent_i;
    HRI_ENTITIES * entities;
    char* objectrealname;

    entities = MY_ALLOC(HRI_ENTITIES, 1);
    entities->entities = NULL;
    ent_i = 0;
    entities->eventsInTheWorld = FALSE;
    entities->lastEventsInTheWorldStep = 4; //TODO use a constant
    entities->isWorldStatic = TRUE;
    entities->maxWorldNotStaticBeforeRecompute = 50;
    entities->numSuccessiveWorldNotStaticSinceLastRecompute = 0;
    entities->forbidWorldStatic = FALSE; /* allow to forbid go back to isWorldStatic = TRUE to inhibit situation assessment processing */
    entities->needSituationAssessmentUpdate = FALSE;
    entities->needLooksatUpdate = FALSE;
    entities->general_allow_disappear = TRUE;
    entities->printVisibilityImages = FALSE;
    entities->printSARecomputeStatus = FALSE;
    entities->needToUpdateParameters = FALSE;
    entities->manageDivergentBeliefs = false;
    entities->numParameterToUpdate = 0;
    entities->hackedReachabilityDistance = 0.8;
    entities->maxUnexplainedUndetectionIter = 7;
    entities->isOnThreshold = 0.05;
    entities->specialSupportEntityIndex = -1;

    for(i=0; i<env->nr; i++) {
        if(!strcasestr(env->robot[i]->name,"GRIPPER") && !strcasestr(env->robot[i]->name,"VISBALL") && !strcasestr(env->robot[i]->name,"SAHandRight")) {

            if(!strcasestr(env->robot[i]->name,"CHAIR")) {
                entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
                entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
                strcpy(entities->entities[ent_i]->name, env->robot[i]->name);

                entities->entities[ent_i]->indexInEnv = i;
                entities->entities[ent_i]->is_fixed = FALSE;
                entities->entities[ent_i]->is_present = FALSE;
                entities->entities[ent_i]->is_detected = FALSE;
                entities->entities[ent_i]->detection_time = 0;
                entities->entities[ent_i]->last_detection_time = 0;
                entities->entities[ent_i]->undetection_iter = 0;
                entities->entities[ent_i]->undetection_status = HRI_NEVER_DETECTED;
                entities->entities[ent_i]->visibility_percentage = 0.0;
                entities->entities[ent_i]->can_disappear_and_move = FALSE;
                entities->entities[ent_i]->disappeared = FALSE;
                entities->entities[ent_i]->allow_disappear = TRUE;
                entities->entities[ent_i]->hasInferrence = FALSE;
                entities->entities[ent_i]->inferrenceType = HRI_NO_INFERRENCE;
                strcpy(entities->entities[ent_i]->inferrenceObjectOrAgentPartName,"");
                entities->entities[ent_i]->agentPartNum = 0;
                entities->entities[ent_i]->inferrenceValidity = HRI_NO_PROBABILITY;
                entities->entities[ent_i]->perceptionInferrenceConflictThresholdMultiply = 0.0;
                entities->entities[ent_i]->perceptionInferrenceConflictThreshold = 0.0;
                entities->entities[ent_i]->perceptionInferrenceConflictValue = 0.0;

                entities->entities[ent_i]->last_ismoving_iter = 0;
                entities->entities[ent_i]->filtered_motion = HRI_STATIC;
                entities->entities[ent_i]->minStaticDist = 0;
                entities->entities[ent_i]->is_pl_state_transition_new = FALSE;
                entities->entities[ent_i]->pl_state_transition = HRI_UK_PL_STATE_TRANSITION;
                entities->entities[ent_i]->isOnSpecialSupport = FALSE;

                entities->entities[ent_i]->robotPt = env->robot[i];
                entities->entities[ent_i]->partPt = NULL;
                entities->entities[ent_i]->type = HRI_OBJECT;
                if(strcasestr(env->robot[i]->name,"TABLE")||strcasestr(env->robot[i]->name,"SHELF")){
                    entities->entities[ent_i]->subtype = HRI_OBJECT_SUPPORT;
                    //Temporary Hack to get isOn in poster for specific furniture.
                    if(strcasestr(env->robot[i]->name,"HRP2TABLE"))
                        entities->specialSupportEntityIndex = ent_i;
                }
                else if(strcasestr(env->robot[i]->name,"TAPE")||strcasestr(env->robot[i]->name,"BOTTLE")||strcasestr(env->robot[i]->name,"BOX")||strcasestr(env->robot[i]->name,"CUBE")||strcasestr(env->robot[i]->name,"KIT")){
                    entities->entities[ent_i]->subtype = HRI_MOVABLE_OBJECT;
                    entities->entities[ent_i]->can_disappear_and_move = TRUE;
                }
                else if(strcasestr(env->robot[i]->name,"TRASHBIN"))
                    entities->entities[ent_i]->subtype = HRI_OBJECT_CONTAINER;
                else if(strcasestr(env->robot[i]->name,"PLACEMAT"))
                    entities->entities[ent_i]->subtype = HRI_OBJECT_PLACEMAT;
                else
                    entities->entities[ent_i]->subtype = HRI_UK_ENTITY_SUBTYPE;
                ent_i++;
            }

            /// We get the GHOST object part to have correct values for Bounding Boxes.
            /// matthieu warnier 13 may 2011 : Gkuka6 and rarm7 are dirty hacks to have one hand for robot as well for Jido and Pr2. I should
            /// manage this better by adding something in the  p3d files
            for(j=0; j<env->robot[i]->no; j++) {
                objectrealname = strrchr(env->robot[i]->o[j]->name, '.')+1;
                if((strcasestr(objectrealname,"GHOST") || strcasestr(objectrealname,"G")) &&
                   (strcasestr(objectrealname,"SURFACE") || strcasestr(objectrealname,"HAND") ||
                    strcasestr(objectrealname,"HEAD")    ||  strcasestr(objectrealname,"CAMERA") || strcasestr(objectrealname,"Gkuka6") || strcasestr(objectrealname,"Grarm7")) ) {
                    entities->entities = MY_REALLOC(entities->entities, HRI_ENTITY*, ent_i, ent_i+1);
                    entities->entities[ent_i] = MY_ALLOC(HRI_ENTITY,1);
                    strcpy(entities->entities[ent_i]->name, objectrealname);
                    entities->entities[ent_i]->can_disappear_and_move = FALSE;
                    entities->entities[ent_i]->is_present = FALSE;
                    entities->entities[ent_i]->disappeared = FALSE;
                    entities->entities[ent_i]->robotPt = env->robot[i];
                    entities->entities[ent_i]->partPt = env->robot[i]->o[j];
                    entities->entities[ent_i]->type = HRI_OBJECT_PART;
                    if(strcasestr(env->robot[i]->name,"HEAD")||strcasestr(env->robot[i]->name,"CAMERA"))
                        entities->entities[ent_i]->subtype = HRI_AGENT_HEAD;
                    else if(strcasestr(env->robot[i]->name,"HAND") || strcasestr(objectrealname,"Gkuka6") || strcasestr(objectrealname,"Grarm7"))
                        entities->entities[ent_i]->subtype = HRI_AGENT_HAND;
                    else
                        entities->entities[ent_i]->subtype = HRI_UK_ENTITY_SUBTYPE;
                    ent_i++;
                }
            }
        }
    }

    entities->entities_nb = ent_i;

    return entities;
}

/* By default the entity structure doesn't attached to agents */
/* The user can call following function to link entities to agents */
int hri_link_agents_with_entities(HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
    int i;
    int is_human;
    int agent_idx;

    if(entities == NULL || agents == NULL)
        return FALSE;

    for(i=0; i<entities->entities_nb; i++) {
        switch (entities->entities[i]->type) {
        case HRI_OBJECT:
            if(hri_is_robot_an_agent(entities->entities[i]->robotPt, agents, &is_human, &agent_idx)) {
                entities->entities[i]->type = HRI_ISAGENT;
                entities->entities[i]->agent_idx = agent_idx;
                agents->all_agents[agent_idx]->entity_idx = i;
            }
            break;
      case HRI_OBJECT_PART:
            if(hri_is_robot_an_agent(entities->entities[i]->robotPt, agents, &is_human, &agent_idx)) {
                entities->entities[i]->type = HRI_AGENT_PART;
                entities->entities[i]->agent_idx = agent_idx;
                if(strcasestr(entities->entities[i]->partPt->name, "head") || strcasestr(entities->entities[i]->partPt->name, "camera")) {
                    agents->all_agents[agent_idx]->head = MY_REALLOC(agents->all_agents[agent_idx]->head, HRI_ENTITY*,
                                                                     agents->all_agents[agent_idx]->head_nb,
                                                                     agents->all_agents[agent_idx]->head_nb+1);
                    agents->all_agents[agent_idx]->head[agents->all_agents[agent_idx]->head_nb++] = entities->entities[i];

                }
                if(strcasestr(entities->entities[i]->partPt->name, "hand") || strcasestr(entities->entities[i]->partPt->name, "Gkuka6") || strcasestr(entities->entities[i]->partPt->name, "Grarm7") ) {
                    agents->all_agents[agent_idx]->hand = MY_REALLOC(agents->all_agents[agent_idx]->hand, HRI_ENTITY*,
                                                                     agents->all_agents[agent_idx]->hand_nb,
                                                                     agents->all_agents[agent_idx]->hand_nb+1);
                    agents->all_agents[agent_idx]->hand[agents->all_agents[agent_idx]->hand_nb++] = entities->entities[i];
                }
            }
            break;
      default:
            break;
        }
    }

    return TRUE;
}


/* This functions switch is_present value of entities linked to one agent (agent itself and agent hands and heads) */
int hriSetEntitiesPresenceFromAgentPresence(HRI_ENTITIES * entities,HRI_AGENT * agent , int presence)
{
    int i;

    if(agent == NULL)
        return FALSE;

    if(agent->agentEntitiesPresence == presence)
        return TRUE;

    // Change presence for entity agent
    // Test uninitialized entity_idx value. Why?
    if(agent->entity_idx >= 0 && agent->entity_idx<entities->entities_nb)
        entities->entities[agent->entity_idx]->is_present = TRUE;
    else {
        if(agent->entity_idx >= -1){
            // We print the error message only once
            printf("entity_idx of agent %s has unvalid value %d \n",agent->robotPt->name,agent->entity_idx);
            agent->entity_idx = -2;
        }
    }


    // Change presence for entities that are heads of agent.
    // Add test to prevent spark crashing. Why?
    if(agent->head_nb < 3){
        for(i=0; i<agent->head_nb; i++) {
            agent->head[i]->is_present = presence;
        }
    }
    else{
        if(agent->head_nb >= 3){
            // We print the error message only once
            printf("head_nb of agent %s has unvalid value %d\n",agent->robotPt->name,agent->head_nb);
            agent->head_nb = 0;
        }
    }

    // Change presence for entities that are hands of agent.
    // Add test to prevent spark crashing. Why?
    if(agent->hand_nb < 3){
        for(i=0; i<agent->hand_nb; i++) {
            agent->hand[i]->is_present = presence;
        }
    }
    else{
        if(agent->hand_nb >= 3){
            // We print the error message only once
            printf("hand_nb of agent %s has unvalid value %d\n",agent->robotPt->name,agent->hand_nb);
            agent->hand_nb = 0;
        }
    }


    agent->agentEntitiesPresence = presence;

    return TRUE;
}


int hri_initialize_all_agents_knowledge(HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
    int i;

    for(i=0; i<agents->all_agents_no; i++) {
        if (!hri_initialize_agent_knowledge(agents->all_agents[i]->knowledge, entities, agents)) {
            return FALSE;
        }
    }
    return TRUE;
}

int hri_initialize_agent_knowledge(HRI_KNOWLEDGE * knowledge, HRI_ENTITIES * entities, HRI_AGENTS * agents)
{
    int i, j;

    if((knowledge == NULL) || (entities == NULL) || (agents == NULL))
        return FALSE;

    knowledge->entities = MY_ALLOC(HRI_KNOWLEDGE_ON_ENTITY, entities->entities_nb);
    knowledge->entities_nb = entities->entities_nb;

    for(i=0; i<knowledge->entities_nb; i++) {

        knowledge->entities[i].entPt = entities->entities[i];
	knowledge->entities[i].hasEntityPosition = false;
	knowledge->entities[i].isEntityPositionInModel = false;
	knowledge->entities[i].hasEntityPositionKnowledge = true; 

        knowledge->entities[i].disappeared_isexported = TRUE;
        knowledge->entities[i].presenceValueExported = FALSE;
        knowledge->entities[i].presence_isexported = FALSE;

        knowledge->entities[i].motion = HRI_UK_MOTION;
        knowledge->entities[i].motion_ischanged = FALSE;
        knowledge->entities[i].motion_isexported = FALSE;

        knowledge->entities[i].is_placed_from_visibility = HRI_UK_VIS_PLACE;
        knowledge->entities[i].visibility_placement_ischanged = FALSE;
        knowledge->entities[i].visibility_placement_isexported = FALSE;

        knowledge->entities[i].visibility = HRI_UK_VIS;
        knowledge->entities[i].visibility_ischanged = FALSE;
        knowledge->entities[i].visibility_isexported = FALSE;

        knowledge->entities[i].reachability = HRI_UK_REACHABILITY;
        knowledge->entities[i].reachability_ischanged = FALSE;
        knowledge->entities[i].reachability_isexported = FALSE;

        knowledge->entities[i].is_looked_at = HRI_UK_V;
        knowledge->entities[i].is_looked_at_ischanged = FALSE;
        knowledge->entities[i].is_looked_at_isexported = FALSE;

        knowledge->entities[i].isSeen = HRI_UK_V;
        knowledge->entities[i].isSeenischanged = FALSE;
        knowledge->entities[i].isSeenisexported = FALSE;

        knowledge->entities[i].is_pointed_at = HRI_UK_V;
        knowledge->entities[i].is_pointed_at_ischanged = FALSE;
        knowledge->entities[i].is_pointed_at_isexported = FALSE;

	///// New structure for divergent belief management.
        knowledge->entities[i].is_placed_from_visibilityBy = MY_ALLOC(HRI_VISIBILITY_PLACEMENT, agents->all_agents_no);
        knowledge->entities[i].visibility_placementBy_ischanged = MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].visibility_placementBy_isexported =  MY_ALLOC(int, agents->all_agents_no);

        knowledge->entities[i].visibilityBy = MY_ALLOC(HRI_VISIBILITY, agents->all_agents_no);
        knowledge->entities[i].visibilityBy_ischanged =  MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].visibilityBy_isexported =  MY_ALLOC(int, agents->all_agents_no);

        knowledge->entities[i].reachabilityBy = MY_ALLOC(HRI_REACHABILITY, agents->all_agents_no);
        knowledge->entities[i].reachabilityBy_ischanged =  MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].reachabilityBy_isexported =  MY_ALLOC(int, agents->all_agents_no);

        knowledge->entities[i].is_looked_atBy = MY_ALLOC(HRI_TRUE_FALSE_UK_V, agents->all_agents_no);
        knowledge->entities[i].is_looked_atBy_ischanged =  MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].is_looked_atBy_isexported =  MY_ALLOC(int, agents->all_agents_no);

        knowledge->entities[i].isSeenBy = MY_ALLOC(HRI_TRUE_FALSE_UK_V, agents->all_agents_no);
        knowledge->entities[i].isSeenByischanged =  MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].isSeenByisexported =  MY_ALLOC(int, agents->all_agents_no);

        knowledge->entities[i].is_pointed_atBy = MY_ALLOC(HRI_TRUE_FALSE_UK_V, agents->all_agents_no);
        knowledge->entities[i].is_pointed_atBy_ischanged =  MY_ALLOC(int, agents->all_agents_no);
        knowledge->entities[i].is_pointed_atBy_isexported =  MY_ALLOC(int, agents->all_agents_no);

	for(j=0; j<agents->all_agents_no; j++) {
	    knowledge->entities[i].is_placed_from_visibilityBy[j] = HRI_UK_VIS_PLACE;
	    knowledge->entities[i].visibility_placementBy_ischanged[j] = FALSE;
	    knowledge->entities[i].visibility_placementBy_isexported[j] = FALSE;

	    knowledge->entities[i].visibilityBy[j] = HRI_UK_VIS;
	    knowledge->entities[i].visibilityBy_ischanged[j] = FALSE;
	    knowledge->entities[i].visibilityBy_isexported[j] = FALSE;

	    knowledge->entities[i].reachabilityBy[j] = HRI_UK_REACHABILITY;
	    knowledge->entities[i].reachabilityBy_ischanged[j] = FALSE;
	    knowledge->entities[i].reachabilityBy_isexported[j] = FALSE;

	    knowledge->entities[i].is_looked_atBy[j] = HRI_UK_V;
	    knowledge->entities[i].is_looked_atBy_ischanged[j] = FALSE;
	    knowledge->entities[i].is_looked_atBy_isexported[j] = FALSE;

	    knowledge->entities[i].isSeenBy[j] = HRI_UK_V;
	    knowledge->entities[i].isSeenByischanged[j] = FALSE;
	    knowledge->entities[i].isSeenByisexported[j] = FALSE;

	    knowledge->entities[i].is_pointed_atBy[j] = HRI_UK_V;
	    knowledge->entities[i].is_pointed_atBy_ischanged[j] = FALSE;
	    knowledge->entities[i].is_pointed_atBy_isexported[j] = FALSE;
	}

        knowledge->entities[i].is_located_from_agent = HRI_UK_RELATION;
        knowledge->entities[i].is_front_behind_from_agent = HRI_UK_RELATION;
        knowledge->entities[i].is_left_right_from_agent = HRI_UK_RELATION;
        knowledge->entities[i].is_far_near_from_agent = HRI_UK_RELATION;
        knowledge->entities[i].spatial_relation_ischanged = FALSE;
        knowledge->entities[i].spatial_relation_isexported = FALSE;

        knowledge->entities[i].is_placed = MY_ALLOC(HRI_PLACEMENT_RELATION, entities->entities_nb);
        knowledge->entities[i].is_placed_old = MY_ALLOC(HRI_PLACEMENT_RELATION, entities->entities_nb);
        knowledge->entities[i].placement_relation_ischanged = MY_ALLOC(int, entities->entities_nb);
        knowledge->entities[i].placement_relation_isexported = MY_ALLOC(int, entities->entities_nb);

        knowledge->entities[i].is_placed_nb = entities->entities_nb;

        for(j=0; j<knowledge->entities[i].is_placed_nb; j++) {
            knowledge->entities[i].is_placed[j] = HRI_UK_PLR;
            knowledge->entities[i].is_placed_old[j] = HRI_UK_PLR;
            knowledge->entities[i].placement_relation_ischanged[j] = FALSE;
            knowledge->entities[i].placement_relation_isexported[j] = FALSE;
        }
    }

    return TRUE;
}

HRI_REACHABILITY hri_is_reachable_single_arm(HRI_ENTITY * object, HRI_AGENT *agent,HRI_GIK_TASK_TYPE task_type,p3d_vector3* Tcoord)
{
    int reached = FALSE;
    configPt q;
    HRI_REACHABILITY reachability;

    q  = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */

    reached = hri_agent_single_task_manip_move(agent, task_type , Tcoord, 0.02, &q);

    p3d_set_and_update_this_robot_conf(agent->robotPt, q);

    p3d_col_activate_robot(agent->robotPt);

    // Check the final config for collision
    // If it is in collision with the entity, then it is reached
    // If it is in collision with another thing then it is hard to reach
    // Otherwise unreachable

    if(!p3d_col_test_robot(agent->robotPt, 0)) {
        // There is a collision. Test if it's with the target.
        if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART))
            p3d_col_deactivate_obj(object->partPt);
        else
            p3d_col_deactivate_rob(object->robotPt);

        if(!p3d_col_test_robot(agent->robotPt, 0)) {
            // The robot colliding another object
            if(reached)
                reachability = HRI_HARDLY_REACHABLE;
            else
                reachability = HRI_UNREACHABLE;
        }
        else {
            // Robot collides only with the target
            // That means it is reached
            reachability = HRI_REACHABLE;
        }
        if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART))
            p3d_col_activate_obj(object->partPt);
        else
            p3d_col_activate_rob(object->robotPt);
    }
    else {
        if(reached) {
            reachability = HRI_REACHABLE;
        }
        else {
            reachability = HRI_UNREACHABLE;
        }
    }

    MY_FREE(q, double , agent->robotPt->nb_dof); /* FREE */

    return reachability;
}

// TODO: There is a serious problem on the bounding boxes of robot parts.
// I think the reason is that in a macro the first body is takeninto account ghost <->real switch
HRI_REACHABILITY hri_is_reachable(HRI_ENTITY * object, HRI_AGENT *agent)
{
    configPt qs,q;
    p3d_vector3 Tcoord;
    HRI_REACHABILITY reachability,reachabilitySecondArm;
    double distance = 0.0;

    // Target object
    if((object->type == HRI_OBJECT_PART) || (object->type == HRI_AGENT_PART)) {
        Tcoord[0] = (object->partPt->BB.xmax + object->partPt->BB.xmin)/2;
        Tcoord[1] = (object->partPt->BB.ymax + object->partPt->BB.ymin)/2;
        Tcoord[2] = (object->partPt->BB.zmax + object->partPt->BB.zmin)/2;
    }
    else {
        Tcoord[0] = (object->robotPt->BB.xmax + object->robotPt->BB.xmin)/2;
        Tcoord[1] = (object->robotPt->BB.ymax + object->robotPt->BB.ymin)/2;
        Tcoord[2] = (object->robotPt->BB.zmax + object->robotPt->BB.zmin)/2;
    }

    distance = DISTANCE3D(Tcoord[0],Tcoord[1],Tcoord[2],
                          (agent->robotPt->BB.xmax + agent->robotPt->BB.xmin)/2,
                          (agent->robotPt->BB.ymax + agent->robotPt->BB.ymin)/2,
                          (agent->robotPt->BB.zmax + agent->robotPt->BB.zmin)/2);

    if( distance > 1.5) {
        reachability = HRI_UNREACHABLE;
    }
    else {
        //
        // TO MODIFY!
        // Matthieu Warnier 06/09/2011
        // Simplify reachability processing for human
        // Very simple first draft. We only use distance. We should use Amit processing.
        //
        if(agent->is_human){
            if(distance < HRI_KNOWLEDGE_HUMAN_REACHABILITY_DISTANCE)
                reachability =  HRI_REACHABLE;
            else
                reachability =  HRI_UNREACHABLE;
        }
        else {
            qs = MY_ALLOC(double, agent->robotPt->nb_dof); /* ALLOC */


            p3d_get_robot_config_into(agent->robotPt, &qs); /* Saving agent config */

            reachability = hri_is_reachable_single_arm(object,agent,GIK_LATREACH,&Tcoord);

            // If agent has two arms and object not reachable with first hand we try with second
            if(agent->manip->type == TWO_ARMED && (reachability != HRI_REACHABLE)) {
                p3d_set_and_update_this_robot_conf(agent->robotPt, qs);
                reachabilitySecondArm = hri_is_reachable_single_arm(object,agent,GIK_RATREACH,&Tcoord);
                if(reachabilitySecondArm ==  HRI_REACHABLE) {
                    reachability =  HRI_REACHABLE;
                }
                else if(reachabilitySecondArm ==  HRI_HARDLY_REACHABLE){
                    reachability =  HRI_HARDLY_REACHABLE;
                }
            }

            p3d_set_and_update_this_robot_conf(agent->robotPt,qs);

            MY_FREE(qs, double, agent->robotPt->nb_dof); /* FREE */
        }
    }
    return reachability;
}

/* Computes Placement relations (Ison, isin, etc) between two objects */
HRI_PLACEMENT_RELATION hri_placement_relation(p3d_rob *sourceObj, p3d_rob *targetObj)
{
    p3d_vector3 sourceObjC, targetObjC;

    if(sourceObj == NULL || targetObj == NULL) {
        printf("%s:%d plrelation input is null",__FILE__,__LINE__);
        return HRI_NOPLR;
    }

    sourceObjC[0] = (sourceObj->BB.xmin + sourceObj->BB.xmax)/2;
    sourceObjC[1] = (sourceObj->BB.ymin + sourceObj->BB.ymax)/2;
    sourceObjC[2] = (sourceObj->BB.zmin + sourceObj->BB.zmax)/2;

    targetObjC[0] = (targetObj->BB.xmin + targetObj->BB.xmax)/2;
    targetObjC[1] = (targetObj->BB.ymin + targetObj->BB.ymax)/2;
    targetObjC[2] = (targetObj->BB.zmin + targetObj->BB.zmax)/2;

    /* Test if source Obj is in targetObj */

    if( hri_is_in(sourceObjC, &targetObj->BB) )
        return HRI_ISIN;

    /* Test if sourceObj is on targetObj */

    if( hri_is_on(sourceObjC, &sourceObj->BB, &targetObj->BB) )
        return HRI_ISON;

    /* Test if sourceObj is next to targetObj */

    if ( hri_is_nexto(sourceObjC, &sourceObj->BB, targetObjC, &targetObj->BB) )
        return HRI_ISNEXTTO;

    return HRI_NOPLR;
}

/* Computes Placement relations (Ison, isin, etc) between two entities */
HRI_PLACEMENT_RELATION hri_placement_relation(HRI_ENTITY *sourceObj, HRI_ENTITY *targetObj)
{
    p3d_vector3 sourceObjC, targetObjC;
    p3d_BB * sourceBB, *targetBB;

    if(sourceObj == NULL || targetObj == NULL) {
        printf("%s:%d plrelation input is null",__FILE__,__LINE__);
        return HRI_NOPLR;
    }

    if((sourceObj->type == HRI_OBJECT_PART) || (sourceObj->type == HRI_AGENT_PART) ) {
        sourceObjC[0] = (sourceObj->partPt->BB.xmax + sourceObj->partPt->BB.xmin)/2;
        sourceObjC[1] = (sourceObj->partPt->BB.ymax + sourceObj->partPt->BB.ymin)/2;
        sourceObjC[2] = (sourceObj->partPt->BB.zmax + sourceObj->partPt->BB.zmin)/2;
        sourceBB = &sourceObj->partPt->BB;
    }
    else {
        sourceObjC[0] = (sourceObj->robotPt->BB.xmax + sourceObj->robotPt->BB.xmin)/2;
        sourceObjC[1] = (sourceObj->robotPt->BB.ymax + sourceObj->robotPt->BB.ymin)/2;
        sourceObjC[2] = (sourceObj->robotPt->BB.zmax + sourceObj->robotPt->BB.zmin)/2;
        sourceBB = &sourceObj->robotPt->BB;
    }

    if((targetObj->type == HRI_OBJECT_PART) || (targetObj->type == HRI_AGENT_PART) ) {
        targetObjC[0] = (targetObj->partPt->BB.xmax + targetObj->partPt->BB.xmin)/2;
        targetObjC[1] = (targetObj->partPt->BB.ymax + targetObj->partPt->BB.ymin)/2;
        targetObjC[2] = (targetObj->partPt->BB.zmax + targetObj->partPt->BB.zmin)/2;
        targetBB = &targetObj->partPt->BB;
    }
    else {
        targetObjC[0] = (targetObj->robotPt->BB.xmax + targetObj->robotPt->BB.xmin)/2;
        targetObjC[1] = (targetObj->robotPt->BB.ymax + targetObj->robotPt->BB.ymin)/2;
        targetObjC[2] = (targetObj->robotPt->BB.zmax + targetObj->robotPt->BB.zmin)/2;
        targetBB = &targetObj->robotPt->BB;
    }

    /* TEST of placement relations */
    /* Relations are mutually exclusive */

    /* Test if source Obj is in targetObj */

    if( hri_is_in(sourceObjC, targetBB) )
        return HRI_ISIN;

    /* Test if sourceObj is on targetObj */

    if( hri_is_on(sourceObjC, sourceBB, targetBB) )
        return HRI_ISON;

    /* Test if sourceObj is next to targetObj */

    if ( hri_is_nexto(sourceObjC, sourceBB, targetObjC, targetBB) )
        return HRI_ISNEXTTO;

    return HRI_NOPLR;
}

/* Test if topObj is on bottomObj */
int hri_is_on(p3d_vector3 topObjC, p3d_BB *topObjBB, p3d_BB *bottomObjBB)
{
    /* Compute ON */

    /* Test if topObj is on bottomObj */
    /* Condition 1: The center of topObj BB should be in the x,y limits of bottomObj BB and higher than bottomObj maximum z limit */
    /* Condition 2: The lower part of topObj BB souldn not be (higher than 5 cm) and (lower than 5 cm) from the higher part of bottomObj BB */
    if((topObjC[0] >= bottomObjBB->xmin) && (topObjC[0] <= bottomObjBB->xmax) &&
       (topObjC[1] >= bottomObjBB->ymin) && (topObjC[1] <= bottomObjBB->ymax))
        //&& (topObjC[2] >= bottomObjBB->zmax)) --> commented to manage PLACEMAT
        if((topObjBB->zmin - bottomObjBB->zmax > -HRI_KNOWLEDGE_IS_ON_MAX_DIST) && (topObjBB->zmin-bottomObjBB->zmax < HRI_KNOWLEDGE_IS_ON_MAX_DIST))
            return TRUE;

    return FALSE;
}

/* Test if insideObj is in outsideObj */
int hri_is_in(p3d_vector3 insideObjC, p3d_BB *outsideObjBB)
{
    /* Compute IN */
    /* Test if insideObj is in outsideObj */
    /* Condition: insideObj center should be wholly in outsideObj BB */

    if((outsideObjBB->xmin <= insideObjC[0]) && (outsideObjBB->xmax >= insideObjC[0]) &&
       (outsideObjBB->ymin <= insideObjC[1]) && (outsideObjBB->ymax >= insideObjC[1]) &&
       (outsideObjBB->zmin <= insideObjC[2]) && (outsideObjBB->zmax >= insideObjC[2]))
        return TRUE;

    return FALSE;
}

/* Test if sourceObj is next to targetObj */
int hri_is_nexto(p3d_vector3 sourceC, p3d_BB *sourceBB, p3d_vector3 targetC, p3d_BB *targetBB)
{
    /* Compute NEXT */

    /* Test if sourceObj is next to targetObj */
    /* Condition 1: The Z values of sourceObj and targetObj BB's should intersect = One should not be wholly above the other */
    /* Condition 2: Distance(sourceObj,targetObj) should be less then a constant */

    if ( !(sourceBB->zmax <= targetBB->zmin || sourceBB->zmin >= targetBB->zmax) ) {
        if(DISTANCE2D(sourceC[0], sourceC[1], targetC[0], targetC[1]) <
           MAX(ABS(sourceBB->xmin-sourceBB->xmax), ABS(sourceBB->ymin-sourceBB->ymax)) &&
           DISTANCE2D(sourceC[0], sourceC[1], targetC[0], targetC[1]) >
           MAX(ABS(sourceBB->xmin-sourceBB->xmax)/2, ABS(sourceBB->ymin-sourceBB->ymax)/2)) {

            return TRUE;
        }
    }

    if ( !(targetBB->zmax <= sourceBB->zmin || targetBB->zmin >= sourceBB->zmax) ) {
        if(DISTANCE2D(targetC[0], targetC[1], sourceC[0], sourceC[1]) <
           MAX(ABS(targetBB->xmin-targetBB->xmax), ABS(targetBB->ymin-targetBB->ymax)) &&
           DISTANCE2D(targetC[0], targetC[1], sourceC[0], sourceC[1]) >
           MAX(ABS(targetBB->xmin-targetBB->xmax)/2, ABS(targetBB->ymin-targetBB->ymax)/2)) {

            return TRUE;
        }
    }

    return FALSE;
}

HRI_SPATIAL_RELATION hri_spatial_relation(p3d_rob * object, p3d_rob * robot)
{
    p3d_vector4 targetRealCoord;
    p3d_vector4 targetRelativeCoord;
    p3d_matrix4 inv;
    double rho, phi, theta;
    int isFar;
    double frontAngle = 0.26; //TODO: Make this changeable
    double farLimit = 5.0; //TODO: Make this changeable

    if( (robot == NULL) || (object == NULL) ) {
        return HRI_NO_RELATION;
    }

    targetRealCoord[0] = object->joints[1]->abs_pos[0][3];
    targetRealCoord[1] = object->joints[1]->abs_pos[1][3];
    targetRealCoord[2] = object->joints[1]->abs_pos[2][3];
    targetRealCoord[3] = 1;

    p3d_matInvertXform(robot->joints[1]->abs_pos, inv);

    p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);

    p3d_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
                            &rho, &theta, &phi);


    isFar = (farLimit < DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));

    //  printf("real coord %f %f %f\n",targetRealCoord[0],targetRealCoord[1],targetRealCoord[2]);
    //  printf("relative coord %f %f %f\n",targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2]);
    //  printf("Phi is %f, isFar is %d\n",phi,isFar);

    /* Phi is the horizontal one */

    if(ABS(phi) < frontAngle) { /* In front */
        if(isFar) return HRI_FAR_FRONT ;
        else   return  HRI_NEAR_FRONT ;
    }
    if(frontAngle <= phi && phi < 3*M_PI/8) { /* Front left */
        if(isFar)  return  HRI_FAR_FRONT_LEFT ;
        else   return HRI_NEAR_FRONT_LEFT ;
    }
    if(3*M_PI/8 <= phi && phi < 5*M_PI/8) { /* left */
        if(isFar)   return HRI_FAR_LEFT ;
        else    return HRI_NEAR_LEFT ;
    }
    if(5*M_PI/8 <= phi && phi < 7*M_PI/8) { /* Back left */
        if(isFar)  return  HRI_FAR_BACK_LEFT ;
        else   return HRI_NEAR_BACK_LEFT ;
    }
    if(7*M_PI/8 <= ABS(phi)) { /* Back */
        if(isFar)  return  HRI_FAR_BACK ;
        else   return  HRI_NEAR_BACK ;
    }
    if(-7*M_PI/8 <= phi && phi < -5*M_PI/8) { /* Back right */
        if(isFar)  return HRI_FAR_BACK_RIGHT ;
        else  return HRI_NEAR_BACK_RIGHT ;
    }
    if(-5*M_PI/8 <= phi && phi < -3*M_PI/8) { /* right */
        if(isFar) return HRI_FAR_RIGHT ;
        else   return HRI_NEAR_RIGHT ;
    }
    if(-3*M_PI/8 <= phi && phi < -1*frontAngle) { /* Front right */
        if(isFar)  return HRI_FAR_FRONT_RIGHT ;
        else   return HRI_NEAR_FRONT_RIGHT ;
    }
    printf("Bad angle value, This shouldn't happen.\n");

    return HRI_NO_RELATION;
}

HRI_SPATIAL_RELATION hri_spatial_relation(HRI_ENTITY * object, HRI_AGENT * agent)
{
    p3d_vector4 targetRealCoord;
    p3d_vector4 targetRelativeCoord;
    p3d_matrix4 inv;
    p3d_BB *objectBB;
    double rho, phi, theta;
    int isFar;
    double frontAngle = 0.26; //TODO: Make this changeable
    double farLimit = 5.0; //TODO: Make this changeable

    if( (agent == NULL) || (object == NULL) ) {
        return HRI_NO_RELATION;
    }

    if(object->type == HRI_OBJECT_PART || object->type == HRI_AGENT_PART)
        objectBB = &object->partPt->BB;
    else
        objectBB = &object->robotPt->BB;

    targetRealCoord[0] = (objectBB->xmax+objectBB->xmin)/2;
    targetRealCoord[1] = (objectBB->ymax+objectBB->ymin)/2;
    targetRealCoord[2] = (objectBB->zmax+objectBB->zmin)/2;
    targetRealCoord[3] = 1;

    p3d_matInvertXform(agent->robotPt->joints[1]->abs_pos, inv);

    p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);

    p3d_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
                            &rho, &theta, &phi);


    isFar = (farLimit < DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));

    //  printf("real coord %f %f %f\n",targetRealCoord[0],targetRealCoord[1],targetRealCoord[2]);
    //  printf("relative coord %f %f %f\n",targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2]);
    //  printf("Phi is %f, isFar is %d\n",phi,isFar);

    /* Phi is the horizontal one */

    if(ABS(phi) < frontAngle) { /* In front */
        if(isFar) return HRI_FAR_FRONT ;
        else   return  HRI_NEAR_FRONT ;
    }
    if(frontAngle <= phi && phi < 3*M_PI/8) { /* Front left */
        if(isFar)  return  HRI_FAR_FRONT_LEFT ;
        else   return HRI_NEAR_FRONT_LEFT ;
    }
    if(3*M_PI/8 <= phi && phi < 5*M_PI/8) { /* left */
        if(isFar)   return HRI_FAR_LEFT ;
        else    return HRI_NEAR_LEFT ;
    }
    if(5*M_PI/8 <= phi && phi < 7*M_PI/8) { /* Back left */
        if(isFar)  return  HRI_FAR_BACK_LEFT ;
        else   return HRI_NEAR_BACK_LEFT ;
    }
    if(7*M_PI/8 <= ABS(phi)) { /* Back */
        if(isFar)  return  HRI_FAR_BACK ;
        else   return  HRI_NEAR_BACK ;
    }
    if(-7*M_PI/8 <= phi && phi < -5*M_PI/8) { /* Back right */
        if(isFar)  return HRI_FAR_BACK_RIGHT ;
        else  return HRI_NEAR_BACK_RIGHT ;
    }
    if(-5*M_PI/8 <= phi && phi < -3*M_PI/8) { /* right */
        if(isFar) return HRI_FAR_RIGHT ;
        else   return HRI_NEAR_RIGHT ;
    }
    if(-3*M_PI/8 <= phi && phi < -1*frontAngle) { /* Front right */
        if(isFar)  return HRI_FAR_FRONT_RIGHT ;
        else   return HRI_NEAR_FRONT_RIGHT ;
    }
    printf("Bad angle value, This shouldn't happen.\n");

    return HRI_NO_RELATION;
}

HRI_SPATIAL_RELATION hri_spatial_relation_new(HRI_ENTITY * object, HRI_AGENT * agent, HRI_SPATIAL_RELATION * front_behind, HRI_SPATIAL_RELATION * left_right , HRI_SPATIAL_RELATION * far_near , HRI_SPATIAL_RELATION  front_behind_old, HRI_SPATIAL_RELATION  left_right_old , HRI_SPATIAL_RELATION  far_near_old)
{
    p3d_vector4 targetRealCoord;
    p3d_vector4 targetRelativeCoord;
    p3d_matrix4 inv;
    p3d_BB *objectBB;
    double rho, phi, theta;
    int isFar;
    double farLimit = 5.0; //TODO: Make this changeable
    double frontAngleLimit = 3*M_PI/8;
    double behindAngleLimit = 5*M_PI/8;
    double leftRightMinAngleLimit = M_PI/8;
    double leftRightMaxAngleLimit = 7*M_PI/8;
    double hysteresisIncrease = 11/10;
    double hysteresisDecrease = 9/10;

    if( (agent == NULL) || (object == NULL) ) {
        return HRI_NO_RELATION;
    }

    if(object->type == HRI_OBJECT_PART || object->type == HRI_AGENT_PART)
        objectBB = &object->partPt->BB;
    else
        objectBB = &object->robotPt->BB;

    targetRealCoord[0] = (objectBB->xmax+objectBB->xmin)/2;
    targetRealCoord[1] = (objectBB->ymax+objectBB->ymin)/2;
    targetRealCoord[2] = (objectBB->zmax+objectBB->zmin)/2;
    targetRealCoord[3] = 1;

    p3d_matInvertXform(agent->robotPt->joints[1]->abs_pos, inv);

    p3d_matvec4Mult(inv, targetRealCoord, targetRelativeCoord);

    p3d_cartesian2spherical(targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2],
                            &rho, &theta, &phi);


    // Add hysteresis to avoid switch if HRI_FAR diminish farLimit else increase it.
    if(far_near_old == HRI_FAR)
        farLimit = farLimit*hysteresisDecrease;
    else if(far_near_old == HRI_NEAR)
        farLimit = farLimit*hysteresisIncrease;

    isFar = (farLimit < DISTANCE2D(targetRelativeCoord[0],targetRelativeCoord[1],0,0));
    if(isFar)
        *far_near = HRI_FAR;
    else
        *far_near = HRI_NEAR;
    //  printf("real coord %f %f %f\n",targetRealCoord[0],targetRealCoord[1],targetRealCoord[2]);
    //  printf("relative coord %f %f %f\n",targetRelativeCoord[0],targetRelativeCoord[1],targetRelativeCoord[2]);
    //  printf("Phi is %f, isFar is %d\n",phi,isFar);

    /* Phi is the horizontal one */
    if(front_behind_old == HRI_FRONT)
        frontAngleLimit =  frontAngleLimit*hysteresisIncrease;
    else if(front_behind_old == HRI_BEHIND)
        behindAngleLimit = behindAngleLimit*hysteresisDecrease;

    /* FRONT / BEHIND */
    if(ABS(phi) < frontAngleLimit) { /* In front */ // frontAngleLimit initial value 3*M_PI/8
        *front_behind = HRI_FRONT;
    }
    else if(ABS(phi) > behindAngleLimit) {  // behindAngleLimit initial value 5*M_PI/8
        *front_behind = HRI_BEHIND;
    }
    else
        *front_behind = HRI_NO_RELATION;

    if( (left_right_old == HRI_LEFT) || (left_right_old == HRI_RIGHT)){
        leftRightMinAngleLimit = leftRightMinAngleLimit*hysteresisDecrease;
        leftRightMaxAngleLimit = leftRightMaxAngleLimit*hysteresisIncrease;
    }

    /* LEFT / RIGHT */
    if((ABS(phi) > leftRightMinAngleLimit) && (ABS(phi) < leftRightMaxAngleLimit)) { /* In front */  // leftRightMinAngleLimit M_PI/8 ; leftRightMaxAngleLimit 7*M_PI/8
        if(phi > 0)
            *left_right = HRI_LEFT;
        else
            *left_right = HRI_RIGHT;
    }
    else
        *left_right = HRI_NO_RELATION;

    if(ABS(phi) > M_PI)
        printf("Bad angle value, This shouldn't happen.\n");

    return HRI_NO_RELATION;
}


/// This functions computes the inferred positions as the center of the bounding box of the other entity (object or agent parts)
int hri_set_XYZ_of_entity_at_center_of_other_entity(HRI_ENTITY *firstEntity, HRI_ENTITY *otherEntity){
    double vecX=0;
    double vecY=0;
    double vecZ=0;
    double x1,x2,y1,y2,z1,z2;

    //Compute vector between the two bounding box center.
    if((otherEntity->type == HRI_OBJECT_PART) || (otherEntity->type == HRI_AGENT_PART) ) {
        vecX = (otherEntity->partPt->BB.xmax + otherEntity->partPt->BB.xmin)/2;
        vecY = (otherEntity->partPt->BB.ymax + otherEntity->partPt->BB.ymin)/2;
        vecZ = (otherEntity->partPt->BB.zmax + otherEntity->partPt->BB.zmin)/2;
        //p3d_get_BB_obj(otherEntity->partPt, &x1, &x2, &y1, &y2, &z1, &z2);
        ///printf("objBB normal %s  xmax : %f ; xmin : %f ; ymax : %f ; ymin : %f ; zmin : %f ; zmax : %f .\n",otherEntity->name,otherEntity->partPt->BB.xmin,otherEntity->partPt->BB.xmax,otherEntity->partPt->BB.ymin,otherEntity->partPt->BB.ymax,otherEntity->partPt->BB.zmin,otherEntity->partPt->BB.zmax);
        // printf("objBB get processing %s  xmax : %f ; xmin : %f ; ymax : %f ; ymin : %f ; zmin : %f ; zmax : %f .\n",otherEntity->name,x1,x2,y1,y2,z1,z2);

    }
    else {
        vecX = (otherEntity->robotPt->BB.xmax + otherEntity->robotPt->BB.xmin)/2;
        vecY = (otherEntity->robotPt->BB.ymax + otherEntity->robotPt->BB.ymin)/2;
        vecZ = (otherEntity->robotPt->BB.zmax + otherEntity->robotPt->BB.zmin)/2;
    }

    // if((firstEntity->type == HRI_OBJECT_PART) || (firstEntity->type == HRI_AGENT_PART) ) {
    //   vecX -= (firstEntity->partPt->BB.xmax + firstEntity->partPt->BB.xmin)/2;
    //   vecY -= (firstEntity->partPt->BB.ymax + firstEntity->partPt->BB.ymin)/2;
    //   vecZ -= (firstEntity->partPt->BB.zmax + firstEntity->partPt->BB.zmin)/2;
    // }
    // else {
    //   vecX -= (firstEntity->robotPt->BB.xmax + firstEntity->robotPt->BB.xmin)/2;
    //   vecY -= (firstEntity->robotPt->BB.ymax + firstEntity->robotPt->BB.ymin)/2;
    //   vecZ -= (firstEntity->robotPt->BB.zmax + firstEntity->robotPt->BB.zmin)/2;
    // }

    //printf("set_XYZ_of_entity_at_center_of_other_entity VecX : %f ; VecY : %f ; VecZ : %f .\n",vecX,vecY,vecZ);

    //Apply this vector to x,y,z of object
    // firstEntity->robotPt->joints[1]->abs_pos[0][3] = firstEntity->robotPt->joints[1]->abs_pos[0][3] + vecX;
    // firstEntity->robotPt->joints[1]->abs_pos[1][3] = firstEntity->robotPt->joints[1]->abs_pos[1][3] + vecY;
    // firstEntity->robotPt->joints[1]->abs_pos[2][3] = firstEntity->robotPt->joints[1]->abs_pos[2][3] + vecZ;
    // firstEntity->infx = firstEntity->robotPt->joints[1]->abs_pos[0][3] + vecX;
    // firstEntity->infy = firstEntity->robotPt->joints[1]->abs_pos[1][3] + vecY;
    // firstEntity->infz = firstEntity->robotPt->joints[1]->abs_pos[2][3] + vecZ;
    firstEntity->infx =  vecX;
    firstEntity->infy =  vecY;
    firstEntity->infz =  vecZ;

    return TRUE;
}

/// This function assess the probability of an ongoing inferrence on position for an object that is perceived.
int hri_assess_perception_inferrence_conflict(HRI_ENTITY *firstEntity, HRI_ENTITY *otherEntity,double xPerception,double yPerception,double zPerception){
    double deltaX;
    double deltaY;
    double deltaZ;
    double deltaMaxFirst;
    double deltaMaxOther;
    double deltaSum;
    double xInferred;
    double yInferred;
    double zInferred;
    double distPerceptInferrence;

    //Compute max BB middle width for object whose position is inferred
    if((firstEntity->type == HRI_OBJECT_PART) || (firstEntity->type == HRI_AGENT_PART) ) {
        deltaX = (firstEntity->partPt->BB.xmax - firstEntity->partPt->BB.xmin)/2;
        deltaY = (firstEntity->partPt->BB.ymax - firstEntity->partPt->BB.ymin)/2;
        deltaZ = (firstEntity->partPt->BB.zmax - firstEntity->partPt->BB.zmin)/2;
    }
    else {
        deltaX = (firstEntity->robotPt->BB.xmax - firstEntity->robotPt->BB.xmin)/2;
        deltaY = (firstEntity->robotPt->BB.ymax - firstEntity->robotPt->BB.ymin)/2;
        deltaZ = (firstEntity->robotPt->BB.zmax - firstEntity->robotPt->BB.zmin)/2;
    }

    if(deltaX > deltaY)
        deltaMaxFirst = deltaX;
    else
        deltaMaxFirst = deltaY;

    if(deltaMaxFirst < deltaZ)
        deltaMaxFirst = deltaZ;

    //Compute max BB middle width for object or agent part that give the inferred position
    if((otherEntity->type == HRI_OBJECT_PART) || (otherEntity->type == HRI_AGENT_PART) ) {
        deltaX = (otherEntity->partPt->BB.xmax - otherEntity->partPt->BB.xmin)/2;
        deltaY = (otherEntity->partPt->BB.ymax - otherEntity->partPt->BB.ymin)/2;
        deltaZ = (otherEntity->partPt->BB.zmax - otherEntity->partPt->BB.zmin)/2;
    }
    else {
        deltaX = (otherEntity->robotPt->BB.xmax - otherEntity->robotPt->BB.xmin)/2;
        deltaY = (otherEntity->robotPt->BB.ymax - otherEntity->robotPt->BB.ymin)/2;
        deltaZ = (otherEntity->robotPt->BB.zmax - otherEntity->robotPt->BB.zmin)/2;
    }

    if(deltaX > deltaY)
        deltaMaxOther = deltaX;
    else
        deltaMaxOther = deltaY;

    if(deltaMaxOther < deltaZ)
        deltaMaxOther = deltaZ;

    deltaSum =  (deltaMaxFirst + deltaMaxOther)*firstEntity->perceptionInferrenceConflictThresholdMultiply;
    firstEntity->perceptionInferrenceConflictThreshold = deltaSum;

    //To be sure that x,y,z is the inferred value and not the last perceived value
    hri_set_XYZ_of_entity_at_center_of_other_entity(firstEntity,otherEntity);
    // xInferred = firstEntity->robotPt->joints[1]->abs_pos[0][3];
    // yInferred = firstEntity->robotPt->joints[1]->abs_pos[1][3];
    // zInferred = firstEntity->robotPt->joints[1]->abs_pos[2][3];
    xInferred = firstEntity->infx;
    yInferred = firstEntity->infy;
    zInferred = firstEntity->infz;
    distPerceptInferrence = DISTANCE3D(xInferred, yInferred, zInferred, xPerception, yPerception, zPerception );
    firstEntity->perceptionInferrenceConflictValue = distPerceptInferrence;
    if(distPerceptInferrence < deltaSum/4)
        firstEntity->inferrenceValidity = HRI_HIGHLY_PROBABLE;
    else if(distPerceptInferrence < deltaSum/3)
        firstEntity->inferrenceValidity = HRI_PROBABLE;
    else if(distPerceptInferrence < deltaSum/2)
        firstEntity->inferrenceValidity = HRI_AVERAGE;
    else if(distPerceptInferrence < deltaSum)
        firstEntity->inferrenceValidity = HRI_UNPROBABLE;
    else
        firstEntity->inferrenceValidity = HRI_HIGHLY_UNPROBABLE;
    return TRUE;
}


void hri_display_entities(HRI_ENTITIES * ents)
{
    int i;

    if(ents == NULL) {
        printf("ENTITIES not initialized\n");
        return ;
    }

    for(i=0; i<ents->entities_nb; i++) {
        printf("%d - ENTITY name: %s type: %d\n", i, ents->entities[i]->name, ents->entities[i]->type);
    }
}

void hri_display_agent_knowledge(HRI_AGENT * agent)
{
    int i, j;
    HRI_KNOWLEDGE * kn;

    if(agent == NULL || agent->knowledge == NULL) {
        printf("AGENT not initialized\n");
        return ;
    }

    kn = agent->knowledge;

    printf("\nFOR AGENT %s:\n", agent->robotPt->name);

    printf("\nKNOWLEDGE ON ENTITY ");
    for(i=0; i<kn->entities_nb; i++) {
        printf("%s\n", kn->entities[i].entPt->name);

        printf("Motion: %d\n", kn->entities[i].motion);
        printf("Visibility: %d\n",kn->entities[i].visibility);
        printf("Reachability: %d\n",kn->entities[i].reachability);
        printf("IsPlaced(vis): %d\n",kn->entities[i].is_placed_from_visibility);

        printf("IsLocated(byAgent): ");
        printf("%d",kn->entities[i].is_located_from_agent);

        printf("\nIsPlaced(Entities): ");
        for(j=0; j<kn->entities[i].is_placed_nb; j++)
            printf("%d,",kn->entities[i].is_placed[j]);

        printf("\n\n");
    }
}

// Function to init motion history array. Matthieu Warnier 01022011
//

/* int hri_init_motion_history(HRI_ENTITIES * ents, int ent_index) */
/* { */
/*   int i; */
/*   if(ent_index > -1 && ent_index < entities_nb) { */
/*     if(!ents->entities[ent_index]->can_move){ */
/*       ents->entities[ent_index]->motion_history = MY_ALLOC(HRI_MOTION, 3); */
/*       for(i=0;i<3;i++) */
/* 	ents->entities[ent_index]->motion_history[i]=HRI_UK_MOTION; */
/*       ents->entities[ent_index]->can_move = TRUE; */
/*     } */
/*     return TRUE; */
/*   }      */
/*   else */
/*     return FALSE; */
/* } */


// Function that decide and wethre an object should be
//
void hri_manage_object_disappearance_and_move(HRI_AGENTS * agents, HRI_ENTITIES * ents,int robotMyselfIndex)
{
    int e_i;
    HRI_AGENT * agent;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
    configPt objectQ;
    double visibility_percentage_threshold = 80;

    //Object moving and disappearance management.
    agent=agents->all_agents[robotMyselfIndex];

    for(e_i=0; e_i<ents->entities_nb; e_i++) {

        if(ents->entities[e_i]->can_disappear_and_move){
            // Is object detected?
            if(ents->entities[e_i]->is_detected){
                if(ents->entities[e_i]->detection_time != ents->entities[e_i]->last_detection_time){
                    if(!ents->entities[e_i]->is_present)
                        ents->entities[e_i]->is_present = TRUE;
                    /// reinitialize visibility percentage for undetection management
                    if( ents->entities[e_i]->visibility_percentage > 0 )
                        ents->entities[e_i]->visibility_percentage = 0;
                    if(ents->entities[e_i]->undetection_status != HRI_DETECTED){
                        ents->entities[e_i]->undetection_status = HRI_DETECTED;
                        ents->entities[e_i]->undetection_iter = 0;
                    }

                    if(ents->entities[e_i]->disappeared){
                        //APPEAR we put object has static.
                        ents->entities[e_i]->disappeared = FALSE;
                        ents->entities[e_i]->filtered_motion = HRI_STATIC;
                        ents->entities[e_i]->last_ismoving_iter = 0; // Filter first VIMAN isMoving to avoid HRI_START_MOVING and direct HRI_STOP_MOVING
                        ents->eventsInTheWorld = TRUE;
                        ents->entities[e_i]->is_pl_state_transition_new = TRUE;
                        ents->entities[e_i]->pl_state_transition = HRI_APPEAR;
                        printf("%s APPEARED\n",ents->entities[e_i]->name);
                    }
                }
            }
            else{
                if( ents->entities[e_i]->undetection_status != HRI_NEVER_DETECTED){

                    kn_on_ent = &agent->knowledge->entities[e_i];
                    if(!ents->entities[e_i]->disappeared && ((kn_on_ent->is_placed_from_visibility == HRI_FOV) || (kn_on_ent->is_placed_from_visibility == HRI_FOA)) && (kn_on_ent->visibility == HRI_VISIBLE)){
                        if(ents->isWorldStatic){

                            if(ents->general_allow_disappear && ents->entities[e_i]->allow_disappear){
                                // specific test for this entity to assess percentage visibility
                                // Todo : when to recompute it. Not all the time but often enough.
                                if( ents->entities[e_i]->visibility_percentage == 0 ){
                                    g3d_compute_visibility_in_fov_for_suspect_undetected_entity( ents, e_i, agent,agents);
                                    printf("Disappear Management - Visibility percentage : %f for entity %s\n" ,ents->entities[e_i]->visibility_percentage , ents->entities[e_i]->name);
                                }

                                // We will consider that the object should be detected above a certain threshold
                                if( ents->entities[e_i]->visibility_percentage > visibility_percentage_threshold){

                                    // iter on unexplained detection
                                    if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_iter < HRI_KNOWLEDGE_MAX_UNEXPLAINED_DETECTION_VALUE))
                                        ents->entities[e_i]->undetection_iter++;
                                    //  reach maximum number of unexplained detection
                                    else if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_iter == HRI_KNOWLEDGE_MAX_UNEXPLAINED_DETECTION_VALUE))
                                        ents->entities[e_i]->undetection_status = HRI_UNEXPLAINED_UNDETECTION_MAX;
                                    //  initialize iteration on maximum unexplained detection
                                    else if((ents->entities[e_i]->undetection_status != HRI_UNEXPLAINED_UNDETECTION_ITER) && (ents->entities[e_i]->undetection_status != HRI_UNEXPLAINED_UNDETECTION_MAX)){
                                        ents->entities[e_i]->undetection_status = HRI_UNEXPLAINED_UNDETECTION_ITER;
                                        ents->entities[e_i]->undetection_iter = 0;
                                    }
                                    // Object has disappeared
                                    else if((ents->entities[e_i]->undetection_status == HRI_UNEXPLAINED_UNDETECTION_MAX )){
                                        ents->entities[e_i]->disappeared = TRUE;
                                        ents->eventsInTheWorld = TRUE;
                                        ents->entities[e_i]->is_pl_state_transition_new = TRUE;
                                        ents->entities[e_i]->pl_state_transition = HRI_DISAPPEAR;
                                        printf("%s HAS DISAPPEAR\n",ents->entities[e_i]->name);
                                        // Put all facts at unknown value
                                        hri_delete_all_facts_for_disappeared_entity(agents,ents,e_i);
                                        // put object in 0,0,0 if disappear.
                                        // Do it in spark to update the Spark Poster.
                                        // objectQ = MY_ALLOC(double, ents->entities[e_i]->robotPt->nb_dof); /* ALLOC */
                                        // p3d_get_robot_config_into(ents->entities[e_i]->robotPt, &objectQ);
                                        // objectQ[6] = objectQ[7] = objectQ[8] = 0;
                                        // p3d_set_and_update_this_robot_conf(ents->entities[e_i]->robotPt, objectQ);
                                        // MY_FREE(objectQ, double, ents->entities[e_i]->robotPt->nb_dof); /* FREE */
                                    }
                                    else
                                        printf("Unmanaged state for undetected objects in  hri_manage_object_disappearance_and_move function\n");
                                }
                                else {
                                    /** low percentage can explain undetection */
                                    if(!ents->entities[e_i]->disappeared)
                                        ents->entities[e_i]->undetection_status = HRI_EXPLAINED_UNDETECTION;
                                }
                            }
                            else {
                                /// we inhibit disappearance
                                ents->entities[e_i]->undetection_status = HRI_UNEXPLAINED_UNDETECTION_ITER;
                                ents->entities[e_i]->undetection_iter = 0;
                            }
                        }
                        else {
                            // World is not Static
                            // need to update visibility precentage for this entity for next disappear management.
                            ents->entities[e_i]->visibility_percentage = 0;
                        }
                    }
                    else {
                        if(!ents->entities[e_i]->disappeared)
                            ents->entities[e_i]->undetection_status = HRI_EXPLAINED_UNDETECTION;
                    }
                }
            }

            //Is Moving Management
            if(ents->entities[e_i]->is_present && !ents->entities[e_i]->disappeared){
                if((ents->entities[e_i]->last_ismoving_iter>0) && (ents->entities[e_i]->filtered_motion != HRI_MOVING)){
                    //START MOVING
                    ents->entities[e_i]->filtered_motion = HRI_MOVING;
                    ents->eventsInTheWorld = TRUE;
                    ents->entities[e_i]->is_pl_state_transition_new = TRUE;
                    ents->entities[e_i]->pl_state_transition = HRI_START_MOVING;
                    printf("%s START MOVING\n",ents->entities[e_i]->name);
                }
                else if ((ents->entities[e_i]->last_ismoving_iter == 0 ) && (ents->entities[e_i]->filtered_motion != HRI_STATIC)){
                    //STOP MOVING
                    ents->entities[e_i]->filtered_motion = HRI_STATIC;
                    ents->eventsInTheWorld = TRUE;
                    ents->entities[e_i]->is_pl_state_transition_new = TRUE;
                    ents->entities[e_i]->pl_state_transition = HRI_STOP_MOVING;
                    printf("%s STOP MOVING\n",ents->entities[e_i]->name);
                }
                else if ((ents->entities[e_i]->last_ismoving_iter == 0 ) && (ents->entities[e_i]->filtered_motion == HRI_STATIC)){
                    //Static, Nothing to do
                }
                else if ((ents->entities[e_i]->last_ismoving_iter > 0 ) && (ents->entities[e_i]->filtered_motion == HRI_MOVING)){
                    ///while moving we consider it as events in the world (world is not static ) but no placement state transition.
                    ents->eventsInTheWorld = TRUE;
                    printf("%s IS MOVING\n",ents->entities[e_i]->name);
                }
                else
                    printf("Impossible motion state %s for entity %d\n",ents->entities[e_i]->name,ents->entities[e_i]->filtered_motion);
                // increment last is moving seen
                if(ents->entities[e_i]->last_ismoving_iter>0)
                    ents->entities[e_i]->last_ismoving_iter--;
            }

        }
    }

    // Computing all Situation assessment after each event Appear, Start Moving, Stop Moving and disappear can be costly and delay reading the state of the world ( object, human, robot ) as these events can be quite often folowed shortly one by another. Heavy computations should be done only once the world is detected as "static". We wait four step without an event. This Four should be replaced by a constant.
    // We also inhibit reprocessing while objects or agent parts are moving ( we consider it as eventsInTheWorld ) We have to check wether it is not too restrictive.

    if(ents->eventsInTheWorld){
        ents->lastEventsInTheWorldStep = 0;
        ents->isWorldStatic = FALSE;
        ents->needSituationAssessmentUpdate = TRUE;
    }
    else {
        if(ents->lastEventsInTheWorldStep == 4){
            if(!ents->forbidWorldStatic)
                ents->isWorldStatic = TRUE;
        }
        else
            ents->lastEventsInTheWorldStep ++;
    }


}


// Function that put to unknown value every fact for disappeared entity
//

int hri_delete_all_facts_for_disappeared_entity(HRI_AGENTS * agents, HRI_ENTITIES * ents,int disappearedEntityIndex)
{
    int a_i, a_j,e_i, e_j, ge_j;
    HRI_ENTITY * ent, ** present_ents;
    int * present_ents_global_idxs;
    int present_ents_nb;
    HRI_AGENT * agent,*agent2;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;


    if(agents == NULL || ents == NULL) {
        printf("Not Initialized\n");
        return FALSE;
    }

    for(a_i=0; a_i<agents->all_agents_no; a_i++) {
        agent = agents->all_agents[a_i];

        // if(agent->is_present == FALSE)
        //     continue;

        kn_on_ent = &agent->knowledge->entities[disappearedEntityIndex];
        //visibility
        kn_on_ent->visibility =  HRI_UK_VIS;
        kn_on_ent->visibility_ischanged = TRUE;
        kn_on_ent->visibility_isexported = FALSE;

        kn_on_ent->motion = HRI_UK_MOTION;
        kn_on_ent->motion_ischanged = TRUE;
        kn_on_ent->motion_isexported = FALSE;

        kn_on_ent->is_looked_at = HRI_UK_V;
        kn_on_ent->is_looked_at_ischanged = TRUE;
        kn_on_ent->is_looked_at_isexported = FALSE;

        kn_on_ent->isSeen = HRI_UK_V;
        kn_on_ent->isSeenischanged = TRUE;
        kn_on_ent->isSeenisexported = FALSE;

        kn_on_ent->is_pointed_at = HRI_UK_V;
        kn_on_ent->is_pointed_at_ischanged = TRUE;
        kn_on_ent->is_pointed_at_isexported = FALSE;

        kn_on_ent->reachability = HRI_UK_REACHABILITY;
        kn_on_ent->reachability_ischanged = TRUE;
        kn_on_ent->reachability_isexported = FALSE;

	///
	if(ents->manageDivergentBeliefs){
	    for(a_j=0; a_j<agents->all_agents_no; a_j++) {
		agent2 = agents->all_agents[a_j];

		///We delete divergent position if any.
		///We assume that present agent has some knowledge on position.
		if(kn_on_ent->hasEntityPosition){
		    MY_FREE(kn_on_ent->entityPositionForAgent, double, ents->entities[e_i]->robotPt->nb_dof);
		    kn_on_ent->hasEntityPosition = false;
		    agent->knowledge->numDivergentPositions--;
		}
		if(agent2->is_present == TRUE)
		    kn_on_ent->hasEntityPositionKnowledge = true;
		else
		    kn_on_ent->hasEntityPositionKnowledge = true;

		    
		kn_on_ent->is_placed_from_visibilityBy[a_j] = HRI_UK_VIS_PLACE;
		kn_on_ent->visibility_placementBy_ischanged[a_j] = FALSE;
		kn_on_ent->visibility_placementBy_isexported[a_j] = FALSE;

		kn_on_ent->visibilityBy[a_j] = HRI_UK_VIS;
		kn_on_ent->visibilityBy_ischanged[a_j] = FALSE;
		kn_on_ent->visibilityBy_isexported[a_j] = FALSE;

		kn_on_ent->reachabilityBy[a_j] = HRI_UK_REACHABILITY;
		kn_on_ent->reachabilityBy_ischanged[a_j] = FALSE;
		kn_on_ent->reachabilityBy_isexported[a_j] = FALSE;

		kn_on_ent->is_looked_atBy[a_j] = HRI_UK_V;
		kn_on_ent->is_looked_atBy_ischanged[a_j] = FALSE;
		kn_on_ent->is_looked_atBy_isexported[a_j] = FALSE;

		kn_on_ent->isSeenBy[a_j] = HRI_UK_V;
		kn_on_ent->isSeenByischanged[a_j] = FALSE;
		kn_on_ent->isSeenByisexported[a_j] = FALSE;

		kn_on_ent->is_pointed_atBy[a_j] = HRI_UK_V;
		kn_on_ent->is_pointed_atBy_ischanged[a_j] = FALSE;
		kn_on_ent->is_pointed_atBy_isexported[a_j] = FALSE;
	    }
	}
	    
	kn_on_ent->is_located_from_agent = HRI_UK_RELATION;
        kn_on_ent->is_front_behind_from_agent = HRI_UK_RELATION;
        kn_on_ent->is_left_right_from_agent = HRI_UK_RELATION;
        kn_on_ent->is_far_near_from_agent = HRI_UK_RELATION;
        kn_on_ent->spatial_relation_ischanged = TRUE;
        kn_on_ent->spatial_relation_isexported = FALSE;

        // PLACEMENT RELATION
        // Pick entities that exist
        present_ents_nb = 0;
        present_ents = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
        present_ents_global_idxs = MY_ALLOC(int, ents->entities_nb); // ALLOC
        for(e_i=0; e_i<ents->entities_nb; e_i++) {
            // If the entity is a part of the current agent, we skip it since it doesn't make sense to compute it from his own point of view
            // TODO: Or does it?
            if( (ents->entities[e_i]->type == HRI_AGENT_PART) || (ents->entities[e_i]->type == HRI_ISAGENT) ) {
                if( agent == agents->all_agents[ents->entities[e_i]->agent_idx] )
                    continue;
            }
            if(ents->entities[e_i]->is_present) {
                present_ents[present_ents_nb] = ents->entities[e_i];
                present_ents_global_idxs[present_ents_nb] = e_i;
                present_ents_nb++;
            }
        }
        // PLACEMENT RELATION
        for(e_j=0; e_j<present_ents_nb; e_j++) {
            ge_j = present_ents_global_idxs[e_j];
            // do not compute placement relations that involve an agent or an agent part
            /* if( ((ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT)) || !ent->can_disappear_and_move || ((ents->entities[ge_j]->type == HRI_AGENT_PART) || (ents->entities[ge_j]->type == HRI_ISAGENT)) ) { */
            /*   continue; */
            /* } */

            // We want to know wether objects are on furniture, on placemat or inside a container
            // Wa also want to know on which furnitures are placemat
            if( ((ents->entities[disappearedEntityIndex]->subtype == HRI_MOVABLE_OBJECT) && ((ents->entities[ge_j]->subtype == HRI_MOVABLE_OBJECT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_CONTAINER) || (ents->entities[ge_j]->subtype == HRI_OBJECT_PLACEMAT))) || ((ents->entities[disappearedEntityIndex]->subtype == HRI_OBJECT_PLACEMAT) && (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT))) {

                if( e_j != disappearedEntityIndex) {
                    kn_on_ent->is_placed[ge_j] = HRI_UK_PLR;
                    kn_on_ent->placement_relation_ischanged[ge_j] = TRUE;
                    kn_on_ent->placement_relation_isexported[ge_j] = FALSE;
                }
            }
        }
        MY_FREE(present_ents, HRI_ENTITY*, ents->entities_nb); // FREE
        MY_FREE(present_ents_global_idxs, int, ents->entities_nb); // FREE
    }
}

/// To specifically delete facts of one agent ( not source agent ) because it has no position knowledge anymore.
/// It means we have divergent belief management.
int DeleteAllFactsOfAgentForThisEntity(HRI_AGENTS * agents,int agentIndex, HRI_ENTITIES * ents,int disappearedEntityIndex)
{
    int a_i, a_j,e_i, e_j, ge_j;
    HRI_ENTITY * ent, ** present_ents;
    int * present_ents_global_idxs;
    int present_ents_nb;
    HRI_AGENT * agent,*agent2;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;


    if(agents == NULL || ents == NULL) {
        printf("Not Initialized\n");
        return FALSE;
    }

    agent = agents->all_agents[agentIndex];

    // if(agent->is_present == FALSE)
    //     continue;

    kn_on_ent = &agent->knowledge->entities[disappearedEntityIndex];
    //We do not delete visibility,reachability,isSeen,Lookat ispointat as they actually belongs to source agent model.

    kn_on_ent->motion = HRI_UK_MOTION;
    kn_on_ent->motion_ischanged = TRUE;
    kn_on_ent->motion_isexported = FALSE;

    ///We must be 
    if(ents->manageDivergentBeliefs){
	for(a_j=0; a_j<agents->all_agents_no; a_j++) {
	    agent2 = agents->all_agents[a_j];

	    ///We delete divergent position if any.
	    ///We assume that present agent has some knowledge on position.
	    if(kn_on_ent->hasEntityPosition){
		MY_FREE(kn_on_ent->entityPositionForAgent, double, ents->entities[e_i]->robotPt->nb_dof);
		kn_on_ent->hasEntityPosition = false;
		agent->knowledge->numDivergentPositions--;
	    }
	    if(agent2->is_present == TRUE)
		kn_on_ent->hasEntityPositionKnowledge = true;
	    else
		kn_on_ent->hasEntityPositionKnowledge = true;

		    
	    kn_on_ent->is_placed_from_visibilityBy[a_j] = HRI_UK_VIS_PLACE;
	    kn_on_ent->visibility_placementBy_ischanged[a_j] = FALSE;
	    kn_on_ent->visibility_placementBy_isexported[a_j] = FALSE;

	    kn_on_ent->visibilityBy[a_j] = HRI_UK_VIS;
	    kn_on_ent->visibilityBy_ischanged[a_j] = FALSE;
	    kn_on_ent->visibilityBy_isexported[a_j] = FALSE;

	    kn_on_ent->reachabilityBy[a_j] = HRI_UK_REACHABILITY;
	    kn_on_ent->reachabilityBy_ischanged[a_j] = FALSE;
	    kn_on_ent->reachabilityBy_isexported[a_j] = FALSE;

	    kn_on_ent->is_looked_atBy[a_j] = HRI_UK_V;
	    kn_on_ent->is_looked_atBy_ischanged[a_j] = FALSE;
	    kn_on_ent->is_looked_atBy_isexported[a_j] = FALSE;

	    kn_on_ent->isSeenBy[a_j] = HRI_UK_V;
	    kn_on_ent->isSeenByischanged[a_j] = FALSE;
	    kn_on_ent->isSeenByisexported[a_j] = FALSE;

	    kn_on_ent->is_pointed_atBy[a_j] = HRI_UK_V;
	    kn_on_ent->is_pointed_atBy_ischanged[a_j] = FALSE;
	    kn_on_ent->is_pointed_atBy_isexported[a_j] = FALSE;
	}
    }
    else
        printf("DeleteAllFactsOfAgentForThisEntity function should be used only when divergentBelief management is on.\n");	
	    
    kn_on_ent->is_located_from_agent = HRI_UK_RELATION;
    kn_on_ent->is_front_behind_from_agent = HRI_UK_RELATION;
    kn_on_ent->is_left_right_from_agent = HRI_UK_RELATION;
    kn_on_ent->is_far_near_from_agent = HRI_UK_RELATION;
    kn_on_ent->spatial_relation_ischanged = TRUE;
    kn_on_ent->spatial_relation_isexported = FALSE;

    // PLACEMENT RELATION
    // Pick entities that exist
    present_ents_nb = 0;
    present_ents = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
    present_ents_global_idxs = MY_ALLOC(int, ents->entities_nb); // ALLOC
    for(e_i=0; e_i<ents->entities_nb; e_i++) {
	// If the entity is a part of the current agent, we skip it since it doesn't make sense to compute it from his own point of view
	// TODO: Or does it?
	if( (ents->entities[e_i]->type == HRI_AGENT_PART) || (ents->entities[e_i]->type == HRI_ISAGENT) ) {
	    if( agent == agents->all_agents[ents->entities[e_i]->agent_idx] )
		continue;
	}
	if(ents->entities[e_i]->is_present) {
	    present_ents[present_ents_nb] = ents->entities[e_i];
	    present_ents_global_idxs[present_ents_nb] = e_i;
	    present_ents_nb++;
	}
    }
    // PLACEMENT RELATION
    for(e_j=0; e_j<present_ents_nb; e_j++) {
	ge_j = present_ents_global_idxs[e_j];
	// do not compute placement relations that involve an agent or an agent part
	/* if( ((ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT)) || !ent->can_disappear_and_move || ((ents->entities[ge_j]->type == HRI_AGENT_PART) || (ents->entities[ge_j]->type == HRI_ISAGENT)) ) { */
	/*   continue; */
	/* } */

	// We want to know wether objects are on furniture, on placemat or inside a container
	// Wa also want to know on which furnitures are placemat
	if( ((ents->entities[disappearedEntityIndex]->subtype == HRI_MOVABLE_OBJECT) && ((ents->entities[ge_j]->subtype == HRI_MOVABLE_OBJECT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_CONTAINER) || (ents->entities[ge_j]->subtype == HRI_OBJECT_PLACEMAT))) || ((ents->entities[disappearedEntityIndex]->subtype == HRI_OBJECT_PLACEMAT) && (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT))) {

	    if( e_j != disappearedEntityIndex) {
		kn_on_ent->is_placed[ge_j] = HRI_UK_PLR;
		kn_on_ent->placement_relation_ischanged[ge_j] = TRUE;
		kn_on_ent->placement_relation_isexported[ge_j] = FALSE;
	    }
	}
    }
    MY_FREE(present_ents, HRI_ENTITY*, ents->entities_nb); // FREE
    MY_FREE(present_ents_global_idxs, int, ents->entities_nb); // FREE
}



///Divergent Belief Management. Save geometric position of manipulable objects for some agent. 
int SaveObjectsCurrentPositionForAgent(HRI_AGENT* agent, HRI_ENTITIES * ents){
    int e_i;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
    for(e_i=0; e_i<ents->entities_nb; e_i++) { 
	if(ents->entities[e_i]->is_present && (ents->entities[e_i]->subtype == HRI_MOVABLE_OBJECT)){
	    kn_on_ent = &agent->knowledge->entities[e_i];
	    /// save position if this agent and the robot both have the same belief about this position.
	    if( !kn_on_ent->hasEntityPosition && kn_on_ent->hasEntityPositionKnowledge && !ents->entities[e_i]->disappeared){   
		kn_on_ent->entityPositionForAgent = MY_ALLOC(double, ents->entities[e_i]->robotPt->nb_dof); /* ALLOC */
		p3d_get_robot_config_into(ents->entities[e_i]->robotPt, &kn_on_ent->entityPositionForAgent);
		kn_on_ent->hasEntityPosition = true;  
		agent->knowledge->numDivergentPositions++;
	    }
	}
    }
}

/// Divergent Belief Management. Reset entity position in model as this agent position 
int SetMainAgentEntityPositionInModel(HRI_AGENTS * agents, HRI_ENTITIES * ents){
    int e_i;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
    for(e_i=0; e_i<ents->entities_nb; e_i++) { 
	if(ents->entities[e_i]->is_present && (ents->entities[e_i]->subtype == HRI_MOVABLE_OBJECT)){
	    kn_on_ent = &agents->all_agents[agents->source_agent_idx]->knowledge->entities[e_i];
	    if(kn_on_ent->hasEntityPosition){
		p3d_set_and_update_this_robot_conf(ents->entities[e_i]->robotPt, kn_on_ent->entityPositionForAgent);
		MY_FREE(kn_on_ent->entityPositionForAgent, double, ents->entities[e_i]->robotPt->nb_dof);
		kn_on_ent->hasEntityPosition = false;
	    }
	}
    }
}

/// Divergent Belief Management. Set entity position in model as this agent position for all entity that have a different position than main agent one. Save main agent position to put back its position after.
int SetCurrentAgentEntityPositionsInModel(HRI_AGENTS * agents,int agentIndex, HRI_ENTITIES * ents){
    int e_i;
    HRI_KNOWLEDGE_ON_ENTITY* kn_on_ent,* sourceAgentKn_on_ent;;
    for(e_i=0; e_i<ents->entities_nb; e_i++) { 
	if(ents->entities[e_i]->is_present && (ents->entities[e_i]->subtype == HRI_MOVABLE_OBJECT)){
	    kn_on_ent = &agents->all_agents[agentIndex]->knowledge->entities[e_i];
	    if(kn_on_ent->hasEntityPosition){
		///Save current position for source agent.
		sourceAgentKn_on_ent = &agents->all_agents[agents->source_agent_idx]->knowledge->entities[e_i];
		sourceAgentKn_on_ent->entityPositionForAgent = MY_ALLOC(double, ents->entities[e_i]->robotPt->nb_dof); /* ALLOC */
		p3d_get_robot_config_into(ents->entities[e_i]->robotPt, &(sourceAgentKn_on_ent->entityPositionForAgent));
		sourceAgentKn_on_ent->hasEntityPosition = true;  
		
		///Set agent diverging position as position in models for facts computation
		p3d_set_and_update_this_robot_conf(ents->entities[e_i]->robotPt, kn_on_ent->entityPositionForAgent);
	    }
	}
    }
}

//Compare existing divergent positions with robot one . (Is agent position different from robot? Does agent see new position) 
int CompareCurrentAgentEntityPositionsWithRobotOnes(HRI_AGENTS * agents, int agentIndex, HRI_ENTITIES * ents,double distThreshold){
    int e_i;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent,*sourceAgentKn_on_ent;
    bool deleteDivergentPosition;
    double dist;
    HRI_AGENT * agent;
    agent = agents->all_agents[agentIndex];
    
    for(e_i=0; e_i<ents->entities_nb; e_i++) { 
	if(ents->entities[e_i]->is_present && (ents->entities[e_i]->subtype == HRI_MOVABLE_OBJECT)){
	    kn_on_ent = &(agents->all_agents[agentIndex]->knowledge->entities[e_i]);
	    if(kn_on_ent->hasEntityPosition){
		sourceAgentKn_on_ent = &agents->all_agents[agents->source_agent_idx]->knowledge->entities[e_i];
		
		/// Is object visible in robot model?
		if(sourceAgentKn_on_ent->visibilityBy[agentIndex] == HRI_VISIBLE)
		    deleteDivergentPosition = true;
	    
		/// Is divergent object position different enough from
		if(!deleteDivergentPosition){
		    dist = DISTANCE3D(ents->entities[e_i]->robotPt->joints[1]->abs_pos[0][3],
				      ents->entities[e_i]->robotPt->joints[1]->abs_pos[1][3],
				      ents->entities[e_i]->robotPt->joints[1]->abs_pos[2][3],
				      kn_on_ent->entityPositionForAgent[6],
				      kn_on_ent->entityPositionForAgent[7],
				      kn_on_ent->entityPositionForAgent[8]
				      );
		    if(dist < distThreshold)
			deleteDivergentPosition = true;
		}
		
		/// Delete position
		if(deleteDivergentPosition){
		    MY_FREE(kn_on_ent->entityPositionForAgent, double, ents->entities[e_i]->robotPt->nb_dof);
		    kn_on_ent->hasEntityPosition = false;
		    agent->knowledge->numDivergentPositions--;
		}       	
	    }
	    else if(!kn_on_ent->hasEntityPositionKnowledge){
		sourceAgentKn_on_ent = &agents->all_agents[agents->source_agent_idx]->knowledge->entities[e_i];
		/// Is object visible in robot model?
		if(sourceAgentKn_on_ent->visibilityBy[agentIndex] == HRI_VISIBLE){
		    kn_on_ent->hasEntityPositionKnowledge = true;
		    agent->knowledge->numUnknownPositions--;
		}
	    }	
	}
    }
}
int UpdateIsLookedAtValues(HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent,int firstAgentIndex,int secondAgentIndex,HRI_VISIBILITY_PLACEMENT visPointValue, bool divergentBeliefManagement){	    
    ///Populate simple structure
    if(secondAgentIndex == firstAgentIndex){
	// need to update is_exported , is_changed?
	kn_on_ent->is_placed_from_visibility = visPointValue;
	if ( visPointValue == HRI_FOA
	     && kn_on_ent->visibility  == HRI_VISIBLE) {

	    if (kn_on_ent->is_looked_at != HRI_TRUE_V){
		kn_on_ent->is_looked_at = HRI_TRUE_V;
		kn_on_ent->is_looked_at_ischanged = TRUE;
		kn_on_ent->is_looked_at_isexported = FALSE;
	    }
	}
	else {
	    if (kn_on_ent->is_looked_at == HRI_TRUE_V){
		kn_on_ent->is_looked_at_ischanged = TRUE;
		kn_on_ent->is_looked_at_isexported = FALSE;
	    }
	    if( visPointValue == HRI_UK_VIS_PLACE
		|| kn_on_ent->visibility  == HRI_UK_VIS) {
		kn_on_ent->is_looked_at = HRI_UK_V;
	    }
	    else {
		kn_on_ent->is_looked_at = HRI_FALSE_V;
	    }
	}
    }

    ///Populate divergent belief structure.
    if(divergentBeliefManagement){
	// need to update is_exported , is_changed?
	kn_on_ent->is_placed_from_visibilityBy[secondAgentIndex] = visPointValue;

	if ( visPointValue == HRI_FOA
	     && kn_on_ent->visibilityBy[secondAgentIndex]  == HRI_VISIBLE) {

	    if (kn_on_ent->is_looked_atBy[secondAgentIndex] != HRI_TRUE_V){
		kn_on_ent->is_looked_atBy[secondAgentIndex] = HRI_TRUE_V;
		kn_on_ent->is_looked_atBy_ischanged[secondAgentIndex] = TRUE;
		kn_on_ent->is_looked_atBy_isexported[secondAgentIndex] = FALSE;
	    }
	}
	else {
	    if (kn_on_ent->is_looked_atBy[secondAgentIndex] == HRI_TRUE_V){
		kn_on_ent->is_looked_atBy_ischanged[secondAgentIndex] = TRUE;
		kn_on_ent->is_looked_atBy_isexported[secondAgentIndex] = FALSE;
	    }
	    if( visPointValue == HRI_UK_VIS_PLACE
		|| kn_on_ent->visibilityBy[secondAgentIndex]  == HRI_UK_VIS) {
		kn_on_ent->is_looked_atBy[secondAgentIndex] = HRI_UK_V;
	    }
	    else {
		kn_on_ent->is_looked_atBy[secondAgentIndex] = HRI_FALSE_V;
	    }
	}
    }
}

int UpdateIsSeenValues(HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent,int firstAgentIndex,int secondAgentIndex,HRI_VISIBILITY_PLACEMENT visPointValue, bool divergentBeliefManagement){	    
    ///Populate simple structure
    if(secondAgentIndex == firstAgentIndex){
	if ( (visPointValue == HRI_FOA || visPointValue == HRI_FOV) &&
	     kn_on_ent->visibility  == HRI_VISIBLE) {

	    if (kn_on_ent->isSeen != HRI_TRUE_V){
		kn_on_ent->isSeen = HRI_TRUE_V;
		kn_on_ent->isSeenischanged = TRUE;
		kn_on_ent->isSeenisexported = FALSE;
	    }
	}
	else {

	    if (kn_on_ent->isSeen == HRI_TRUE_V){
		kn_on_ent->isSeenischanged = TRUE;
		kn_on_ent->isSeenisexported = FALSE;
	    }

	    if( visPointValue == HRI_UK_VIS_PLACE || kn_on_ent->visibility == HRI_UK_VIS) {
		kn_on_ent->isSeen = HRI_UK_V;
	    }
	    else {
		kn_on_ent->isSeen = HRI_FALSE_V;
	    }
	}
    }
    ///Populate divergent belief structure.
    if(divergentBeliefManagement){
	if ( (visPointValue == HRI_FOA || visPointValue == HRI_FOV) &&
	     kn_on_ent->visibilityBy[secondAgentIndex]  == HRI_VISIBLE) {

	    if (kn_on_ent->isSeenBy[secondAgentIndex] != HRI_TRUE_V){
		kn_on_ent->isSeenBy[secondAgentIndex] = HRI_TRUE_V;
		kn_on_ent->isSeenByischanged[secondAgentIndex] = TRUE;
		kn_on_ent->isSeenByisexported[secondAgentIndex] = FALSE;
	    }
	}
	else {

	    if (kn_on_ent->isSeenBy[secondAgentIndex] == HRI_TRUE_V){
		kn_on_ent->isSeenByischanged[secondAgentIndex] = TRUE;
		kn_on_ent->isSeenByisexported[secondAgentIndex] = FALSE;
	    }

	    if( visPointValue == HRI_UK_VIS_PLACE || kn_on_ent->visibilityBy[secondAgentIndex] == HRI_UK_VIS) {
		kn_on_ent->isSeenBy[secondAgentIndex] = HRI_UK_V;
	    }
	    else {
		kn_on_ent->isSeenBy[secondAgentIndex] = HRI_FALSE_V;
	    }
	}
    }
}

int UpdateIsPointedAtValues(HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent,int firstAgentIndex,int secondAgentIndex,HRI_VISIBILITY_PLACEMENT visPointValue, bool divergentBeliefManagement){
    ///Populate simple structure
    if(secondAgentIndex == firstAgentIndex){  
	if ( (visPointValue == HRI_FOV || visPointValue == HRI_FOA) &&
	     kn_on_ent->visibility == HRI_VISIBLE) {

	    if (kn_on_ent->is_pointed_at != HRI_TRUE_V){
		kn_on_ent->is_pointed_at = HRI_TRUE_V;
		kn_on_ent->is_pointed_at_ischanged = TRUE;
		kn_on_ent->is_pointed_at_isexported = FALSE;
	    }
	}
	else {
	    if (kn_on_ent->is_pointed_at == HRI_TRUE_V){
		kn_on_ent->is_pointed_at_ischanged = TRUE;
		kn_on_ent->is_pointed_at_isexported = FALSE;
	    }
	    if( (visPointValue == HRI_UK_VIS_PLACE) || (kn_on_ent->visibility  == HRI_UK_VIS))
		kn_on_ent->is_pointed_at = HRI_UK_V;
	    else
		kn_on_ent->is_pointed_at = HRI_FALSE_V;
	}
    }
    if(divergentBeliefManagement){
	if ( (visPointValue == HRI_FOV || visPointValue == HRI_FOA) &&
	     kn_on_ent->visibilityBy[secondAgentIndex] == HRI_VISIBLE) {

	    if (kn_on_ent->is_pointed_atBy[secondAgentIndex] != HRI_TRUE_V){
		kn_on_ent->is_pointed_atBy[secondAgentIndex] = HRI_TRUE_V;
		kn_on_ent->is_pointed_atBy_ischanged[secondAgentIndex] = TRUE;
		kn_on_ent->is_pointed_atBy_isexported[secondAgentIndex] = FALSE;
	    }
	}
	else {
	    if (kn_on_ent->is_pointed_atBy[secondAgentIndex] == HRI_TRUE_V){
		kn_on_ent->is_pointed_atBy_ischanged[secondAgentIndex] = TRUE;
		kn_on_ent->is_pointed_atBy_isexported[secondAgentIndex] = FALSE;
	    }
	    if( (visPointValue == HRI_UK_VIS_PLACE) || (kn_on_ent->visibilityBy[secondAgentIndex]  == HRI_UK_VIS))
		kn_on_ent->is_pointed_atBy[secondAgentIndex] = HRI_UK_V;
	    else
		kn_on_ent->is_pointed_atBy[secondAgentIndex] = HRI_FALSE_V;
	}
    }	    
}

int UpdateReachabilityValues(HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent,int firstAgentIndex,int secondAgentIndex,HRI_REACHABILITY reachability_result, bool divergentBeliefManagement){
    ///Populate simple structure
    if(secondAgentIndex == firstAgentIndex){  
	if ( kn_on_ent->reachability ==  reachability_result) {
	    if ( kn_on_ent->reachability_ischanged)
		kn_on_ent->reachability_ischanged = FALSE;
	}
	else {
	    kn_on_ent->reachability = reachability_result;
	    kn_on_ent->reachability_ischanged = TRUE;
	    kn_on_ent->reachability_isexported = FALSE;
	}
    }
    if(divergentBeliefManagement){
	if ( kn_on_ent->reachabilityBy[secondAgentIndex] ==  reachability_result) {
	    if ( kn_on_ent->reachabilityBy_ischanged[secondAgentIndex])
		kn_on_ent->reachabilityBy_ischanged[secondAgentIndex] = FALSE;
	}
	else {
	    kn_on_ent->reachabilityBy[secondAgentIndex] = reachability_result;
	    kn_on_ent->reachabilityBy_ischanged[secondAgentIndex] = TRUE;
	    kn_on_ent->reachabilityBy_isexported[secondAgentIndex] = FALSE;
	}
    }
}

// Function computing geometric facts between agents and objects
// Each agent has its own view of the environment.

int hri_compute_geometric_facts(HRI_AGENTS * agents, HRI_ENTITIES * ents, int robotMyselfIndex)
{
    int a_i, a_j,a_k, e_i, e_j, ge_i, ge_j;
    double elevation, azimuth;
    HRI_ENTITY * ent, ** present_ents, ** present_entsAgent2;
    int * present_ents_global_idxs,* present_ents_global_idxsAgent2;
    int present_ents_nb,present_ents_nbAgent2;
    HRI_AGENT * agent,* agent2,* sourceAgent;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
    ENUM_HRI_VISIBILITY_PLACEMENT res;
    int counter = 0;
    HRI_VISIBILITY * vis_result;
    HRI_REACHABILITY reachability_result;
    HRI_SPATIAL_RELATION spatial_relation_result;
    HRI_SPATIAL_RELATION far_near_result;
    HRI_SPATIAL_RELATION left_right_result;
    HRI_SPATIAL_RELATION front_behind_result;
    HRI_PLACEMENT_RELATION placement_relation_result;
    int forceRecomputation = FALSE;
    int specialSupportEntityIndex = -1;
    // int specialSupportEntityIndexInPresentEnts = -1;

    if(agents == NULL || ents == NULL) {
        printf("Not Initialized\n");
        return FALSE;
    }

    // Hack to get isOn relation for special support
    specialSupportEntityIndex = ents->specialSupportEntityIndex;
    //printf("specialSupportEntityIndex : %d\n" ,specialSupportEntityIndex);

    // update parameters used for situation if needed.
    if(ents->needToUpdateParameters){
        ents->needToUpdateParameters = FALSE;
        if(ents->numParameterToUpdate == 0)
            HRI_KNOWLEDGE_HUMAN_REACHABILITY_DISTANCE = ents->hackedReachabilityDistance;
        else if(ents->numParameterToUpdate == 1)
            HRI_KNOWLEDGE_MAX_UNEXPLAINED_DETECTION_VALUE = ents->maxUnexplainedUndetectionIter;
        else if(ents->numParameterToUpdate == 2)
            HRI_KNOWLEDGE_IS_ON_MAX_DIST = ents->isOnThreshold;
        else
            printf("Unmanaged value for numParameterToUpdate : %d\n",ents->numParameterToUpdate );
    }

    // force recomputation if world not static during a long time
    if(!ents->isWorldStatic){
        if(ents->numSuccessiveWorldNotStaticSinceLastRecompute > ents->maxWorldNotStaticBeforeRecompute){
            ents->numSuccessiveWorldNotStaticSinceLastRecompute = 0;
            forceRecomputation = TRUE;
        }
        else
            ents->numSuccessiveWorldNotStaticSinceLastRecompute++;
    }
    else
        ents->numSuccessiveWorldNotStaticSinceLastRecompute = 0;

    // Print to inform about recomputation status.
    if(ents->printSARecomputeStatus)
        printf("needSituationAssessmentUpdate : %d ; EventsInTheWorldStep : %d; isWorldStatic : %d ; successive not static : %d ; forceRecomputation : %d ; ; ;\n", ents->needSituationAssessmentUpdate,ents->eventsInTheWorld,ents->isWorldStatic,ents->numSuccessiveWorldNotStaticSinceLastRecompute,forceRecomputation);

    vis_result = MY_ALLOC(HRI_VISIBILITY, ents->entities_nb); // ALLOC
    present_ents = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
    present_ents_global_idxs = MY_ALLOC(int, ents->entities_nb); // ALLOC
    present_entsAgent2 = MY_ALLOC(HRI_ENTITY*, ents->entities_nb); // ALLOC
    present_ents_global_idxsAgent2 = MY_ALLOC(int, ents->entities_nb); // ALLOC
    sourceAgent=agents->all_agents[agents->source_agent_idx];

    for(a_j=0; a_j<agents->all_agents_no; a_j++) {
	//We want to be sure that facts for the main (perceiving) agent are computed first 

	if(a_j == 0)
	    a_i = agents->source_agent_idx;
	else if(a_j == agents->source_agent_idx)
	    a_i = 0;    
	else
	    a_i = a_j;
	    
	agent = agents->all_agents[a_i];
    
	if(agent->is_present == FALSE){
	    //Save current manipulable objects position for this agent if needed.
	    if( ents->manageDivergentBeliefs && agent->knowledge->needToSaveObjPositions){
		SaveObjectsCurrentPositionForAgent(agent,ents);
		agent->knowledge->needToSaveObjPositions = false;	
	    }
	    //a_i != agents->source_agent_idx){
	    continue;
	}
	else{

	    }
	}

        // Pick entities that exist
        present_ents_nb = 0;
        for(e_i=0; e_i<ents->entities_nb; e_i++) {
            if(ents->entities[e_i]->is_present) {
                present_ents[present_ents_nb] = ents->entities[e_i];
                present_ents_global_idxs[present_ents_nb] = e_i;
                present_ents_nb++;
            }
        }

	///
	if(ents->manageDivergentBeliefs){
	    //Assess existing divergent positions . (Is agent position different from robot? Does agent see new position)
	    if(agent->knowledge->numDivergentPositions>0 || agent->knowledge->numUnknownPositions>0)
		CompareCurrentAgentEntityPositionsWithRobotOnes(agents,a_i, ents,0.05);

	    ///Use divergent positions if any
	    if(agent->knowledge->numDivergentPositions>0)
		SetCurrentAgentEntityPositionsInModel(agents,a_i,ents);		

	    //Switch bool so that current manipulable objects position for this agent will be saved when agent disappears.
	    if(!agent->knowledge->needToSaveObjPositions && (a_i != agents->source_agent_idx)){
		agent->knowledge->needToSaveObjPositions = true;
	    }
	}

	if(ents->needSituationAssessmentUpdate && (ents->isWorldStatic || forceRecomputation)){
	    ///Visibility Computation
	    for(a_k=0; a_k<agents->all_agents_no; a_k++) {
		agent2 = agents->all_agents[a_k];
		if(agent2->is_present == TRUE){
		    /// We consider all agents only if divergent belief management is on
		    if(a_k == a_i || ents->manageDivergentBeliefs){
			/// We compute visibility once for robot and then only recompute for other agents who have some diferent positions.
			// Pick entities that exist and are not agent parts of this agent.
			present_ents_nbAgent2 = 0;
			for(e_i=0; e_i<ents->entities_nb; e_i++) {
			    // If the entity is a part of the current agent, we skip it since it doesn't make sense to compute it from his own point of 
			    if( (ents->entities[e_i]->type == HRI_AGENT_PART) || (ents->entities[e_i]->type == HRI_ISAGENT) ) {
				if( agent2 == agents->all_agents[ents->entities[e_i]->agent_idx] )
				    continue;
			    }
			    if(ents->entities[e_i]->is_present) {
				present_entsAgent2[present_ents_nbAgent2] = ents->entities[e_i];
				present_ents_global_idxsAgent2[present_ents_nbAgent2] = e_i;
				present_ents_nbAgent2++;
			    }
			}


			if(!ents->manageDivergentBeliefs || (a_i == agents->source_agent_idx) || (agent->knowledge->numDivergentPositions>0))
			    g3d_compute_visibility_for_given_entities(present_entsAgent2, agent2, vis_result, present_ents_nbAgent2);
			else{
			    ///No recomputation needed. Wecopy values from main robot
			    for(e_j=0; e_j<present_ents_nbAgent2; e_j++) {
				ge_j = present_ents_global_idxsAgent2[e_j];
				kn_on_ent = &sourceAgent->knowledge->entities[ge_j];
				vis_result[e_j] = kn_on_ent->visibilityBy[a_k];
			    }
			}
			///Populate simple structure
			if(a_k == a_i){
			    for(e_j=0; e_j<present_ents_nbAgent2; e_j++) {
				ge_j = present_ents_global_idxsAgent2[e_j];
				kn_on_ent = &agent->knowledge->entities[ge_j];
				if ( kn_on_ent->visibility  ==  vis_result[e_j]) {
				    if ( kn_on_ent->visibility_ischanged)
					kn_on_ent->visibility_ischanged  = FALSE;
				}
				else {
				    kn_on_ent->visibility = vis_result[e_j];
				    kn_on_ent->visibility_ischanged = TRUE;
				    kn_on_ent->visibility_isexported = FALSE;
				}
			    }
			}
			
			///Populate divergent belief structure
			if(ents->manageDivergentBeliefs){
			    for(e_j=0; e_j<present_ents_nbAgent2; e_j++) {
				ge_j = present_ents_global_idxsAgent2[e_j];
				kn_on_ent = &agent->knowledge->entities[ge_j];
				if ( kn_on_ent->visibilityBy[a_k]  ==  vis_result[e_j]) {
				    if ( kn_on_ent->visibilityBy_ischanged[a_k])
					kn_on_ent->visibilityBy_ischanged[a_k]  = FALSE;
				}
				else {
				    kn_on_ent->visibilityBy[a_k] = vis_result[e_j];
				    kn_on_ent->visibilityBy_ischanged[a_k] = TRUE;
				    kn_on_ent->visibilityBy_isexported[a_k] = FALSE;
				}
			    				    

				/// If agent has divergent position and should see entities for its position but doesn't see it then it knows it doesn't have position knowledge for this object.
				if((a_k == a_i) && (a_i != agents->source_agent_idx) && kn_on_ent->hasEntityPosition && (kn_on_ent->visibilityBy[a_k] ==  HRI_VISIBLE)){
				    MY_FREE(kn_on_ent->entityPositionForAgent, double, ents->entities[e_i]->robotPt->nb_dof);
				    kn_on_ent->hasEntityPosition = false;
				    agent->knowledge->numDivergentPositions--;
				    kn_on_ent->hasEntityPositionKnowledge = false;
				    agent->knowledge->numUnknownPositions++;
				    /// Delete All Facts for this entity in this agent model.
				    DeleteAllFactsOfAgentForThisEntity(agents,a_i,ents,e_i);
				}				
			    }
			}
		    }
		}
	    }
	}
	
	for(e_i=0; e_i<present_ents_nb; e_i++) {
            ge_i = present_ents_global_idxs[e_i];

            ent = ents->entities[ge_i];
            kn_on_ent = &agent->knowledge->entities[ge_i];

	    /// Do not compute facts if agent doen't have knowledge on this robot position.
	    if(!kn_on_ent->hasEntityPositionKnowledge)
		continue;

            //printf("Testing: %s with %s\n", agent->robotPt->name, ent->robotPt->name);

            //
            if(ent->is_pl_state_transition_new){
                if((ent->pl_state_transition == HRI_APPEAR) || (ent->pl_state_transition == HRI_DISAPPEAR)){
                    kn_on_ent->disappeared_isexported = FALSE;
                }
                else if(((ent->pl_state_transition == HRI_START_MOVING) || (ent->pl_state_transition == HRI_STOP_MOVING) || (ent->pl_state_transition == HRI_APPEAR)) && (a_i == robotMyselfIndex)){
                    kn_on_ent->motion = ent->filtered_motion;
                    kn_on_ent->motion_ischanged = TRUE;
                    kn_on_ent->motion_isexported = FALSE;
                }

            }

	    for(a_k=0; a_k<agents->all_agents_no; a_k++) {
		agent2 = agents->all_agents[a_k];

		if(agent2->is_present == TRUE){
		    ///We consider this entity for this agent except if it is a part of this agent.
		    if( (ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT) ) {
			if( agent2 == agents->all_agents[ent->agent_idx] )
			    continue;
		    }

		    /// We consider all agents only if divergent belief management is on
		    if(a_k == a_i || ents->manageDivergentBeliefs){

			// LOOKS AT / VISIBILITY PLACEMENT - FOV,FOA,OOF
			// TODO: visibility placement for robot parts
			if(ent->disappeared)
			    res = HRI_UK_VIS_PLACE;
			else{
			    /// We compute once for robot and then only recompute for other agents who have some diferent positions.
			    if(!ents->manageDivergentBeliefs || (a_i == agents->source_agent_idx) || (agent->knowledge->numDivergentPositions>0)){
				HRI_VISIBILITY_PLACEMENT current_state =
				    ents->manageDivergentBeliefs ?  kn_on_ent->is_placed_from_visibilityBy[a_k] : kn_on_ent->is_placed_from_visibility;
				hri_entity_visibility_placement(agent,
								ent,
								true, current_state, // use hysteresis filtering
								&res,
								&elevation, &azimuth);
			    }
			    else
				res = ents->manageDivergentBeliefs ?  sourceAgent->knowledge->entities[ge_j].is_placed_from_visibilityBy[a_k] : sourceAgent->knowledge->entities[ge_j].is_placed_from_visibility;
			}
	    
			////////////////////////////////////////////
			// is_looked_at
			////////////////////////////////////////////
			UpdateIsLookedAtValues(kn_on_ent,a_i,a_j,res,ents->manageDivergentBeliefs);
			////////////////////////////////////////////
			// isSeen. We use the value processed above.
			/////////////////////////////////////////////
			UpdateIsSeenValues(kn_on_ent,a_i,a_j,res,ents->manageDivergentBeliefs);	    
	    
			// POINTS AT / POINTING PLACEMENT - FOV,FOA,OOF
			// TODO: visibility placement for robot parts
			if(ent->disappeared) {
			    res = HRI_UK_VIS_PLACE;
			}
			else {
			    /// We compute once for robot and then only recompute for other agents who have some diferent positions.	 
			    if(!ents->manageDivergentBeliefs || (a_i == agents->source_agent_idx) || (agent->knowledge->numDivergentPositions>0)){
				HRI_TRUE_FALSE_UK_V pointed_value = 
				    ents->manageDivergentBeliefs ?  kn_on_ent->is_pointed_atBy[a_k] : kn_on_ent->is_pointed_at;
				HRI_VISIBILITY_PLACEMENT current_state =
				    (pointed_value == HRI_TRUE_V) ? HRI_FOV : HRI_OOF;
				hri_entity_visibility_placement(agent,
								ent,
								true, current_state, // use hysteresis filtering
								&res,
								&elevation, &azimuth);
			    }
			    else{
				HRI_TRUE_FALSE_UK_V pointed_value = 
				    ents->manageDivergentBeliefs ?  sourceAgent->knowledge->entities[ge_j].is_pointed_atBy[a_k] : sourceAgent->knowledge->entities[ge_j].is_pointed_at;

				res = (pointed_value == HRI_TRUE_V) ? HRI_FOV : HRI_OOF;
			    }
			}	    
			// For pointing computation, FoA is set to 0.0 rad. Only FoV is actually useful.
			UpdateIsPointedAtValues(kn_on_ent,a_i,a_j,res,ents->manageDivergentBeliefs);
		    }
		}
	    }
		    
	    if(ents->needSituationAssessmentUpdate && (ents->isWorldStatic || forceRecomputation)){		
                // REACHABILITY - REACHABLE, UNREACHABLE, HARDLY REACHABLE
                // TODO: Fix this global variable use. It's ugly.
                // To simplify we do not compute reachability on agent or agent parts
                if ( (ent->type != HRI_AGENT_PART) && (ent->type != HRI_ISAGENT) && ent->can_disappear_and_move) {
		    for(a_k=0; a_k<agents->all_agents_no; a_k++) {
			agent2 = agents->all_agents[a_k];
			if(agent2->is_present == TRUE){
			    /// We consider all agents only if divergent belief management is on
			    if(a_k == a_i || ents->manageDivergentBeliefs){
				if(!ents->manageDivergentBeliefs || (a_i == agents->source_agent_idx) || (agent->knowledge->numDivergentPositions>0)){
				    GIK_VIS = 500;
				    if(ent->disappeared)
					reachability_result = HRI_UK_REACHABILITY;
				    else
					reachability_result = hri_is_reachable(ent, agent2);
				}
				else
				    reachability_result = sourceAgent->knowledge->entities[ge_j].reachabilityBy[a_k];
				
				UpdateReachabilityValues(kn_on_ent,a_i,a_k,reachability_result,ents->manageDivergentBeliefs);
			    }
			}
		    }
		}
	    }

            if(ents->needSituationAssessmentUpdate && (ents->isWorldStatic || forceRecomputation)){
                // SPATIAL RELATION
                if( ent->type != HRI_AGENT_PART) {
                    if(ent->disappeared){
                        spatial_relation_result = HRI_UK_RELATION;
                        front_behind_result = HRI_UK_RELATION;
                        left_right_result = HRI_UK_RELATION;
                        far_near_result = HRI_UK_RELATION;
                    }
                    else
                        spatial_relation_result = hri_spatial_relation_new(ent,
                                                                           agent,
                                                                           &front_behind_result ,
                                                                           &left_right_result ,
                                                                           &far_near_result,
                                                                           kn_on_ent->is_front_behind_from_agent,
                                                                           kn_on_ent->is_left_right_from_agent, 
                                                                           kn_on_ent->is_far_near_from_agent);

                    if (  ( kn_on_ent->is_front_behind_from_agent == front_behind_result) &&
                          ( kn_on_ent->is_left_right_from_agent == left_right_result) &&
                          ( kn_on_ent->is_far_near_from_agent == far_near_result)){
                        if (kn_on_ent->spatial_relation_ischanged)
                            kn_on_ent->spatial_relation_ischanged = FALSE;
                    }
                    else {
                        kn_on_ent->is_front_behind_from_agent = front_behind_result;
                        kn_on_ent->is_left_right_from_agent = left_right_result;
                        kn_on_ent->is_far_near_from_agent = far_near_result;
                        kn_on_ent->spatial_relation_ischanged = TRUE;
                        kn_on_ent->spatial_relation_isexported = FALSE;
                    }
                }
            }


            //
            // No inhibition of PLACEMENT RELATION processing while world is not static as it is considered as easy to process
            //

            // PLACEMENT RELATION
            for(e_j=0; e_j<present_ents_nb; e_j++) {
                ge_j = present_ents_global_idxs[e_j];

                // if( e_j == specialSupportEntityIndex)
                //   specialSupportEntityIndexInPresentEnts = ge_j;
                // do not compute placement relations that involve an agent or an agent part
                /* if( ((ent->type == HRI_AGENT_PART) || (ent->type == HRI_ISAGENT)) || !ent->can_disappear_and_move || ((ents->entities[ge_j]->type == HRI_AGENT_PART) || (ents->entities[ge_j]->type == HRI_ISAGENT)) ) { */
                /*   continue; */
                /* } */

                // We want to know wether objects are on furniture, on placemat or inside a container
                // Wa also want to know on which furnitures are placemat
                if( ((ent->subtype == HRI_MOVABLE_OBJECT) && ((ents->entities[ge_j]->subtype == HRI_MOVABLE_OBJECT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT) || (ents->entities[ge_j]->subtype == HRI_OBJECT_CONTAINER) || (ents->entities[ge_j]->subtype == HRI_OBJECT_PLACEMAT))) || ((ent->subtype == HRI_OBJECT_PLACEMAT) && (ents->entities[ge_j]->subtype == HRI_OBJECT_SUPPORT))) {

                    if( e_j != e_i) {

                        if(ent->disappeared || ents->entities[ge_j]->disappeared || !agent->knowledge->entities[ge_i].hasEntityPositionKnowledge)
                            placement_relation_result = HRI_UK_PLR;
                        else
                            placement_relation_result = hri_placement_relation(ent, ents->entities[ge_j]);
                        if (  kn_on_ent->is_placed[ge_j] ==  placement_relation_result) {
                            if ( kn_on_ent->placement_relation_ischanged[ge_j])
                                kn_on_ent->placement_relation_ischanged[ge_j] = FALSE;
                        }
                        else {
                            kn_on_ent->is_placed_old [ge_j] = kn_on_ent->is_placed[ge_j];
                            kn_on_ent->is_placed[ge_j] = placement_relation_result;
                            kn_on_ent->placement_relation_ischanged[ge_j] = TRUE;
                            kn_on_ent->placement_relation_isexported[ge_j] = FALSE;
                        }

                        if( ge_j == specialSupportEntityIndex){
                            //printf(" placement_relation_result : %d\n" ,placement_relation_result);
                            if( placement_relation_result == HRI_ISON)
                                ent->isOnSpecialSupport  = TRUE;
                            else
                                ent->isOnSpecialSupport  = FALSE;
                        }
                    }
                }
            }
        }
	//// entity positions should be the one of main agent.
	if(ents->manageDivergentBeliefs)
	    SetMainAgentEntityPositionInModel(agents,ents);

    // all placement state transition events have been managed
    for(e_i=0; e_i<present_ents_nb; e_i++) {
        ge_i = present_ents_global_idxs[e_i];
        ent = ents->entities[ge_i];
        if(ent->is_pl_state_transition_new)
            ent->is_pl_state_transition_new = FALSE;
    }

    MY_FREE(vis_result, HRI_VISIBILITY, ents->entities_nb); // FREE
    MY_FREE(present_ents, HRI_ENTITY*, ents->entities_nb); // FREE
    MY_FREE(present_ents_global_idxs, int, ents->entities_nb); // FREE
    MY_FREE(present_entsAgent2, HRI_ENTITY*, ents->entities_nb); // FREE
    MY_FREE(present_ents_global_idxsAgent2, int, ents->entities_nb); // FREE

    // Events in the Wolrd have been managed.
    if(ents->eventsInTheWorld)
        ents->eventsInTheWorld = FALSE;

    if(ents->needSituationAssessmentUpdate && (ents->isWorldStatic || forceRecomputation))
        ents->needSituationAssessmentUpdate = FALSE;
    if(ents->needLooksatUpdate)
        ents->needLooksatUpdate = FALSE;

    return counter;
}



/// Draw small spheres to show divergent positions.
void hri_draw_divergent_positions()
{
    int a_i,e_i;
    int nbDivergentPosition;
    HRI_KNOWLEDGE_ON_ENTITY * kn_on_ent;
    double x,y,z,r,opacity;
    GLdouble color[4];
    HRI_AGENT* agent;

    if((GLOBAL_AGENTS == NULL) || (GLOBAL_ENTITIES == NULL)){
	return;
    }
    if(!GLOBAL_ENTITIES->manageDivergentBeliefs){
	return;
    }
    

    for(a_i=0; a_i<GLOBAL_AGENTS->all_agents_no; a_i++) {
	//We want to be sure that facts for the main (perceiving) agent are computed first 

	/// not for source agent.
	if(a_i == GLOBAL_AGENTS->source_agent_idx)
	    continue;

	agent = GLOBAL_AGENTS->all_agents[a_i];

	/// only if agent is present.
	if(agent->is_present == FALSE)
	    continue;

	nbDivergentPosition = GLOBAL_AGENTS->all_agents[a_i]->knowledge->numDivergentPositions;

	for(e_i=0; e_i<GLOBAL_ENTITIES->entities_nb; e_i++) { 
	    if(nbDivergentPosition == 0)
		break;
	    if(GLOBAL_ENTITIES->entities[e_i]->is_present && (GLOBAL_ENTITIES->entities[e_i]->subtype == HRI_MOVABLE_OBJECT)){
		kn_on_ent = &(GLOBAL_AGENTS->all_agents[a_i]->knowledge->entities[e_i]);
		if(kn_on_ent->hasEntityPosition){
		    nbDivergentPosition--;
		    
		    x = kn_on_ent->entityPositionForAgent[6];
		    y = kn_on_ent->entityPositionForAgent[7];
		    z = kn_on_ent->entityPositionForAgent[8];
		    opacity = 1.0;
		    r = 0.1;
		    color[0] = 0.0; color[1]= 0.0; color[2]= 1.0; color[3]= opacity;
		    glEnable(GL_BLEND);
		    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		    g3d_set_color(Any,color);
		    g3d_draw_solid_sphere(x,y,z,r,20);		    
		    glDisable(GL_BLEND);///g3d_drawSphere(x,y,z,r);
		}
	    }      
	}
    }
}

