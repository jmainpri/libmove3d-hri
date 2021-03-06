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
#ifndef _HRI_AGENT_H
#define _HRI_AGENT_H

#include "hri_bitmap.h"
#include "hri_manip.h"
#include "hri_knowledge.h"

#define GIK_MAX_JOINT_NO 50
#define GIK_MAX_TASK_NO 5
#define GIK_CONSTRAINTS 3
#define HRI_ACHILE_NECKHEIGHT 1.4

//! @defgroup AGENTS Agents
//! A module for agents

//! @ingroup AGENTS
typedef enum HRI_AGENT_TYPE_ENUM {
    HRI_JIDO1,
    HRI_JIDOKUKA,
    HRI_PR2,
    HRI_HRP214,
    HRI_B21,
    HRI_JUSTIN,
    HRI_ICUB,
    HRI_BERT,
    HRI_SUPERMAN,
    HRI_OLDDUDE,
    HRI_ACHILE,
    HRI_HERAKLES,
    HRI_BIOMECH,
    HRI_TINMAN,
    HRI_BH,
    HRI_MOBILE_JUSTIN,
    HRI_XAVIER
}	HRI_AGENT_TYPE;

//! @ingroup AGENTS
typedef enum HRI_GIK_TASK_TYPE_ENUM {
    GIK_LOOK,
    GIK_RAREACH,
    GIK_LAREACH,
    GIK_RATREACH,
    GIK_LATREACH,
    GIK_RAWREACH,
    GIK_LAWREACH,
    GIK_RAPOINT,
    GIK_LAPOINT,
    GIK_RATPOINT,
    GIK_LATPOINT,
    GIK_NOTASK
} HRI_GIK_TASK_TYPE;

//! @ingroup AGENTS
typedef struct STRUCT_GIK_TASK {
    HRI_GIK_TASK_TYPE type;
    int default_joints[GIK_MAX_JOINT_NO];
    int default_joints_no;
    int actual_joints[GIK_MAX_JOINT_NO];
    int actual_joints_no;
    int active_joint;
    double target[GIK_CONSTRAINTS];
} GIK_TASK;

//! @ingroup AGENTS
typedef enum HRI_MANIP_TYPE_ENUM {
    ONE_ARMED,
    TWO_ARMED
} HRI_MANIP_TYPE;

//! @ingroup AGENTS
typedef struct STRUCT_HRI_MANIP {
    hri_gik * gik;
    HRI_MANIP_TYPE type;

    signed int gik_max_step; /* TODO: add these two to hri_gik structure */
    double reach_tolerance;

    GIK_TASK * tasklist;
    int tasklist_no;

    int activetasks[GIK_MAX_TASK_NO];
    int activetasks_no;
} HRI_MANIP;

//! @ingroup AGENTS
typedef struct STRUCT_HRI_NAVIG {
    hri_bitmapset * btset;
    int btset_initialized;
} HRI_NAVIG;

//! @ingroup AGENTS
typedef struct STRUCT_HRI_PERSP {
    /* Visual Perspective */
    p3d_jnt * camjoint;
    double fov;
    double foa;
    int pan_jnt_idx;
    int tilt_jnt_idx;
    HRI_VISIBILITY_LIST currently_sees;

    /* Reachability Perspective */
    p3d_jnt * pointjoint;
    double point_tolerance;

    /* Graphics */
    int enable_vision_draw;
    int enable_pointing_draw;
    int enable_visible_objects_draw;
} HRI_PERSP;

/* --------------- */
//! @ingroup AGENTS
typedef enum ENUM_HRI_AGENT_POSTURE {
    HRI_STANDING = 0,
    HRI_SITTING = 1
              } HRI_AGENT_POSTURE;

//! @ingroup AGENTS
typedef struct STRUCT_HRI_AGENT {
    HRI_AGENT_TYPE type;
    int is_human; /* TODO: put something more generic, at least an enum. I don't like these stupid flags. */
    p3d_rob * robotPt;
    HRI_MANIP * manip;
    HRI_NAVIG * navig;
    HRI_PERSP * perspective;
    HRI_KNOWLEDGE * knowledge;
    int is_present;
    /* number of possible states for this agent (e.g. SITTING/STANDING) */
    int states_no;
    int actual_state;
    /* possible states */
    /* TODO: change the name human_state -> agent state */
    hri_human_state * state;

    /* Link with entities */
    int entity_idx;
    HRI_ENTITY ** head; /* Table containing indexes of head entities */
    int head_nb; /* a Human cannot have 2 hands, but a robot can have multiple cameras */
    HRI_ENTITY ** hand;
    int hand_nb;
    int agentEntitiesPresence;

    char * object_name;
    int is_grasping_object;
    int grasped_object_arm_id;

} HRI_AGENT;

//! @ingroup AGENTS
typedef struct STRUCT_HRI_AGENTS {
    HRI_AGENT ** all_agents; /* The list of agents robots+humans */
    int all_agents_no;
    int source_agent_idx; /* This is the index of the agent who is the robot */
    HRI_AGENT ** robots;
    int robots_no;
    HRI_AGENT ** humans;
    int humans_no;
}HRI_AGENTS;

/* A test structure */
typedef struct struct_hri_shared_zone {
    double x;
    double y;
    double z;
    int value;
} hri_shared_zone;

#endif
