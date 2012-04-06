#ifndef _HRI_KNOWLEDGE_H
#define _HRI_KNOWLEDGE_H

//! @defgroup KNOWLEDGE Knowledge

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_ENTITY_TYPE {
  HRI_OBJECT = 0,
  HRI_ISAGENT = 1, /* There is already a type called HRI_AGENT */
  HRI_AGENT_PART = 2,
  HRI_OBJECT_PART = 3,
  HRI_VIRTUAL_OBJECT = 4
} HRI_ENTITY_TYPE;

//! @ingroup KNOWLEDGE
//! We had some more sementic to adapt the level of processing and reasoning to the
typedef enum ENUM_HRI_ENTITY_SUBTYPE {
  HRI_MOVABLE_OBJECT = 0, // Tape, Bottles
  HRI_OBJECT_SUPPORT = 1,  // Tables, Shelf , ...
  HRI_OBJECT_CONTAINER = 2, // Dust bin, Box
  HRI_AGENT_HEAD = 3,
  HRI_AGENT_HAND = 4,
  HRI_AGENT_AGENT = 5, 
  HRI_OBJECT_PLACEMAT = 6,  // planar objects to materialize some areas on furnitures 
  HRI_UK_ENTITY_SUBTYPE = 7
} HRI_ENTITY_SUBTYPE;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_MOTION {
  HRI_STATIC = 1,
  HRI_MOVING = 2,
  HRI_UK_MOTION = 0 /* Unknown motion. meaning we don't know */
} HRI_MOTION;

extern int HRI_MAX_OBJECT_UNDETECTION;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_DETECTION { 
  HRI_NEVER_DETECTED = 0,
  HRI_DETECTED = 1,
  HRI_EXPLAINED_UNDETECTION = 2,
  HRI_UNEXPLAINED_UNDETECTION_ITER = 3, 
  HRI_UNEXPLAINED_UNDETECTION_MAX = 4
} HRI_DETECTION;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_PLACEMENT_STATE_TRANSITION { 
  HRI_DISAPPEAR = 0,
  HRI_APPEAR = 1,
  HRI_START_MOVING = 2,
  HRI_STOP_MOVING = 3, 
  HRI_UK_PL_STATE_TRANSITION = 4
} HRI_PLACEMENT_STATE_TRANSITION;

//! @ingroup KNOWLEDGE
//! different type of inferences managed
typedef enum ENUM_HRI_INFERRENCE_TYPE {
  HRI_NO_INFERRENCE =  0,
  HRI_PRECISE_ROBOT_HAND = 1,
  HRI_ROUGH_ROBOT_HAND = 2,
  HRI_HUMAN_HAND =  3,
  HRI_IS_IN = 4

} HRI_INFERRENCE_TYPE;

//! @ingroup KNOWLEDGE
//! different type of inferences managed
typedef enum ENUM_HRI_INFERRENCE_VALIDITY {
  HRI_NO_PROBABILITY =  0,
  HRI_HIGHLY_PROBABLE = 1,
  HRI_PROBABLE = 2,
  HRI_AVERAGE =  3,
  HRI_UNPROBABLE = 4,
  HRI_HIGHLY_UNPROBABLE =  5
} HRI_INFERRENCE_VALIDITY;

//! @ingroup KNOWLEDGE
typedef struct STRUCT_HRI_ENTITY {
  char name[64];
  
  HRI_ENTITY_TYPE type;
  ENUM_HRI_ENTITY_SUBTYPE subtype;
  int is_present; /* Is present in the env, i.e. at least seen once by the robot */
  
  int is_detected; // has robot just perceived the object
  unsigned long detection_time;
  unsigned long last_detection_time;
  int undetection_iter;
  HRI_DETECTION undetection_status; /* why and how much was the object undetected */
  double visibility_percentage; /** visibility percentage score for robot for disappear management */

  int can_disappear_and_move; /* Can this entity disappear? For example a furniture can be considered a not movable */
  int disappeared; /* Is present but it is not at the place where it is supposed to be */
  int allow_disappear; /*    */
  int hasInferrence ; /* is this entity position inferred */
  HRI_INFERRENCE_TYPE inferrenceType; //what is the type of this infference
  char inferrenceObjectOrAgentPartName[64] ; //Agent, object name used to define this inference (in agent hand, inside object )
  int agentPartNum;  // An agent can have several hands, or parts, this is the index of the part.
  double infx,infy,infz; // x,y,z coordinate to save the inferred position candidate.
  double perceptionInferrenceConflictThresholdMultiply; // Multiplicative term to modify automatically computed threshold
  double perceptionInferrenceConflictThreshold; // resulting  Value for the threshold to classify perception inferrence conflict validity
  double perceptionInferrenceConflictValue; // Distance between perceived and ijnfered position
  HRI_INFERRENCE_VALIDITY inferrenceValidity; // inferrence validity value in case of perception inference conflicts 

  int last_ismoving_iter; /* how many*/
  HRI_MOTION filtered_motion; /* */
  double minStaticDist; /* minimum distance between some former and new position of entity since last motion detected*/

  int is_pl_state_transition_new;
  HRI_PLACEMENT_STATE_TRANSITION pl_state_transition;
  
  int isOnSpecialSupport; // Hack to bypass ontology to get isOn value for one chosen support to speed the system.
  

  p3d_rob * robotPt;
  p3d_obj * partPt;
  int agent_idx; /* if it's a part of an agent, then its index */
  int is_fixed;
  int indexInEnv; /* index in the environment */
} HRI_ENTITY;

//! @ingroup KNOWLEDGE
typedef struct STRUCT_HRI_ENTITIES {
  HRI_ENTITY ** entities;
  int entities_nb;
  int eventsInTheWorld; // put to 1 if something happen that make SA recomputation necessary
  int lastEventsInTheWorldStep; // number of step without something happening that make Situation Assessment recomputation necessary
  int isWorldStatic; // 1 if nothing has happened since a given step threshold that make Situation Assessment recomputation necessary
  int maxWorldNotStaticBeforeRecompute; // number of maximum step with world not static before to force recoompute
  int numSuccessiveWorldNotStaticSinceLastRecompute; // number of step with world not static since last recoomputation

  int forbidWorldStatic;
  int needSituationAssessmentUpdate; //1 if full situation assessment update is required
  int needLooksatUpdate; //1 if specific look at situation assessment update is required
  int general_allow_disappear; /// flag to allow or forbid disappear management
  int printVisibilityImages;  /// allow print of visibility of images for debugging purposes.
  int printSARecomputeStatus; /// flag to print info about SA recomputation Status.

  bool manageDivergentBeliefs; /// true if divergent belief management is activated.

  int needToUpdateParameters; /// true if we need to update parameters used for situation assement
  int numParameterToUpdate; /// 0 for hackedReachabilityDistance / 1 for maxUnexplainedUndetectionIter / 2 isOnThreshold. 
  double hackedReachabilityDistance; /// hacked distance to compute reachability for human while GIK do not work for Herakles.
  int maxUnexplainedUndetectionIter; /// object is considered to have disappear if it is undetected a numer of time equual to this parameter without any reason.
  double isOnThreshold; /// max allowed threshold between min "isOn candidate" object height value and max furniture height value.
  int specialSupportEntityIndex; //
    
    double averageDistanceScoreExportThreshold; /// minimum threshold to export average motion score threshold. 
} HRI_ENTITIES;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_VISIBILITY {
  HRI_VISIBLE = 0,
  HRI_INVISIBLE = 1,
  HRI_UK_VIS = 2 /* Unknown visibility. meaning we don't know */
} HRI_VISIBILITY;

//! @ingroup KNOWLEDGE
//! For the boolean facts we always have possible values : true, false and unknown
typedef enum ENUM_HRI_TRUE_FALSE_UK_V {
  HRI_FALSE_V = 0,
  HRI_TRUE_V = 1,
  HRI_UK_V = 2 
} HRI_TRUE_FALSE_UK_V;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_VISIBILITY_PLACEMENT {
  HRI_FOA = 1,
  HRI_FOV = 2,
  HRI_OOF = 3,
  HRI_UK_VIS_PLACE = 4 /* Unknown visibility placement. meaning we don't know */
} HRI_VISIBILITY_PLACEMENT;

//! @ingroup KNOWLEDGE
typedef struct STRUCT_HRI_VISIBILITY_LIST {
  HRI_VISIBILITY *vis; /* The index is the same as env->nb */
  HRI_VISIBILITY_PLACEMENT *vispl; /* The index is the same as env->nb */
  int vis_nb; /* Normally this number should be = to the number of robots in the env */
} HRI_VISIBILITY_LIST;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_REACHABILITY {
  HRI_UNREACHABLE = 0,
  HRI_REACHABLE = 1,
  HRI_HARDLY_REACHABLE = 2,
  HRI_UK_REACHABILITY = 3 /* Unknown reachability. meaning we don't know */
} HRI_REACHABILITY;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_PLACEMENT_RELATION {
  HRI_ISIN     = 0,
  HRI_ISON     = 1,
  HRI_ISNEXTTO = 2,
  HRI_NOPLR    = 3,
  HRI_UK_PLR   = 4 /* Unknown placement. meaning we don't know */
} HRI_PLACEMENT_RELATION;

//! @ingroup KNOWLEDGE
typedef enum ENUM_HRI_SPATIAL_RELATION {
  HRI_NO_RELATION = 0,
  HRI_NEAR_FRONT = 1,
  HRI_NEAR_FRONT_LEFT =  2,
  HRI_NEAR_LEFT = 3,
  HRI_NEAR_BACK_LEFT = 4,
  HRI_NEAR_BACK = 5,
  HRI_NEAR_BACK_RIGHT = 6,
  HRI_NEAR_RIGHT = 7,
  HRI_NEAR_FRONT_RIGHT = 8,
  HRI_FAR_FRONT = 9,
  HRI_FAR_FRONT_LEFT =  10,
  HRI_FAR_LEFT = 11,
  HRI_FAR_BACK_LEFT = 12,
  HRI_FAR_BACK = 13,
  HRI_FAR_BACK_RIGHT = 14,
  HRI_FAR_RIGHT = 15,
  HRI_FAR_FRONT_RIGHT = 16,
  HRI_UK_RELATION = 17, /* Unknown relation. meaning we don't know */
  HRI_FRONT = 18,
  HRI_BEHIND = 19,
  HRI_LEFT = 20,
  HRI_RIGHT = 21,
  HRI_FAR = 22,
  HRI_NEAR = 23
} HRI_SPATIAL_RELATION;


/// Structure use to study distance and speed between one entity and agent part
/// It is used to get table agent move to or object agent want to grasp.
typedef struct STRUCT_HRI_DISTANCE_AND_SPEED_INFO {
    double distOld;
    double distNew;
    double timeOld;
    double timeNew;
    int numTimeStep;
    double speed;
    double averageDistanceScore;
    double averageDistanceScoreOld;
} HRI_DISTANCE_AND_SPEED_INFO;

//! @ingroup KNOWLEDGE
typedef struct STRUCT_HRI_KNOWLEDGE_ON_ENTITY {
    HRI_ENTITY * entPt;
    
    configPt entityPositionForAgent;  // to save entity position in case it is different from main agent position. It is created when agent leave the scene and deleted as soon as agent has same position as robot. Also used for source agent to store its position while other agent diverging position are set as the one in the model.
    bool hasEntityPosition;  /// true if entityPositionForAgent is instantiated.
    bool isEntityPositionInModel; /// true if this position is currently the one in model
    /* bool hasDifferentEntityPosition; */  /// true if different from main agent.
    bool hasEntityPositionKnowledge; /// true if agent knows position. (It could be the same one as robot, a different one known by robot (hasEntityPosition true and hasDifferentEntityPosition true) or unknown to robot.
    bool hasEntityPositionKnowledgeExportedValue; 
    double lastEntPosX,lastEntPosY,lastEntPosZ; ///X,Y,Z value of enity last known position.
    //! @ingroup VISIBILITY
    /**
     * entityPositionForAgent is filled 
     *
     * Divergent Belief Management.
     * A Robot has belief where object is
     *  a Human has belief where object isknows where object is 
     *   1 Belief same as robot
     *   
     *   2 Belief is different
     *  b Human has no belief where object is.
     */

    /// Agent Entity Distance & Speed Info use to describe motion between agent body or hands and entities.
    HRI_DISTANCE_AND_SPEED_INFO * distanceAndSpeedInfoArray;
    int manageDistanceAndSpeedInfoArraySize; /// number of Agent parts considered
    int * manageDistanceAndSpeedInfoArrayAgentPartIndex; /// agent part type for each arry element.type -1 for body and hand index (0 -> max hand index) for hands.


    long update_date;

    int disappeared_isexported; /* bool to know wether disappear information was exported for this agent */  
    int presenceValueExported;
    int presence_isexported;

    HRI_MOTION motion;
    int motion_ischanged;
    int motion_isexported;

    HRI_VISIBILITY visibility;
    int visibility_ischanged;
    int visibility_isexported;

    ///Divergent Belief Management
    HRI_VISIBILITY* visibilityBy;
    int* visibilityBy_ischanged;
    int* visibilityBy_isexported;

    HRI_REACHABILITY reachability;
    int reachability_ischanged;
    int reachability_isexported;

    ///Divergent Belief Management
    HRI_REACHABILITY* reachabilityBy;
    int* reachabilityBy_ischanged;
    int* reachabilityBy_isexported;

    HRI_TRUE_FALSE_UK_V is_looked_at;
    int is_looked_at_ischanged;
    int is_looked_at_isexported;

    ///Divergent Belief Management
    HRI_TRUE_FALSE_UK_V* is_looked_atBy;
    int* is_looked_atBy_ischanged;
    int* is_looked_atBy_isexported;

    HRI_TRUE_FALSE_UK_V isSeen;
    int isSeenischanged;
    int isSeenisexported;

    ///Divergent Belief Management
    HRI_TRUE_FALSE_UK_V* isSeenBy;
    int* isSeenByischanged;
    int* isSeenByisexported;

    HRI_TRUE_FALSE_UK_V is_pointed_at;
    int is_pointed_at_ischanged;
    int is_pointed_at_isexported;

    ///Divergent Belief Management
    HRI_TRUE_FALSE_UK_V* is_pointed_atBy;
    int* is_pointed_atBy_ischanged;
    int* is_pointed_atBy_isexported;

    HRI_VISIBILITY_PLACEMENT is_placed_from_visibility; /* oof, fov, ... */
    int visibility_placement_ischanged;
    int visibility_placement_isexported;

    HRI_VISIBILITY_PLACEMENT* is_placed_from_visibilityBy; /* oof, fov, ... */
    int* visibility_placementBy_ischanged;
    int* visibility_placementBy_isexported;

    HRI_SPATIAL_RELATION is_located_from_agent; /* Front, right, .... */
    int spatial_relation_ischanged;
    int spatial_relation_isexported;
    HRI_SPATIAL_RELATION is_left_right_from_agent;
    HRI_SPATIAL_RELATION is_front_behind_from_agent;
    HRI_SPATIAL_RELATION is_far_near_from_agent;


    HRI_PLACEMENT_RELATION * is_placed; /* on, in, ... */
    HRI_PLACEMENT_RELATION * is_placed_old; /* To limit number of messages send to ontology */
    int * placement_relation_ischanged;
    int * placement_relation_isexported;
    int is_placed_nb;  /* length is HRI_ENTITIES->all_entities_nb */

} HRI_KNOWLEDGE_ON_ENTITY;

//! @ingroup KNOWLEDGE
//! Each agent has a knowledge structure. It presents the agent's knowledge on the geometry of the world 
typedef struct STRUCT_HRI_KNOWLEDGE {
    /* The spatial knowledge on the state of things from the perspective of the agent */
    /* Normally all indexes should be synchronized with entities structure */

    /* int * looks_at; /\* indexes of entities in entity structure *\/ */
    /* int looks_at_nb; */
    /* int * points_at; /\* indexes of entities in entity structure *\/ */
    /* int points_at_nb; */

    HRI_KNOWLEDGE_ON_ENTITY * entities;
    int entities_nb;
    int numDivergentPositions; /// number
    int numUnknownPositions;  /// number of object
   bool needToSaveObjPositions; // true if current object positions for this agent need to be saved. (Divergent Belief Management)
    bool needToCheckEntitiesPositionKnowledge;  /// true if agent position knowledge need to be checked once agent appears.  
} HRI_KNOWLEDGE;



/// Draw small spheres to show divergent positions.
void hri_draw_divergent_positions();


#endif
