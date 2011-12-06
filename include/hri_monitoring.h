#ifndef _HRI_MONITORING_H
#define _HRI_MONITORING_H

//! @ingroup KNOWLEDGE
//! sphere type used for initial sphere geometric dimension processing and monitoring trigger assessment
typedef enum ENUM_HRI_SPHERE_TYPE {
  SIMPLE_ENTRY = 0, // simple sphere defined through radius and center. To monitor entrance in the sphere;
  SIMPLE_EXIT = 1,  // simple sphere defined through radius and center. To monitor exit from the sphere.
  THROW_IN_CONTAINER = 2, // simple sphere those radius and center is automatically defined as fitting above the container.
  PICK_OBJECT =  3, // simple sphere those radius and center is automatically defined as around object to pick.
  PERMANENT_STOP_MONITOR = 4  //
} HRI_SPHERE_TYPE; // Monitor that hands are fixed at some points.

//! @ingroup KNOWLEDGE
//! Action Monitor Through in Spheres Entry or Exit 
typedef struct STRUCT_HRI_ACTION_MONITORING_SPHERE {
  int isSphereActive; // TRUE if monitor is active, FALSE otherwise.
  char agentName[64]; // agent name whose action is monitored.
  int agentIndex; // agent index.
  int handIndexInput; // -1 if we don't want to precise hand index > -1 otherwise.
  char objectName[64]; // Object name used to define sphere for certain sphereType
  int entityIndex; // entity index of the object 
  double sphereCenterX;
  double sphereCenterY;
  double sphereCenterZ;
  double sphereRadius; // sphere radius.
  double filteringTimeThreshold; // time delay to wait for monitor success condition to triger monitor success
  double timeDelayWithMonitorTrue; // TRUE if monitor must be triggered on going out FALSE on going
  HRI_SPHERE_TYPE sphereType; // what is the type of this sphere as far as creating it is concerned
  int monitorEnterInResult; // TRUE if monitor trigger for enter in spheres and FALSE otherwise.
  int monitorGetOutResult; // TRUE if monitor trigger for get out of spheres (if it was in) and FALSE otherwise.
  int handIndexResult; // Whose agent hands trigger monitor.
  int modifIndex; // each time there is something new on this sphere we increment this counter.
} HRI_ACTION_MONITORING_SPHERE;

/** Action Monitor Through in Spheres Entry or Exit */
typedef struct STRUCT_HRI_ACTION_MONITORING_SPHERES {
  HRI_ACTION_MONITORING_SPHERE ** spheres; // Array of spheres pointers used for monitoring purpose.
  int nbSpheresMax; // number of spheres.
  int nbActiveSpheres; // number of spheres that are active.
  int modifIndex; // each time there is something new in spheres we increment this counter.
  int nbIterSinceLastMonitorTest; // number of iter
  int drawSpheres; // do we draw spheres
} HRI_ACTION_MONITORING_SPHERES;


void hri_draw_action_monitoring_spheres();

#endif
