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
#include "P3d-pkg.h"
#include "hri.h"

///////////////////////////////////////////////////////////
/** Action Monitoring Through little spheres Management. */
///////////////////////////////////////////////////////////
HRI_ACTION_MONITORING_SPHERES * GLOBAL_ACTION_MONITORING_SPHERES = NULL;

/** Spheres initialization */

HRI_ACTION_MONITORING_SPHERES * hri_create_spheres(int nbSpheresMax)
{
  HRI_ACTION_MONITORING_SPHERES * spheres;
  int i;
  spheres = MY_ALLOC(HRI_ACTION_MONITORING_SPHERES, 1);
  spheres->spheres = NULL;
  spheres->nbSpheresMax = nbSpheresMax;
  spheres->nbActiveSpheres = 0;
  spheres->modifIndex = 0;
  spheres->nbIterSinceLastMonitorTest = 0; 
  spheres->drawSpheres = FALSE;
  spheres->drawSpheresOpacity = 0.1;

  for(i=0; i<nbSpheresMax; i++) {
    spheres->spheres = MY_REALLOC(spheres->spheres , HRI_ACTION_MONITORING_SPHERE*,i,i+1);
    spheres->spheres[i] = MY_ALLOC(HRI_ACTION_MONITORING_SPHERE,1);
    spheres->spheres[i]->isSphereActive = FALSE;
    strcpy(spheres->spheres[i]->objectName,"");
    spheres->spheres[i]->entityIndex = 0;
    strcpy(spheres->spheres[i]->agentName,"");
    spheres->spheres[i]->agentIndex = 0;
    spheres->spheres[i]->handIndexInput = -1;
    spheres->spheres[i]->sphereCenterX = 0.0;
    spheres->spheres[i]->sphereCenterY = 0.0;
    spheres->spheres[i]->sphereCenterZ = 0.0;
    spheres->spheres[i]->sphereRadius = 0;
    spheres->spheres[i]->filteringTimeThreshold = 0;
    spheres->spheres[i]->sphereType = SIMPLE_ENTRY;
    spheres->spheres[i]->monitorEnterInResult = FALSE;
    spheres->spheres[i]->monitorGetOutResult = FALSE;
    spheres->spheres[i]->handIndexResult = 0;
    spheres->spheres[i]->modifIndex = 0;
  }

  GLOBAL_ACTION_MONITORING_SPHERES = spheres;
  
  return spheres;
}

/** Add / Delete sphere in spheres */

int hriUpdateSphereInSpheres(HRI_AGENTS * agents, HRI_ENTITIES * ents,HRI_ACTION_MONITORING_SPHERES * spheres ,int nbSphereMax,int monitorIndex,int activateSphere, char* agentName, int agentIndex, char* objectName,int entityIndex, double sphereCenterX, double sphereCenterY, double sphereCenterZ, double sphereRadius, double sphereRadiusMultiply, double sphereCenterTranslationMultiply,double filteringTimeThreshold, HRI_SPHERE_TYPE sphereType)
{
  
  if( monitorIndex >= nbSphereMax)
    return FALSE;
  
  if(!activateSphere){
    if(spheres->spheres[monitorIndex]->isSphereActive){
      spheres->spheres[monitorIndex]->isSphereActive = FALSE;
      spheres->nbActiveSpheres--;
    }
  }
  else {
    
    //Update the number of active spheres
    if(!spheres->spheres[monitorIndex]->isSphereActive){
      spheres->nbActiveSpheres++;
      spheres->spheres[monitorIndex]->isSphereActive = TRUE;
    }
    
    //
    strcpy(spheres->spheres[monitorIndex]->agentName, agentName);
    spheres->spheres[monitorIndex]->agentIndex = agentIndex;
    spheres->spheres[monitorIndex]->sphereType = sphereType;
    spheres->spheres[monitorIndex]->filteringTimeThreshold = filteringTimeThreshold;
    // Initialize timeDelayWithMonitorTrue that could be used for temporal filtering.
    spheres->spheres[monitorIndex]->timeDelayWithMonitorTrue = 0;
    
    //Some spheres are completely define through the parameters
    if((sphereType == SIMPLE_ENTRY) || (sphereType == SIMPLE_EXIT)){
      spheres->spheres[monitorIndex]->sphereCenterX = sphereCenterX;
      spheres->spheres[monitorIndex]->sphereCenterY = sphereCenterY;
      spheres->spheres[monitorIndex]->sphereCenterZ = sphereCenterZ;
      spheres->spheres[monitorIndex]->sphereRadius = sphereRadius;
    }
    
    //Some other spheres must be computed from the parameter
    else {
      strcpy(spheres->spheres[monitorIndex]->objectName, objectName);
      spheres->spheres[monitorIndex]->entityIndex = entityIndex;
      hriGetSpherePositionAndSize(agents,ents,spheres->spheres[monitorIndex],sphereRadiusMultiply,sphereCenterTranslationMultiply);
    }   

  }    
  
  //Reinitialize Monitor Results.
  spheres->spheres[monitorIndex]->monitorEnterInResult = FALSE;
  spheres->spheres[monitorIndex]->monitorGetOutResult = FALSE;
  spheres->spheres[monitorIndex]->handIndexResult = 0;

  spheres->spheres[monitorIndex]->modifIndex++;
  spheres->modifIndex++;  

  return TRUE;
}


/* compute radius and sphere center from sphere object and type  */
int hriComputeSphereRadiusAndCenter(HRI_ENTITY *obj , HRI_ACTION_MONITORING_SPHERE * sphere,double sphereRadiusMultiply,double sphereCenterTranslationMultiply)
{
  p3d_vector3 sphereCenter;
  double radius;
  double radius2;

  if(obj == NULL ) {
    printf("%s:%d hriComputeSpherRadiusAndCenter input is null",__FILE__,__LINE__);
    return FALSE;
  }

  if(obj->type == HRI_OBJECT){
    sphereCenter[0] = (obj->robotPt->BB.xmax + obj->robotPt->BB.xmin)/2;
    sphereCenter[1] = (obj->robotPt->BB.ymax + obj->robotPt->BB.ymin)/2;
    sphereCenter[2] = (obj->robotPt->BB.zmax + obj->robotPt->BB.zmin)/2;
    radius = obj->robotPt->BB.xmax - sphereCenter[0];
  
    radius2 = obj->robotPt->BB.ymax - sphereCenter[1];
    if( radius < radius2 ){
      radius = radius2;
    }

    radius2 = obj->robotPt->BB.zmax - sphereCenter[2];
    if( radius < radius2 ){
      radius = radius2;
    }

    // For PICK_OBJECT we choose a sphere centered on a translation of -radius according z axis of the object center. We choose sphere radius as three time object radius.
    if(sphere->sphereType == PICK_OBJECT){
      sphere->sphereCenterX = sphereCenter[0]; 
      sphere->sphereCenterY = sphereCenter[1];
      sphere->sphereCenterZ = sphereCenter[2] - radius*sphereCenterTranslationMultiply;
      sphere->sphereRadius = 2*radius*sphereRadiusMultiply;
    }

    // For THROW_IN_CONTAINER we choose a sphere centered on a 2*radius z translation of object center with a sphere radius of 2*radius.
    else if(sphere->sphereType == THROW_IN_CONTAINER){
      sphere->sphereCenterX = sphereCenter[0]; 
      sphere->sphereCenterY = sphereCenter[1];
      sphere->sphereCenterZ = sphereCenter[2] + 2*radius*sphereCenterTranslationMultiply;
      sphere->sphereRadius = 2*radius*sphereRadiusMultiply;
    }

  }

  else{
    printf("%s:%d hriComputeSpherRadiusAndCenter object sphere %s should have type HRI_OBJECT",__FILE__,__LINE__, obj->name);
    return FALSE;
  }

  return TRUE;
}

/**  get sphere center and radius from sphere type and object */
int hriGetSpherePositionAndSize(HRI_AGENTS * agents, HRI_ENTITIES * ents,HRI_ACTION_MONITORING_SPHERE * sphere,double sphereRadiusMultiply,double sphereCenterTranslationMultiply){
  // If 
  if((sphere->sphereType == THROW_IN_CONTAINER) || (sphere->sphereType == PICK_OBJECT)){
    hriComputeSphereRadiusAndCenter(ents->entities[sphere->entityIndex],sphere,sphereRadiusMultiply,sphereCenterTranslationMultiply);
  }
  else if(sphere->sphereType == PERMANENT_STOP_MONITOR){
  }  
} 

/**  test all active monitors. */
int hriTestMonitor(HRI_AGENTS * agents, HRI_ENTITIES * ents,HRI_ACTION_MONITORING_SPHERES * spheres,int nbMaxSpheres, int nbIterBeforeMonitorTest){

  int h_i, i;
  HRI_ENTITY * ent, ** present_ents;
  int nbActiveSpheres;
  int agentId;
  int handIndexInput;
  HRI_AGENT * agent;
  double distance;
  int monitorTriggered = FALSE;
  double entBBCenterX=0;
  double entBBCenterY=0;
  double entBBCenterZ=0;
  
  if(agents == NULL || ents == NULL) {
    printf("Not Initialized\n");
    return FALSE;
  }
  
  nbActiveSpheres = spheres->nbActiveSpheres;
    
  for(i=0; i<nbMaxSpheres; i++) {
    if(nbActiveSpheres == 0)
      break;

    if(spheres->spheres[i]->isSphereActive){
      nbActiveSpheres--;
      //If monitor has already triggered, we don't need to test it again.
      if(spheres->spheres[i]->monitorEnterInResult && spheres->spheres[i]->monitorGetOutResult)
	continue;

      //Reinit monitorTriggered variable
      monitorTriggered = FALSE;

      //Get concerned agent
      agentId = spheres->spheres[i]->agentIndex;
      
      //Get concerned hand
      handIndexInput = spheres->spheres[i]->handIndexInput;
      
      // We don't yet use threshold. We may have to. 
      //Get Agents
      agent = agents->all_agents[agentId];
      if(agent->is_present == FALSE)
	continue;

      //Check all hands or only one according to handIndexInput value.
      for(h_i=0; h_i<agent->hand_nb ; h_i++) { 	
	if((handIndexInput>-1) && (handIndexInput != h_i))
	  continue;
	ent = agent->hand[h_i];
	if((ent->type == HRI_OBJECT_PART) || (ent->type == HRI_AGENT_PART) ) {
	  entBBCenterX = (ent->partPt->BB.xmax + ent->partPt->BB.xmin)/2;
	  entBBCenterY = (ent->partPt->BB.ymax + ent->partPt->BB.ymin)/2;
	  entBBCenterZ = (ent->partPt->BB.zmax + ent->partPt->BB.zmin)/2;
	}
	else {
	  entBBCenterX = (ent->robotPt->BB.xmax + ent->robotPt->BB.xmin)/2;
	  entBBCenterY = (ent->robotPt->BB.ymax + ent->robotPt->BB.ymin)/2;
	  entBBCenterZ = (ent->robotPt->BB.zmax + ent->robotPt->BB.zmin)/2;
	}

	//distance = DISTANCE3D(ent->robotPt->joints[1]->abs_pos[0][3],ent->robotPt->joints[1]->abs_pos[1][3],ent->robotPt->joints[1]->abs_pos[2][3],spheres->spheres[i]->sphereCenterX,spheres->spheres[i]->sphereCenterY,spheres->spheres[i]->sphereCenterZ);
	distance = DISTANCE3D(entBBCenterX,entBBCenterY,entBBCenterZ,spheres->spheres[i]->sphereCenterX,spheres->spheres[i]->sphereCenterY,spheres->spheres[i]->sphereCenterZ);

	// are we in sphere
	if( !spheres->spheres[i]->monitorEnterInResult && (distance<spheres->spheres[i]->sphereRadius)){
	  //Check on time threshold
	  if( spheres->spheres[i]->filteringTimeThreshold <= spheres->spheres[i]->timeDelayWithMonitorTrue){
	    spheres->spheres[i]->monitorEnterInResult = TRUE;
	    monitorTriggered  = TRUE;
	      }
	  else{	    
	    //We should update timeDelayWithMonitorTrue here	    
	  }
	}
	if( !spheres->spheres[i]->monitorEnterInResult && (distance>=spheres->spheres[i]->sphereRadius))
	  spheres->spheres[i]->timeDelayWithMonitorTrue = 0;
	
	
	// exit of sphere type. We check it only on previously recognized hand.
	if( (h_i == spheres->spheres[i]->handIndexResult) && spheres->spheres[i]->monitorEnterInResult && (distance>spheres->spheres[i]->sphereRadius)){
	  //Check on time threshold
	  if( spheres->spheres[i]->filteringTimeThreshold <= spheres->spheres[i]->timeDelayWithMonitorTrue){
	    spheres->spheres[i]->monitorGetOutResult = TRUE;
	    monitorTriggered  = TRUE;
	  }
	  else{	    
	    //We should update timeDelayWithMonitorTrue here
	  }
	}	
	if( (h_i == spheres->spheres[i]->handIndexResult) && spheres->spheres[i]->monitorEnterInResult && (distance<=spheres->spheres[i]->sphereRadius))
	  spheres->spheres[i]->timeDelayWithMonitorTrue = 0;	
	if (monitorTriggered){
	  spheres->modifIndex++;
	  spheres->spheres[i]->handIndexResult = h_i;
	  spheres->spheres[i]->modifIndex++;
	  break;
	}      
      }
    }
  }
}


void hri_draw_action_monitoring_spheres()
{
  int nbActiveSpheres,i;
  int nbSpheresMax;
  double x,y,z,r,opacity;
  GLdouble color[4];

  if(GLOBAL_ACTION_MONITORING_SPHERES == NULL){
    return;
  }
  if(!GLOBAL_ACTION_MONITORING_SPHERES->drawSpheres){
    return;
  }


  nbActiveSpheres = GLOBAL_ACTION_MONITORING_SPHERES->nbActiveSpheres;
  nbSpheresMax = GLOBAL_ACTION_MONITORING_SPHERES->nbSpheresMax;
  opacity = GLOBAL_ACTION_MONITORING_SPHERES->drawSpheresOpacity;
  if(opacity > 1)
    opacity = 1.0;
  if(opacity < 0)
    opacity = 0.0;

  for(i=0; i<nbSpheresMax; i++) {
    if(nbActiveSpheres == 0)
      break;
    if(GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->isSphereActive){
      nbActiveSpheres--;
      //If monitor has already triggered, we don't need to test it again.
      if(GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->monitorEnterInResult && GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->monitorGetOutResult){
	color[0] = 1.0; color[1]= 0.0; color[2]= 0.0; color[3]= opacity;
      }
      else if(GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->monitorEnterInResult){
	color[0] = 1.0; color[1]= 0.5; color[2]= 0.0; color[3]= opacity;
      }
      else{
	color[0] = 0.0; color[1]= 1.0; color[2]= 0.0; color[3]= opacity;
      }      

      //glColor4f(color[0], color[1], color[2], color[3]);
      x = GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->sphereCenterX;
      y = GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->sphereCenterY;
      z = GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->sphereCenterZ;
      r = GLOBAL_ACTION_MONITORING_SPHERES->spheres[i]->sphereRadius;
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
      g3d_set_color(Any,color);
      g3d_draw_solid_sphere(x,y,z,r,20);
  
      glDisable(GL_BLEND);///g3d_drawSphere(x,y,z,r);
    }      
  }
}
