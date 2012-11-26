#ifndef _AGENT_STATE_ANALYSIS_PROTO_H
#define _AGENT_STATE_ANALYSIS_PROTO_H

#include "Agent_State_Analysis.h"
#include "Mightability_Analysis.h"

extern int hri_execute_Agent_State_Analysis_functions();
extern int prepare_for_Agent_State_Analysis(char *threshold_file_path);
extern int init_thresholds_for_ASA(char *file_name_with_path);
extern int is_object_laying_on_a_support(int obj_index, int &support_index);
extern int update_agent_sitting_information();

#endif