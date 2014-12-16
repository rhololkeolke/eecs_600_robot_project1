#ifndef FOOTSTEP_PLAN__ROADMAP_H_
#define FOOTSTEP_PLAN__ROADMAP_H_

#include <map>
#include <footstep_plan/FootStepPlan.h>
#include <footstep_plan/FootStepPose.h>

namespace footstep_plan {

	typedef std::map<int, footstep_plan::FootStepPlan> RoadmapInnerEdges;
	typedef std::map<int, RoadmapInnerEdges> RoadmapEdges; 
	typedef std::map<int, footstep_plan::FootStepPose> RoadmapNodes;
	
	typedef struct Roadmap_ {
		RoadmapEdges edges;
		RoadmapNodes nodes;
	} Roadmap;
}


#endif
