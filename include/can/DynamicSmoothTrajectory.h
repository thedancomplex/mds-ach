/*! Object to generate quickest path trajectories
* This object is used to generate a quickest path trajectory between two points, given an initial
* velocity and desired acceleration.
* \brief Declaration of object to generate quickest path trajectories
* \date 08/01/2008
* \author Kailas Narendran (kailas@xitome.com)
* \file DynamicSmoothTrajectory.h
* \ingroup CommonHelperFunctions
*/

#include "pthread.h"

#pragma once

//! NextTraejctoryPoint return values
enum retNextTrajectoryPoint {
	//! Continue calling nextpoint to get points on trajectory
	NEXTPOINT_CONTINUE = 0,
	//! trajectory complete
	NEXTPOINT_DONE = 1,
	//! timeout waiting for mutex lock
	NEXTPOINT_TIMEOUT = 2
};

//! default timestep
#define DEFAULT_TIMESTEP    0.010

//! Object to create a quickest path trajectory
/*!
*
* \class CDynamicSmoothTrajectory
*/
class CDynamicSmoothTrajectory {
	private:
		float
		ta0,ta1,						//! time for each phase
		x_d, x_o, v_o, a;					//! movement parameters

		float nta0,nta1;					//! steps for each phase

		int nt;							//! time step

		float fTimestep;					//! CT timestep
		
		//! mutex lock for creating a new target point
		pthread_mutex_t mut;

	public:

	//! Default Constructor
    CDynamicSmoothTrajectory();
	//! default desctructor
	~CDynamicSmoothTrajectory();

	//! Constructor to specify non-default timestep
	CDynamicSmoothTrajectory(float fts);

	//! total # of steps in trajectory
	int GetTotalTrajectoryLength(void){return (int)(nta0+nta1);};
	
	//! Sets the Timestep
	void SetTimestep(float fts){fTimestep=fts;};

	retNextTrajectoryPoint NextTrajectoryPoint(float &fVal, float &fVVal, int incnt);
	int NewTrajectoryTarget(float sx_d,float sx_o, float sv_o, float sa);
};
