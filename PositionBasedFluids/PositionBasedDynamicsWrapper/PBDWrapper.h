#ifndef __PBDWrapper_h__
#define __PBDWrapper_h__

#include "../SPlisHSPlasH/Common.h"
#include <string>
#include<map>
#include <Demos/Simulation/SimulationModel.h>
#include <Demos/Simulation/TimeStepController.h>
#include <Demos/Simulation/DistanceFieldCollisionDetection.h>


class PBDWrapper
{
protected:
	std::string m_dataPath;
	PBD::SimulationModel m_model;
	PBD::DistanceFieldCollisionDetection m_cd;
	PBD::TimeStepController m_sim;

	short m_clothSimulationMethod = 2;
	short m_solidSimulationMethod = 2;
	short m_bendingMethod = 2;
	/*bool m_drawAABB;
	int m_drawDistanceFields;
	bool m_drawStaticBodies;
	int m_drawBVHDepth;*/
	std::string m_sceneName;
	std::string m_sceneFileName;
	bool m_enableMayaExport = false;
	Real m_dampingCoeff = 0.0;


public:
	PBDWrapper();
	~PBDWrapper();

	void reset();
	void initModel(const Real timeStepSize);

	/** Read rigid body scene and create the rigid body model
	*/
	void readScene(const std::string &sceneFileName,std::vector<std::string> &rigidBodyFileNames);
	void initTriangleModelConstraints();
	void initTetModelConstraints();

	void timeStep();
	void updateVisModels();


	PBD::SimulationModel &getSimulationModel() { return m_model; }
	PBD::DistanceFieldCollisionDetection &getCollisionDetection() { return m_cd; }
	PBD::TimeStepController &getTimeStepController() { return m_sim; }


	
};





#endif
