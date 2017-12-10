#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include "common.h"
#include "parameters.h"
#include"SPlisHSPlasH\RigidBodyObject.h"
#include<Demos\Simulation\TimeManager.h>
#include"PositionBasedDynamicsWrapper\PBDRigidBody.h"
#include<Common/Common.h>

class ParticleSystem {
public:
	bool running;
	bool moveWall;
	solver* s;

	ParticleSystem();
	~ParticleSystem();

	void initialize(tempSolver &tp, solverParams &tempParams);
	void updateWrapper(solverParams &tempParams);
	void getPositions(float* positionsPtr, int numParticles);
	void getDiffuse(float* diffusePosPtr, float* diffuseVelPtr, int numDiffuse);
	void updateBoundaryForces(std::vector<SPH::RigidBodyParticleObject*> rigidBodies,tempSolver &tp, solverParams &tempParams);
	void updateBoundaryParticles(std::vector<SPH::RigidBodyParticleObject*> rigidBodies,const tempSolver &tp, solverParams& tempParams);

private:
	int getIndex(float i, float j);
	float easeInOutQuad(float t, float b, float c, float d);


	#define cudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }
};

#endif
