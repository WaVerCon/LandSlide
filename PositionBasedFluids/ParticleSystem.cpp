#include "ParticleSystem.h"
#include "ParticleSystem.cuh"

using namespace std;

static float t = 0.0f;
static int flag = 1;
static int frameCounter = 0;
static const float deltaT = 0.0083f;

ParticleSystem::ParticleSystem() : running(false), moveWall(false), s(new solver) {}

ParticleSystem::~ParticleSystem() {
	cudaCheck(cudaFree(s->oldPos));
	cudaCheck(cudaFree(s->newPos));
	cudaCheck(cudaFree(s->velocities));
	cudaCheck(cudaFree(s->densities));
	cudaCheck(cudaFree(s->phases));
	cudaCheck(cudaFree(s->diffusePos));
	cudaCheck(cudaFree(s->diffuseVelocities));
	cudaCheck(cudaFree(s->neighbors));
	cudaCheck(cudaFree(s->numNeighbors));
	cudaCheck(cudaFree(s->gridCells));
	cudaCheck(cudaFree(s->gridCounters));
	cudaCheck(cudaFree(s->contacts));
	cudaCheck(cudaFree(s->numContacts));
	cudaCheck(cudaFree(s->deltaPs));
	cudaCheck(cudaFree(s->buffer0));
	
	cudaCheck(cudaFree(s->forces));
	cudaCheck(cudaFree(s->boundaryPsi));
	cudaCheck(cudaFreeHost(s->rigidPosPinned));
	cudaCheck(cudaFreeHost(s->rigidVelPinned));
	cudaCheck(cudaFreeHost(s->rigidForcePinned));
	delete s;
}

void ParticleSystem::initialize(tempSolver &tp, solverParams &tempParams) {
	//General particle info
	cudaCheck(cudaMalloc((void**)&s->oldPos, tempParams.numParticles * sizeof(float4)));
	cudaCheck(cudaMalloc((void**)&s->newPos, tempParams.numParticles * sizeof(float4)));
	cudaCheck(cudaMalloc((void**)&s->velocities, tempParams.numParticles * sizeof(float3)));
	cudaCheck(cudaMalloc((void**)&s->densities, tempParams.numParticles * sizeof(float)));
	cudaCheck(cudaMalloc((void**)&s->phases, tempParams.numParticles * sizeof(int)));
	//Diffuse
	cudaCheck(cudaMalloc((void**)&s->diffusePos, tempParams.numDiffuse * sizeof(float4)));
	cudaCheck(cudaMalloc((void**)&s->diffuseVelocities, tempParams.numDiffuse * sizeof(float3)));
	//Cloth
	cudaCheck(cudaMalloc((void**)&s->clothIndices, tempParams.numConstraints * 2 * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->restLengths, tempParams.numConstraints * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->stiffness, tempParams.numConstraints * sizeof(int)));
	//Neighbor finding and buffers
	cudaCheck(cudaMalloc((void**)&s->neighbors, tempParams.maxNeighbors * tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->numNeighbors, tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->gridCells, tempParams.maxParticles * tempParams.gridSize * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->gridCounters, tempParams.gridSize * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->contacts, tempParams.maxContacts * tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->numContacts, tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMalloc((void**)&s->deltaPs, tempParams.numParticles * sizeof(float3)));
	cudaCheck(cudaMalloc((void**)&s->buffer0, tempParams.numParticles * sizeof(float)));
	//RigidBodyParticle info
	cudaCheck(cudaMalloc((void**)&s->forces, tempParams.numRigidParticles*sizeof(float3)));
	cudaCheck(cudaMalloc((void**)&s->boundaryPsi, tempParams.numRigidParticles*sizeof(float)));
	cudaCheck(cudaMallocHost((void**)&s->rigidPosPinned, tempParams.numRigidParticles*sizeof(float4)));
	cudaCheck(cudaMallocHost((void**)&s->rigidVelPinned, tempParams.numRigidParticles*sizeof(float3)));
	cudaCheck(cudaMallocHost((void**)&s->rigidForcePinned, tempParams.numRigidParticles*sizeof(float3)));
	
	cudaCheck(cudaMemset(s->oldPos, 0, tempParams.numParticles * sizeof(float4)));
	cudaCheck(cudaMemset(s->newPos, 0, tempParams.numParticles * sizeof(float4)));
	cudaCheck(cudaMemset(s->velocities, 0, tempParams.numParticles * sizeof(float3)));
	cudaCheck(cudaMemset(s->densities, 0, tempParams.numParticles * sizeof(float)));
	cudaCheck(cudaMemset(s->phases, 0, tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMemset(s->diffusePos, 0, tempParams.numDiffuse * sizeof(float4)));
	cudaCheck(cudaMemset(s->diffuseVelocities, 0, tempParams.numDiffuse * sizeof(float3)));
	cudaCheck(cudaMemset(s->clothIndices, 0, tempParams.numConstraints * 2 * sizeof(int)));
	cudaCheck(cudaMemset(s->restLengths, 0, tempParams.numConstraints * sizeof(int)));
	cudaCheck(cudaMemset(s->stiffness, 0, tempParams.numConstraints * sizeof(int)));
	cudaCheck(cudaMemset(s->neighbors, 0, tempParams.maxNeighbors * tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMemset(s->numNeighbors, 0, tempParams.numParticles * sizeof(int)));
	cudaCheck(cudaMemset(s->gridCells, 0, tempParams.maxParticles * tempParams.gridSize * sizeof(int)));
	cudaCheck(cudaMemset(s->gridCounters, 0, tempParams.gridSize * sizeof(int)));
	//RigidBodyParticle
	cudaCheck(cudaMemset(s->forces, 0, tempParams.numRigidParticles*sizeof(float3)));
	cudaCheck(cudaMemset(s->boundaryPsi, 0, tempParams.numRigidParticles*sizeof(float)));
	
	cudaCheck(cudaMemset(s->rigidPosPinned, 0, tempParams.numRigidParticles*sizeof(float4)));
	cudaCheck(cudaMemset(s->rigidVelPinned, 0, tempParams.numRigidParticles*sizeof(float3)));
	cudaCheck(cudaMemset(s->rigidForcePinned, 0, tempParams.numRigidParticles*sizeof(float3)));

	cudaCheck(cudaMemcpy(s->oldPos, &tp.positions[0], tempParams.numParticles * sizeof(float4), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->newPos, &tp.positions[0], tempParams.numParticles * sizeof(float4), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->velocities, &tp.velocities[0], tempParams.numParticles * sizeof(float3), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->phases, &tp.phases[0], tempParams.numParticles * sizeof(int), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->diffusePos, &tp.diffusePos[0], tempParams.numDiffuse * sizeof(float4), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->diffuseVelocities, &tp.diffuseVelocities[0], tempParams.numDiffuse * sizeof(float3), cudaMemcpyHostToDevice));
	if (tp.clothIndices.size()>0)
		cudaCheck(cudaMemcpy(s->clothIndices, &tp.clothIndices[0], tempParams.numConstraints * 2 * sizeof(int), cudaMemcpyHostToDevice));
	if (tp.restLengths.size()>0)
		cudaCheck(cudaMemcpy(s->restLengths, &tp.restLengths[0], tempParams.numConstraints * sizeof(int), cudaMemcpyHostToDevice));
	if (tp.stiffness.size()>0)
		cudaCheck(cudaMemcpy(s->stiffness, &tp.stiffness[0], tempParams.numConstraints * sizeof(int), cudaMemcpyHostToDevice));
	setParams(&tempParams);
}
void ParticleSystem::updateBoundaryParticles(const tempSolver &tp, solverParams &params)//对刚体粒子的velocity，直接使用计算结果进行替代，而不累加
{
	const unsigned int nObjects = params.numRigidBody;
	int pindex = 0;
	cudaCheck(cudaMemcpy(s->rigidPosPinned, s->newPos, params.numRigidParticles*sizeof(float4), cudaMemcpyDeviceToHost));
	for (unsigned int i = 0; i < nObjects; i++)
	{
		SPH::RigidBodyParticleObject rbpo = tp.rigidBodies[i];
		SPH::RigidBodyObject *rbo = rbpo.rigidBody;
		if (rbo->isDynamic())
		{
			for (unsigned int j = 0; j < rbpo.numberOfParticles; ++j){
				unsigned int idx = pindex + j;
				SPH::Vector3r pos = rbo->getRotation() * SPH::Vector3r(s->rigidPosPinned[idx].x, s->rigidPosPinned[idx].y, s->rigidPosPinned[idx].z) + rbo->getPosition();
				s->rigidPosPinned[idx] = make_float4(pos.x, pos.y, pos.z, s->rigidPosPinned[idx].w);
				SPH::Vector3r vel = rbo->getAngularVelocity().cross(SPH::Vector3r(s->rigidPosPinned[idx].x, s->rigidPosPinned[idx].y, s->rigidPosPinned[idx].z) - rbo->getPosition()) + rbo->getVelocity();
				s->rigidVelPinned[idx] = make_float3(vel.x, vel.y, vel.z);
			}
		}
		pindex += rbpo.numberOfParticles;
	}
	cudaCheck(cudaMemcpy(s->oldPos, s->rigidPosPinned, params.numRigidParticles*sizeof(float4), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->newPos, s->rigidPosPinned, params.numRigidParticles*sizeof(float4), cudaMemcpyHostToDevice));
	cudaCheck(cudaMemcpy(s->velocities, s->rigidVelPinned, params.numRigidParticles*sizeof(float3), cudaMemcpyHostToDevice));
}
void ParticleSystem::updateBoundaryForces(tempSolver &tp, solverParams &tempParams){
	Real h = SPH::TimeManager::getCurrent()->getTimeStepSize();
	const unsigned int nObjects = tempParams.numRigidBody;
	int pindex = 0;
	cudaCheck(cudaMemcpy(s->rigidForcePinned, s->forces, tempParams.numRigidParticles*sizeof(float3), cudaMemcpyDeviceToHost));
	for (unsigned int i = 0; i < nObjects; i++)
	{
		SPH::RigidBodyParticleObject rbpo = tp.rigidBodies[i];
		SPH::RigidBodyObject *rbo = rbpo.rigidBody;
		if (rbo->isDynamic())
		{
			((SPH::PBDRigidBody*)rbo)->updateTimeStepSize();
			SPH::Vector3r force, torque;
			force.setZero();
			torque.setZero();
			
			for (int j = 0; j < rbpo.numberOfParticles; j++)
			{
				unsigned int idx = pindex + j;
				SPH::Vector3r m_f = SPH::Vector3r(s->rigidForcePinned[idx].x, s->rigidForcePinned[idx].y, s->rigidForcePinned[idx].z);
				force += m_f;
				SPH::Vector3r pos = SPH::Vector3r(s->rigidPosPinned[idx].x, s->rigidPosPinned[idx].y, s->rigidPosPinned[idx].z);
				torque += (pos - rbo->getPosition()).cross(m_f);
				s->rigidForcePinned[idx] = make_float3(0);
			}
			rbo->addForce(force);
			rbo->addTorque(torque);
		}
		pindex += rbpo.numberOfParticles;
	}
	cudaCheck(cudaMemcpy(s->forces, s->rigidForcePinned, tempParams.numRigidParticles*sizeof(float3),cudaMemcpyHostToDevice));
}

void ParticleSystem::updateWrapper(solverParams &tempParams) {
	if (running) {
		if (moveWall) {
			if (frameCounter >= 0) {
				//width = (1 - abs(sin((frameCounter - 400) * (deltaT / 1.25f)  * 0.5f * PI)) * 1) + 4;
				t += flag * deltaT / 1.0f;
				if (t >= 1) {
					t = 1;
					flag *= -1;
				} else if (t <= 0) {
					t = 0;
					flag *= -1;
				}

				tempParams.bounds.x = easeInOutQuad(t, tempParams.gridWidth * tempParams.radius, -1.5f, 1.0f);
			}

			frameCounter++;
			setParams(&tempParams);
		}

		update(s, &tempParams);
	}
}

void ParticleSystem::getPositions(float* positionsPtr, int numParticles) {
	cudaCheck(cudaMemcpy(positionsPtr, s->oldPos, numParticles * sizeof(float4), cudaMemcpyDeviceToDevice));
}

void ParticleSystem::getDiffuse(float* diffusePosPtr, float* diffuseVelPtr, int numDiffuse) {
	cudaCheck(cudaMemset(diffusePosPtr, 0, numDiffuse * sizeof(float4)));
	cudaCheck(cudaMemcpy(diffusePosPtr, s->diffusePos, numDiffuse * sizeof(float4), cudaMemcpyDeviceToDevice));
	cudaCheck(cudaMemcpy(diffuseVelPtr, s->diffuseVelocities, numDiffuse * sizeof(float3), cudaMemcpyDeviceToDevice));
}

int ParticleSystem::getIndex(float i, float j) {
	return int(i * 20 + j);
}

float ParticleSystem::easeInOutQuad(float t, float b, float c, float d) {
	t /= d / 2;
	if (t < 1) return c / 2 * t*t + b;
	t--;
	return -c / 2 * (t*(t - 2) - 1) + b;
};