#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "common.h"
#include"SPlisHSPlasH\RigidBodyObject.h"
#include"PositionBasedDynamicsWrapper\PBDRigidBody.h"

struct tempSolver {
	std::vector<float4> positions;
	std::vector<float3> velocities;
	std::vector<int> phases;

	std::vector<float4> diffusePos;
	std::vector<float3> diffuseVelocities;

	std::vector<int> clothIndices;
	std::vector<float> restLengths;
	std::vector<float> stiffness;
	std::vector<int> triangles;
	std::vector<SPH::RigidBodyParticleObject> rigidBodies;//每个粒子所属的刚体
	//std::vector<float> boundaryPsi;
	//std::vector<float3> force;
};

struct solver {
	float4* oldPos;
	float4* newPos;
	float3* velocities;
	int* phases;
	float* densities;

	float3* forces;
	float* boundaryPsi;
	
	float4* rigidPosPinned;//用于快速的data transfer
	float3* rigidVelPinned;
	float3* rigidForcePinned;

	float4* diffusePos;//这两个数组用于生成foam
	float3* diffuseVelocities;

	int* clothIndices;//存储每个约束两个点的索引
	float* restLengths;//存储每个约束的静止长度
	float* stiffness;//存储每个约束的系数K

	int* neighbors;//每个粒子的邻居粒子索引
	int* numNeighbors;//每个粒子的邻居个数
	int* gridCells;//存放每个cell里粒子的索引
	int* gridCounters;//grid的每个cell内的粒子个数
	int* contacts;//phase=0的粒子周围phase=1邻居粒子的索引
	int* numContacts;//phase=0的粒子周围phase=1的粒子个数

	float3* deltaPs;//记录每个粒子的deltaPos

	float* buffer0;//记录每个粒子的deltaP个数，用于取平均的deltaP；也用于暂存lambda
};

struct solverParams {
public:
	int maxNeighbors;
	int maxParticles;
	int maxContacts;
	int gridWidth, gridHeight, gridDepth;
	int gridSize;

	int numParticles;
	int numDiffuse;

	int numCloth;
	int numConstraints;

	int numRigidBody;
	int numRigidParticles;

	float3 gravity;
	float3 bounds;

	int numIterations;
	float radius;
	float restDistance;
	//float sor;
	//float vorticity;

	float KPOLY;
	float SPIKY;
	float restDensity;//ρ0
	float lambdaEps;
	float vorticityEps;
	float C;//XSPH viscosity
	float K;//PBF中Scorr项
	float dqMag;
	float wQH;//PBF中Scorr项
};

#endif