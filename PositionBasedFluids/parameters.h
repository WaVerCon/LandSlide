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
	std::vector<SPH::RigidBodyParticleObject> rigidBodies;//ÿ�����������ĸ���
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
	
	float4* rigidPosPinned;//���ڿ��ٵ�data transfer
	float3* rigidVelPinned;
	float3* rigidForcePinned;

	float4* diffusePos;//������������������foam
	float3* diffuseVelocities;

	int* clothIndices;//�洢ÿ��Լ�������������
	float* restLengths;//�洢ÿ��Լ���ľ�ֹ����
	float* stiffness;//�洢ÿ��Լ����ϵ��K

	int* neighbors;//ÿ�����ӵ��ھ���������
	int* numNeighbors;//ÿ�����ӵ��ھӸ���
	int* gridCells;//���ÿ��cell�����ӵ�����
	int* gridCounters;//grid��ÿ��cell�ڵ����Ӹ���
	int* contacts;//phase=0��������Χphase=1�ھ����ӵ�����
	int* numContacts;//phase=0��������Χphase=1�����Ӹ���

	float3* deltaPs;//��¼ÿ�����ӵ�deltaPos

	float* buffer0;//��¼ÿ�����ӵ�deltaP����������ȡƽ����deltaP��Ҳ�����ݴ�lambda
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
	float restDensity;//��0
	float lambdaEps;
	float vorticityEps;
	float C;//XSPH viscosity
	float K;//PBF��Scorr��
	float dqMag;
	float wQH;//PBF��Scorr��
};

#endif