#ifndef SETUP_FUNCTIONS_H
#define SETUP_FUNCTIONS_H

#include "common.h"
#include "parameters.h"

#include"Utilities\FileSystem.h"
#include <Common/Common.h>
#include <Demos\Simulation\TimeManager.h>
#include <iostream>
#include <Demos\Utils\Timing.h>
#include "Utilities/PartioReaderWriter.h"
#include "PositionBasedDynamicsWrapper/PBDRigidBody.h"
#include "SPlisHSPlasH/Utilities/PoissonDiskSampling.h"
#include "PositionBasedDynamicsWrapper/PBDWrapper.h"

using namespace SPH;
/*
void addRigidParticles(RigidBodyObject * rbo,tempSolver* s, const std::vector<SPH::Vector3r> & rigidParticles,float invMass=0.05,int phase=1){
	for (unsigned int i = 0; i < rigidParticles.size(); ++i){
		float3 pos = make_float3(rigidParticles[i].x, rigidParticles[i].y, rigidParticles[i].z);
		s->positions.push_back(make_float4(pos, invMass));
		s->velocities.push_back(make_float3(0));
		s->phases.push_back(phase);

		//s->force.push_back(make_float3(0));
	}
	RigidBodyParticleObject rbpo;
	rbpo.rigidBody = rbo;
	rbpo.numberOfParticles = rigidParticles.size();
	s->rigidBodies.push_back(rbpo);
	//s->boundaryPsi.resize(s->boundaryPsi.size() + rigidParticles.size());
}
void initBoundaryParticles(tempSolver* s,solverParams* sp){
	const unsigned int nObjects = sp->numRigidBody;
	int pindex = 0;
	for (unsigned int i = 0; i < nObjects; i++)
	{
		SPH::RigidBodyParticleObject rbpo = s->rigidBodies[i];
		SPH::RigidBodyObject *rbo = rbpo.rigidBody;
		for (unsigned int j = 0; j < rbpo.numberOfParticles; ++j){
			unsigned int idx = pindex + j;
			PBD::Vector3r pos = rbo->getRotation() * PBD::Vector3r(s->positions[idx].x, s->positions[idx].y, s->positions[idx].z) + rbo->getPosition();
			s->positions[idx] = make_float4(pos.x, pos.y, pos.z, s->positions[idx].w);
			PBD::Vector3r vel = rbo->getAngularVelocity().cross(SPH::Vector3r(s->positions[idx].x, s->positions[idx].y, s->positions[idx].z) - rbo->getPosition()) + rbo->getVelocity();
			s->velocities[idx] = make_float3(vel.x, vel.y, vel.z);
		}
		pindex += rbpo.numberOfParticles;
	}
}
void initRigidBodies(PBDWrapper& pbdWrapper, tempSolver* s, solverParams* sp, std::vector<string> objFiles, const SPH::Vector3r& particleTranslation, const SPH::Matrix3r& particleRotation, Real scale=1.0,Real particleRadius=0.1){
	//std::string base_path = FileSystem::getProgramPath();
	const bool useCache = true;
	for (unsigned int i = 0; i < objFiles.size(); ++i){
		std::vector<SPH::Vector3r> rigidParticles;
		std::string mesh_base_path = FileSystem::getFilePath(objFiles[i]);
		std::string mesh_file_name = FileSystem::getFileName(objFiles[i]);
		std::string scene_file_name = "LandSlide";
		std::string cachePath = mesh_base_path + "/Cache";
		std::string particleFileName = FileSystem::normalizePath(cachePath + "/" + scene_file_name + "_" + mesh_file_name + "_" + std::to_string(i) + ".bgeo");

		PBD::SimulationModel &model = pbdWrapper.getSimulationModel();
		PBD::SimulationModel::RigidBodyVector &rigidBodies = model.getRigidBodies();
		PBDRigidBody *rb = new PBDRigidBody(rigidBodies[i]);
		PBD::RigidBodyGeometry &geo = rigidBodies[i]->getGeometry();
		PBD::IndexedFaceMesh &mesh = geo.getMesh();
		PBD::VertexData &vd = geo.getVertexData();

		bool foundCacheFile = false;
		if (useCache){
			foundCacheFile = PartioReaderWriter::readParticles(particleFileName, particleTranslation, particleRotation, scale, rigidParticles);
			if (foundCacheFile)
				std::cout << "Loaded cached boundary sampling: " << particleFileName << "\n";
		}
		if (!useCache || !foundCacheFile){
			std::cout << "Surface sampling of " << objFiles[i] << "\n";
			Timing::startTiming("Poisson disk sampling");
			PoissonDiskSampling sampling;
			sampling.sampleMesh(mesh.numVertices(), &vd.getPosition(0), mesh.numFaces(), mesh.getFaces().data(), particleRadius, 10, 1, rigidParticles);
			static int timing_timerId = -1;
			Timing::stopTiming(false,timing_timerId);

			// Cache sampling
			if (useCache && (FileSystem::makeDir(cachePath) == 0))
			{
				std::cout << "Save particle sampling: " << particleFileName << "\n";
				PartioReaderWriter::writeParticles(particleFileName, (unsigned int)rigidParticles.size(), rigidParticles.data(), NULL, particleRadius);
			}
			// transform back to local coordinates
			for (unsigned int j = 0; j < rigidParticles.size(); j++)
				rigidParticles[j] = rb->getRotation().transpose() * (rigidParticles[j] - rb->getPosition());
		}
		if (!rb->isDynamic())
			addRigidParticles(rb, s, rigidParticles,0.0f,2);
		else
			addRigidParticles(rb, s, rigidParticles);
		sp->numRigidBody++;
	}

	initBoundaryParticles(s,sp);
}
*/
void createParticleGrid(tempSolver* s, solverParams* sp, float3 lower, int3 dims, float radius) {
	for (int x = 0; x < dims.x; x++) {
		for (int y = 0; y < dims.y; y++) {
			for (int z = 0; z < dims.z; z++) {
				float3 pos = lower + make_float3(float(x), float(y), float(z)) * radius;
				s->positions.push_back(make_float4(pos, 1.0f));
				s->velocities.push_back(make_float3(0));
				s->phases.push_back(0);
			}
		}
	}
}

int getIndex(int x, int y, int dx) {
	return y * dx + x;
}

void addConstraint(tempSolver* s, solverParams* sp, int p1, int p2, float stiffness) {
	s->clothIndices.push_back(p1);
	s->clothIndices.push_back(p2);
	s->restLengths.push_back(length(make_float3(s->positions[p1] - s->positions[p2])));
	s->stiffness.push_back(stiffness);
}

void createCloth(tempSolver* s, solverParams* sp, float3 lower, int3 dims, float radius, int phase, float stretch, float bend, float shear, float invMass) {
	//Create grid of particles and add triangles
	for (int z = 0; z < dims.z; z++) {
		for (int y = 0; y < dims.y; y++) {
			for (int x = 0; x < dims.x; x++) {
				float3 pos = lower + make_float3(float(x), float(y), float(z)) * radius;
				s->positions.push_back(make_float4(pos, invMass));
				s->velocities.push_back(make_float3(0));
				s->phases.push_back(phase);

				if (x > 0 && z > 0) {
					s->triangles.push_back(getIndex(x - 1, z - 1, dims.x));
					s->triangles.push_back(getIndex(x, z - 1, dims.x));
					s->triangles.push_back(getIndex(x, z, dims.x));

					s->triangles.push_back(getIndex(x - 1, z - 1, dims.x));
					s->triangles.push_back(getIndex(x, z, dims.x));
					s->triangles.push_back(getIndex(x - 1, z, dims.x));
				}
			}
		}
	}

	//Horizontal constraints
	for (int j = 0; j < dims.z; j++) {
		for (int i = 0; i < dims.x; i++) {
			int i0 = getIndex(i, j, dims.x);
			if (i > 0) {
				int i1 = j * dims.x + i - 1;
				addConstraint(s, sp, i0, i1, stretch);
			}

			if (i > 1) {
				int i2 = j * dims.x + i - 2;
				addConstraint(s, sp, i0, i2, bend);
			}

			if (j > 0 && i < dims.x - 1) {
				int iDiag = (j - 1) * dims.x + i + 1;
				addConstraint(s, sp, i0, iDiag, shear);
			}

			if (j > 0 && i > 0) {
				int iDiag = (j - 1) * dims.x + i - 1;
				addConstraint(s, sp, i0, iDiag, shear);
			}
		}
	}

	//Vertical constraints
	for (int i = 0; i < dims.x; i++) {
		for (int j = 0; j < dims.z; j++) {
			int i0 = getIndex(i, j, dims.x);
			if (j > 0) {
				int i1 = (j - 1) * dims.x + i;
				addConstraint(s, sp, i0, i1, stretch);
			}

			if (j > 1) {
				int i2 = (j - 2) * dims.x + i;
				addConstraint(s, sp, i0, i2, bend);
			}
		}
	}
}
#endif