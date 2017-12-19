#ifndef SCENE_H
#define SCENE_H

#include "common.h"
#include "parameters.h"
#include "setupFunctions.h"
#include<Demos\Utils\Timing.h>
#include<Demos\Utils\OBJLoader.h>

class Scene {
public:
	Scene(std::string name) : name(name) {}
	virtual void init(tempSolver* tp, solverParams* sp) = 0;
	
private:
	std::string name;
};

class DamBreak : public Scene {
public:
	DamBreak(std::string name) : Scene(name) {}

	virtual void init(tempSolver* tp, solverParams* sp) {
		const float radius = 0.1f;
		const float restDistance = radius * 0.5f;
		float3 lower = make_float3(0.0f, 0.1f, 0.0f);
		int3 dims = make_int3(68, 48, 88);
		createParticleGrid(tp, sp, lower, dims, restDistance);
		
		sp->radius = radius;
		sp->restDistance = restDistance;
		sp->numIterations = 4;
		sp->numDiffuse = 1024 * 2048;
		sp->numParticles = int(tp->positions.size());
		sp->numCloth = 0;
		sp->numConstraints = 0;
		sp->gravity = make_float3(0, -9.8f, 0);
		sp->bounds = make_float3(dims) * radius;
		sp->gridWidth = int(sp->bounds.x / radius);
		sp->gridHeight = int(sp->bounds.y / radius);
		sp->gridDepth = int(sp->bounds.z / radius);
		sp->gridSize = sp->gridWidth * sp->gridHeight * sp->gridDepth;
		sp->maxContacts = 10;
		sp->maxNeighbors = 50;
		sp->maxParticles = 50;
		sp->restDensity = 6378.0f;
		sp->lambdaEps = 600.0f;
		sp->vorticityEps = 0.0001f;
		sp->C = 0.01f; //0.0025f;
		sp->K = 0.00001f;
		sp->KPOLY = 315.0f / (64.0f * PI * pow(radius, 9));
		sp->SPIKY = 45.0f / (PI * pow(radius, 6));
		sp->dqMag = 0.2f * radius;
		sp->wQH = sp->KPOLY * pow((radius * radius - sp->dqMag * sp->dqMag), 3);

		tp->diffusePos.resize(sp->numDiffuse);
		tp->diffuseVelocities.resize(sp->numDiffuse);
	}
};

class FluidCloth : public Scene {
public:
	FluidCloth(std::string name) : Scene(name) {}

	virtual void init(tempSolver* tp, solverParams* sp) {
		float stretch = 1.0f;
		float bend = 1.0f;
		float shear = 1.0f;

		const float radius = 0.1f;
		const float restDistance = radius * 0.5f;
		float3 lower = make_float3(0.0f, 1.0f, 0.0f);
		int3 dims = make_int3(64, 1, 64);
		createCloth(tp, sp, lower, dims, radius * 0.25f, 1, stretch, bend, shear, 0.05f);
		sp->numCloth = int(tp->positions.size());

		//Pinned vertices
		int c1 = 0;
		int c2 = dims.x - 1;
		int c3 = dims.x * (dims.z - 1);
		int c4 = (dims.x * dims.z) - 1;

		tp->positions[c1].w = 0;
		tp->positions[c2].w = 0;
		tp->positions[c3].w = 0;
		tp->positions[c4].w = 0;

		//Tethers
		for (int i = 0; i < int(tp->positions.size()); i++) {
			//tp->positions[i].y = 1.5f - sinf(25.0f * 180.0f / PI) * tp->positions[i].x;
			//tp->positions[i].x *= cosf(25.0f * 180.0f / PI);

			if (i != c1 && i != c2 && i != c3 && i != c4) {
				float tether = -0.1f;
				addConstraint(tp, sp, c1, i, tether);
				addConstraint(tp, sp, c2, i, tether);
				addConstraint(tp, sp, c3, i, tether);
				addConstraint(tp, sp, c4, i, tether);
			}
		}

		//move corners closer together?

		//Add water
		lower = make_float3(0.5f, 1.1f, 0.5f);
		dims = make_int3(10, 10, 10);
		createParticleGrid(tp, sp, lower, dims, restDistance);

		sp->radius = radius;
		sp->restDistance = restDistance;
		sp->numIterations = 4;
		sp->numDiffuse = 1024 * 1024;
		sp->numParticles = int(tp->positions.size());
		sp->numConstraints = int(tp->restLengths.size());
		sp->gravity = make_float3(0, -9.8f, 0);
		sp->bounds = make_float3(dims.x * 4 * radius, dims.y * 4 * radius, dims.z * 4 * radius);
		sp->gridWidth = int(sp->bounds.x / radius);
		sp->gridHeight = int(sp->bounds.y / radius);
		sp->gridDepth = int(sp->bounds.z / radius);
		sp->gridSize = sp->gridWidth * sp->gridHeight * sp->gridDepth;
		sp->maxContacts = 15;
		sp->maxNeighbors = 50;
		sp->maxParticles = 50;
		sp->restDensity = 6378.0f;
		sp->lambdaEps = 600.0f;
		sp->vorticityEps = 0.0001f;
		sp->C = 0.01f; //0.0025f;
		sp->K = 0.00001f;
		sp->KPOLY = 315.0f / (64.0f * PI * pow(radius, 9));
		sp->SPIKY = 45.0f / (PI * pow(radius, 6));
		sp->dqMag = 0.3f * radius;
		sp->wQH = sp->KPOLY * pow((radius * radius - sp->dqMag * sp->dqMag), 3);

		tp->diffusePos.resize(sp->numDiffuse);
		tp->diffuseVelocities.resize(sp->numDiffuse);
	}
};

class LandSlide : public Scene {
public:
	LandSlide(std::string name) : Scene(name) {
		
	}
	std::string scene_file;
	
	void init(tempSolver* tp, solverParams* sp) {
		const float radius = 0.1f;
		//const float radius = 0.025f;
		//const float restDistance = radius * 0.5f;
		const float restDistance = radius;
		/*float3 lower = make_float3(-0.5f,10.0f, -0.5f);
		int3 dims = make_int3(10, 10, 10);*/
		

		
		scene_file = "Scenes/LandSlide.json";
		std::vector<string> rigidBodyFiles;
		sp->numRigidBody = 0;
		
		
		pbdWrapper.readScene(scene_file,rigidBodyFiles);
		pbdWrapper.initModel(TimeManager::getCurrent()->getTimeStepSize());
		
		initRigidBodies(pbdWrapper, tp, sp, rigidBodyFiles, Vector3r::Zero(), Matrix3r::Identity(),1.0,radius);


		sp->numRigidParticles = tp->positions.size();
		//createBodyModel();

		float3 lower = make_float3(0.5f, 2.0f, 0.5f);
		int3 dims = make_int3(10, 10, 10);
		createParticleGrid(tp, sp, lower, dims, restDistance);

		sp->radius = radius;
		sp->restDistance = restDistance;
		sp->numIterations = 4;
		sp->numDiffuse = 1024 * 2048;
		sp->numParticles = int(tp->positions.size());
		sp->numCloth = 0;
		sp->numConstraints = 0;
		sp->gravity = make_float3(0, -9.8f, 0);
		sp->bounds = make_float3(dims.x * 10 * radius, dims.y * 10 * radius, dims.z * 10 * radius);
		
		sp->origin = make_float3(-sp->bounds.x / 2, 0, -sp->bounds.z / 2);

		sp->gridWidth = int(sp->bounds.x / radius);
		sp->gridHeight = int(sp->bounds.y / radius);
		sp->gridDepth = int(sp->bounds.z / radius);
		sp->gridSize = sp->gridWidth * sp->gridHeight * sp->gridDepth;
		sp->maxContacts = 10;
		sp->maxNeighbors = 50;
		sp->maxParticles = 50;
		sp->restDensity = 6378.0f;
		//sp->restDensity = 1000.0f;

		sp->lambdaEps = 600.0f;
		sp->vorticityEps = 0.0001f;
		sp->C = 0.01f; //0.0025f;
		sp->K = 0.00001f;
		sp->KPOLY = 315.0f / (64.0f * PI * pow(radius, 9));
		sp->SPIKY = 45.0f / (PI * pow(radius, 6));
		sp->dqMag = 0.2f * radius;
		sp->wQH = sp->KPOLY * pow((radius * radius - sp->dqMag * sp->dqMag), 3);

		tp->diffusePos.resize(sp->numDiffuse);
		tp->diffuseVelocities.resize(sp->numDiffuse);

		
	}
	//std::vector<Model*> getModels(){ return models; }
	PBDWrapper& getPBDWrapper(){ return pbdWrapper;  }
	std::vector<SPH::RigidBodyParticleObject*>& getRigidBodies(){ return rigidBodies; }
private:
	//std::vector<Model*> models;
	PBDWrapper pbdWrapper;
	std::vector<SPH::RigidBodyParticleObject*> rigidBodies;//每个粒子所属的刚体

	void createBodyModel()
	{
		DistanceFieldCollisionDetection& cd = pbdWrapper.getCollisionDetection();
		SimulationModel& model = pbdWrapper.getSimulationModel();

		SimulationModel::RigidBodyVector &rb = model.getRigidBodies();
		SimulationModel::ConstraintVector &constraints = model.getConstraints();

		const string dataPath = "C:/LandSlide/PositionBasedFluids/Resources/Models";
		string fileNameBox = FileSystem::normalizePath(dataPath + "/cube.obj");
		IndexedFaceMesh meshBox;
		VertexData vdBox;
		OBJLoader::loadObj(fileNameBox, vdBox, meshBox);

		string fileNameCylinder = FileSystem::normalizePath(dataPath + "/cylinder.obj");
		IndexedFaceMesh meshCylinder;
		VertexData vdCylinder;
		OBJLoader::loadObj(fileNameCylinder, vdCylinder, meshCylinder);

		string fileNameTorus = FileSystem::normalizePath(dataPath + "/torus.obj");
		IndexedFaceMesh meshTorus;
		VertexData vdTorus;
		OBJLoader::loadObj(fileNameTorus, vdTorus, meshTorus);

		string fileNameCube = FileSystem::normalizePath(dataPath + "/cube_5.obj");
		IndexedFaceMesh meshCube;
		VertexData vdCube;
		OBJLoader::loadObj(fileNameCube, vdCube, meshCube);

		string fileNameSphere = FileSystem::normalizePath(dataPath + "/sphere.obj");
		IndexedFaceMesh meshSphere;
		VertexData vdSphere;
		OBJLoader::loadObj(fileNameSphere, vdSphere, meshSphere);


		const unsigned int num_piles_x = 5;
		const unsigned int num_piles_z = 5;
		const Real dx_piles = 4.0;
		const Real dz_piles = 4.0;
		const Real startx_piles = -0.5 * (Real)(num_piles_x - 1)*dx_piles;
		const Real startz_piles = -0.5 * (Real)(num_piles_z - 1)*dz_piles;
		const unsigned int num_piles = num_piles_x * num_piles_z;
		const unsigned int num_bodies_x = 3;
		const unsigned int num_bodies_y = 5;
		const unsigned int num_bodies_z = 3;
		const Real dx_bodies = 6.0;
		const Real dy_bodies = 6.0;
		const Real dz_bodies = 6.0;
		const Real startx_bodies = -0.5 * (Real)(num_bodies_x - 1)*dx_bodies;
		const Real starty_bodies = 14.0;
		const Real startz_bodies = -0.5 * (Real)(num_bodies_z - 1)*dz_bodies;
		const unsigned int num_bodies = num_bodies_x * num_bodies_y * num_bodies_z;
		rb.resize(num_piles + num_bodies + 1);
		unsigned int rbIndex = 0;

		// floor
		rb[rbIndex] = new RigidBody();
		rb[rbIndex]->initBody(1.0,
			Vector3r(0.0, -0.5, 0.0),
			Quaternionr(1.0, 0.0, 0.0, 0.0),
			vdBox, meshBox, Vector3r(100.0, 1.0, 100.0));
		rb[rbIndex]->setMass(0.0);

		const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices->size());
		
		cd.addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector3r(100.0, 1.0, 100.0));
		rbIndex++;

		Real current_z = startz_piles;
		for (unsigned int i = 0; i < num_piles_z; i++)
		{

			Real current_x = startx_piles;
			for (unsigned int j = 0; j < num_piles_x; j++)
			{
				rb[rbIndex] = new RigidBody();
				rb[rbIndex]->initBody(100.0,
					Vector3r(current_x, 5.0, current_z),
					Quaternionr(1.0, 0.0, 0.0, 0.0),
					vdCylinder, meshCylinder,
					Vector3r(0.5, 10.0, 0.5));
				rb[rbIndex]->setMass(0.0);

				const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
				const unsigned int nVert = static_cast<unsigned int>(vertices->size());
				cd.addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(0.5, 10.0));
				current_x += dx_piles;
				rbIndex++;
			}
			current_z += dz_piles;
		}

		Real current_y = starty_bodies;
		unsigned int currentType = 0;
		for (unsigned int i = 0; i < num_bodies_y; i++)
		{
			Real current_x = startx_bodies;
			for (unsigned int j = 0; j < num_bodies_x; j++)
			{
				Real current_z = startz_bodies;
				for (unsigned int k = 0; k < num_bodies_z; k++)
				{
					rb[rbIndex] = new RigidBody();

					Real ax = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
					Real ay = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
					Real az = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
					Real w = static_cast <Real> (rand()) / static_cast <Real> (RAND_MAX);
					Quaternionr q(w, ax, ay, az);
					q.normalize();

					currentType = rand() % 4;
					if (currentType == 0)
					{
						rb[rbIndex]->initBody(100.0,
							Vector3r(current_x, current_y, current_z),
							q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
							vdTorus, meshTorus,
							Vector3r(2.0, 2.0, 2.0));

						const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
						const unsigned int nVert = static_cast<unsigned int>(vertices->size());
						cd.addCollisionTorus(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(2.0, 1.0));
					}
					else if (currentType == 1)
					{
						rb[rbIndex]->initBody(100.0,
							Vector3r(current_x, current_y, current_z),
							q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
							vdCube, meshCube,
							Vector3r(4.0, 1.0, 1.0));

						const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
						const unsigned int nVert = static_cast<unsigned int>(vertices->size());
						cd.addCollisionBox(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector3r(4.0, 1.0, 1.0));
					}
					else if (currentType == 2)
					{
						rb[rbIndex]->initBody(100.0,
							Vector3r(current_x, current_y, current_z),
							q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
							vdSphere, meshSphere);

						const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
						const unsigned int nVert = static_cast<unsigned int>(vertices->size());
						cd.addCollisionSphere(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, 1.0);
					}
					else if (currentType == 3)
					{
						rb[rbIndex]->initBody(100.0,
							Vector3r(current_x, current_y, current_z),
							q, //Quaternionr(1.0, 0.0, 0.0, 0.0),
							vdCylinder, meshCylinder,
							Vector3r(0.75, 5.0, 0.75));

						const std::vector<Vector3r> *vertices = rb[rbIndex]->getGeometry().getVertexDataLocal().getVertices();
						const unsigned int nVert = static_cast<unsigned int>(vertices->size());
						cd.addCollisionCylinder(rbIndex, CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, Vector2r(0.75, 5.0));
					}
					currentType = (currentType + 1) % 4;
					current_z += dz_bodies;
					rbIndex++;
				}
				current_x += dx_bodies;
			}
			current_y += dy_bodies;
		}
	}

	void addRigidParticles(RigidBodyObject * rbo, tempSolver* s, const std::vector<SPH::Vector3r> & rigidParticles, float invMass = 0.05, int phase = 1) {
		RigidBodyParticleObject* rbpo=new RigidBodyParticleObject();
		for (unsigned int i = 0; i < rigidParticles.size(); ++i) {
			float3 pos = make_float3(rigidParticles[i].x(), rigidParticles[i].y(), rigidParticles[i].z());
			rbpo->x0.push_back(rigidParticles[i]);
			s->positions.push_back(make_float4(pos, invMass));
			s->velocities.push_back(make_float3(0));
			s->phases.push_back(phase);

			//s->force.push_back(make_float3(0));
		}
		
		rbpo->rigidBody = rbo;
		rbpo->numberOfParticles = rigidParticles.size();
		rigidBodies.push_back(rbpo);
		//s->boundaryPsi.resize(s->boundaryPsi.size() + rigidParticles.size());
	}
	void initBoundaryParticles(tempSolver* s, solverParams* sp) {
		const unsigned int nObjects = sp->numRigidBody;
		int pindex = 0;
		for (unsigned int i = 0; i < nObjects; i++)
		{
			SPH::RigidBodyParticleObject* rbpo = rigidBodies[i];
			SPH::RigidBodyObject *rbo = rbpo->rigidBody;
			for (unsigned int j = 0; j < rbpo->numberOfParticles; ++j) {
				unsigned int idx = pindex + j;
				PBD::Vector3r pos = rbo->getRotation() * PBD::Vector3r(s->positions[idx].x, s->positions[idx].y, s->positions[idx].z) + rbo->getPosition();
				s->positions[idx] = make_float4(pos.x(), pos.y(), pos.z(), s->positions[idx].w);
				PBD::Vector3r vel = rbo->getAngularVelocity().cross(SPH::Vector3r(s->positions[idx].x, s->positions[idx].y, s->positions[idx].z) - rbo->getPosition()) + rbo->getVelocity();
				s->velocities[idx] = make_float3(vel.x(), vel.y(), vel.z());
			}
			pindex += rbpo->numberOfParticles;
		}
	}
	void initRigidBodies(PBDWrapper& pbdWrapper, tempSolver* s, solverParams* sp, std::vector<string> objFiles, const SPH::Vector3r& particleTranslation, const SPH::Matrix3r& particleRotation, Real scale = 1.0, Real particleRadius = 0.1) {
		//std::string base_path = FileSystem::getProgramPath();
		const bool useCache = true;
		for (unsigned int i = 0; i < objFiles.size(); ++i) {
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
			if (useCache) {
				foundCacheFile = PartioReaderWriter::readParticles(particleFileName, particleTranslation, particleRotation, scale, rigidParticles);
				if (foundCacheFile)
					std::cout << "Loaded cached boundary sampling: " << particleFileName << "\n";
			}
			if (!useCache || !foundCacheFile) {
				std::cout << "Surface sampling of " << objFiles[i] << "\n";
			
				PBD::Timing::startTiming("Poisson disk sampling");
				PoissonDiskSampling sampling;
				sampling.sampleMesh(mesh.numVertices(), &vd.getPosition(0), mesh.numFaces(), mesh.getFaces().data(), particleRadius, 10, 1, rigidParticles);
				static int timing_timerId = -1;
				PBD::Timing::stopTiming(false, timing_timerId);

				// Cache sampling
				if (useCache && (FileSystem::makeDir(cachePath) == 0))
				{
					std::cout << "Save particle sampling: " << particleFileName << "\n";
					PartioReaderWriter::writeParticles(particleFileName, (unsigned int)rigidParticles.size(), rigidParticles.data(), NULL, particleRadius);
				}	
			}
			// transform back to local coordinates
			for (unsigned int j = 0; j < rigidParticles.size(); j++)
				rigidParticles[j] = rb->getRotation().transpose() * (rigidParticles[j] - rb->getPosition());
			if (!rb->isDynamic())
				addRigidParticles(rb, s, rigidParticles, 0.0f);
			else
				addRigidParticles(rb, s, rigidParticles);
			sp->numRigidBody++;
		}

		initBoundaryParticles(s, sp);
	}


};

#endif