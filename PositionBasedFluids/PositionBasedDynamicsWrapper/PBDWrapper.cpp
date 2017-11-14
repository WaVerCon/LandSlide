#include "PBDWrapper.h"
#include <iostream>
#include <Demos/Utils/OBJLoader.h>
#include <Demos/Utils/Utilities.h>
#include <Demos/Utils/SceneLoader.h>
#include <Demos/Utils/TetGenLoader.h>
#include <Demos/Utils/Utilities.h>
#include "../SPlisHSPlasH/TimeManager.h"
#include "../SPlisHSPlasH/Utilities/Timing.h"

#define _USE_MATH_DEFINES
#include "math.h"

#define PBD_DATA_PATH "C:\LandSlide\data"

using namespace Eigen;
using namespace std;


float jointColor[4] = { 0.0f, 0.6f, 0.2f, 1 };

PBDWrapper::PBDWrapper()
{
	m_dataPath = std::string(PBD_DATA_PATH);

	m_clothSimulationMethod = 2;
	m_solidSimulationMethod = 2;
	m_bendingMethod = 2;
	m_sceneName = "LandSlideSimulation";
	m_sceneFileName = "LandSlide";
	m_enableMayaExport = false;
	m_dampingCoeff = 0.0;

}

PBDWrapper::~PBDWrapper()
{
	delete SPH::TimeManager::getCurrent();
}



 void PBDWrapper::reset()
 {
	m_model.reset();
	m_sim.reset();
 }



 void PBDWrapper::timeStep()
 {
	PBD::ParticleData &pd = m_model.getParticles();
	PBD::SimulationModel::RigidBodyVector &rb = m_model.getRigidBodies();
	SPH::TimeManager::getCurrent()->setTimeStepSize(SPH::TimeManager::getCurrent()->getTimeStepSize());

	m_sim.step(m_model);

	for (unsigned int i = 0; i < pd.size(); i++)
	{
		pd.getVelocity(i) *= (1.0 - m_dampingCoeff);
	}
	for (unsigned int i = 0; i < rb.size(); i++)
	{
		rb[i]->getVelocity() *= (1.0 - m_dampingCoeff);
		rb[i]->getAngularVelocity() *= (1.0 - m_dampingCoeff);
	}
}

void PBDWrapper::updateVisModels()
{
	PBD::ParticleData &pd = m_model.getParticles();

	// Update visualization models
	for (unsigned int i = 0; i < m_model.getTetModels().size(); i++)
	{
		m_model.getTetModels()[i]->updateMeshNormals(pd);
		m_model.getTetModels()[i]->updateVisMesh(pd);
	}
	for (unsigned int i = 0; i < m_model.getTriangleModels().size(); i++)
	{
		m_model.getTriangleModels()[i]->updateMeshNormals(pd);
	}
}

void PBDWrapper::readScene(const std::string &sceneFileName, std::vector<std::string> &rigidBodyFileNames)
{
	m_sceneFileName = sceneFileName;

	PBD::SimulationModel::RigidBodyVector &rb = m_model.getRigidBodies();
	PBD::SimulationModel::TriangleModelVector &triModels = m_model.getTriangleModels();
	PBD::SimulationModel::TetModelVector &tetModels = m_model.getTetModels();
	PBD::SimulationModel::ConstraintVector &constraints = m_model.getConstraints();

	PBD::SceneLoader::SceneData data;
	PBD::SceneLoader loader;
	loader.readScene(sceneFileName, data);
	std::cout << "Scene: " << sceneFileName << "\n";

	std::string basePath = PBD::Utilities::getFilePath(sceneFileName);

	m_sceneName = data.m_sceneName;

	m_sim.setGravity(data.m_gravity);
	m_sim.setMaxIterations(data.m_maxIter);
	m_sim.setMaxIterationsV(data.m_maxIterVel);
	m_sim.setVelocityUpdateMethod(data.m_velocityUpdateMethod);
	if (data.m_triangleModelSimulationMethod != -1)
		m_clothSimulationMethod = data.m_triangleModelSimulationMethod;
	if (data.m_tetModelSimulationMethod != -1)
		m_solidSimulationMethod = data.m_tetModelSimulationMethod;
	if (data.m_triangleModelBendingMethod != -1)
		m_bendingMethod = data.m_triangleModelBendingMethod;
	m_cd.setTolerance(data.m_contactTolerance);
	m_model.setContactStiffnessRigidBody(data.m_contactStiffnessRigidBody);
	m_model.setContactStiffnessParticleRigidBody(data.m_contactStiffnessParticleRigidBody);

	m_model.setClothStiffness(data.m_cloth_stiffness);
	m_model.setClothBendingStiffness(data.m_cloth_bendingStiffness);
	m_model.setClothXXStiffness(data.m_cloth_xxStiffness);
	m_model.setClothYYStiffness(data.m_cloth_yyStiffness);
	m_model.setClothXYStiffness(data.m_cloth_xyStiffness);
	m_model.setClothXYPoissonRatio(data.m_cloth_xyPoissonRatio);
	m_model.setClothYXPoissonRatio(data.m_cloth_yxPoissonRatio);
	m_model.setClothNormalizeStretch(data.m_cloth_normalizeStretch);
	m_model.setClothNormalizeShear(data.m_cloth_normalizeShear);

	//////////////////////////////////////////////////////////////////////////
	// rigid bodies
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<PBD::VertexData, PBD::IndexedFaceMesh>> objFiles;
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		const PBD::SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		// Check if already loaded
		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
		{
			PBD::IndexedFaceMesh mesh;
			PBD::VertexData vd;
			PBD::OBJLoader::loadObj(PBD::Utilities::normalizePath(rbd.m_modelFile), vd, mesh);
			objFiles[rbd.m_modelFile] = { vd, mesh };
			rigidBodyFileNames.push_back(rbd.m_modelFile);
		}
	}

	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const PBD::SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		if ((tmd.m_modelFileVis != "") &&
			(objFiles.find(tmd.m_modelFileVis) == objFiles.end()))
		{
			PBD::IndexedFaceMesh mesh;
			PBD::VertexData vd;
			PBD::OBJLoader::loadObj(PBD::Utilities::normalizePath(tmd.m_modelFileVis), vd, mesh);
			objFiles[tmd.m_modelFileVis] = { vd, mesh };
		}
	}


	rb.resize(data.m_rigidBodyData.size());
	std::map<unsigned int, unsigned int> id_index;
	for (unsigned int i = 0; i < data.m_rigidBodyData.size(); i++)
	{
		const PBD::SceneLoader::RigidBodyData &rbd = data.m_rigidBodyData[i];

		if (objFiles.find(rbd.m_modelFile) == objFiles.end())
			continue;

		id_index[rbd.m_id] = i;

		PBD::VertexData &vd = objFiles[rbd.m_modelFile].first;
		PBD::IndexedFaceMesh &mesh = objFiles[rbd.m_modelFile].second;

		rb[i] = new PBD::RigidBody();

		rb[i]->initBody(rbd.m_density,
			rbd.m_x,
			rbd.m_q,
			vd, mesh,
			rbd.m_scale);

		if (!rbd.m_isDynamic)
			rb[i]->setMass(0.0);
		else
		{
			rb[i]->setVelocity(rbd.m_v);
			rb[i]->setAngularVelocity(rbd.m_omega);
		}
		rb[i]->setRestitutionCoeff(rbd.m_restitutionCoeff);
		rb[i]->setFrictionCoeff(rbd.m_frictionCoeff);

		const std::vector<PBD::Vector3r> *vertices = rb[i]->getGeometry().getVertexDataLocal().getVertices();
		const unsigned int nVert = static_cast<unsigned int>(vertices->size());

		switch (rbd.m_collisionObjectType)
		{
		case PBD::SceneLoader::RigidBodyData::No_Collision_Object: break;
		case PBD::SceneLoader::RigidBodyData::Sphere:
			m_cd.addCollisionSphere(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case PBD::SceneLoader::RigidBodyData::Box:
			m_cd.addCollisionBox(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case PBD::SceneLoader::RigidBodyData::Cylinder:
			m_cd.addCollisionCylinder(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case PBD::SceneLoader::RigidBodyData::Torus:
			m_cd.addCollisionTorus(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale.head<2>(), rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case PBD::SceneLoader::RigidBodyData::HollowSphere:
			m_cd.addCollisionHollowSphere(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale[0], rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		case PBD::SceneLoader::RigidBodyData::HollowBox:
			m_cd.addCollisionHollowBox(i, PBD::CollisionDetection::CollisionObject::RigidBodyCollisionObjectType, &(*vertices)[0], nVert, rbd.m_collisionObjectScale, rbd.m_thicknessSDF, rbd.m_testMesh, rbd.m_invertSDF);
			break;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	// triangle models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<std::string, pair<PBD::VertexData, PBD::IndexedFaceMesh>> triFiles;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const PBD::SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		// Check if already loaded
		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
		{
			PBD::IndexedFaceMesh mesh;
			PBD::VertexData vd;
			PBD::OBJLoader::loadObj(PBD::Utilities::normalizePath(tmd.m_modelFile), vd, mesh);
			triFiles[tmd.m_modelFile] = { vd, mesh };
		}
	}

	triModels.reserve(data.m_triangleModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index;
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		const PBD::SceneLoader::TriangleModelData &tmd = data.m_triangleModelData[i];

		if (triFiles.find(tmd.m_modelFile) == triFiles.end())
			continue;

		tm_id_index[tmd.m_id] = i;

		PBD::VertexData vd = triFiles[tmd.m_modelFile].first;
		PBD::IndexedFaceMesh &mesh = triFiles[tmd.m_modelFile].second;

		const PBD::Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vd.size(); j++)
		{
			vd.getPosition(j) = R * (vd.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		m_model.addTriangleModel(vd.size(), mesh.numFaces(), &vd.getPosition(0), mesh.getFaces().data(), mesh.getUVIndices(), mesh.getUVs());

		PBD::TriangleModel *tm = triModels[triModels.size() - 1];
		PBD::ParticleData &pd = m_model.getParticles();
		unsigned int offset = tm->getIndexOffset();

		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);
	}

	initTriangleModelConstraints();

	//////////////////////////////////////////////////////////////////////////
	// tet models
	//////////////////////////////////////////////////////////////////////////

	// map file names to loaded geometry to prevent multiple imports of same files
	std::map<pair<string, string>, pair<vector<PBD::Vector3r>, vector<unsigned int>>> tetFiles;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const PBD::SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		// Check if already loaded
		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		if (tetFiles.find(fileNames) == tetFiles.end())
		{
			vector<PBD::Vector3r> vertices;
			vector<unsigned int> tets;
			PBD::TetGenLoader::loadTetgenModel(PBD::Utilities::normalizePath(tmd.m_modelFileNodes), PBD::Utilities::normalizePath(tmd.m_modelFileElements), vertices, tets);
			tetFiles[fileNames] = { vertices, tets };
		}
	}

	tetModels.reserve(data.m_tetModelData.size());
	std::map<unsigned int, unsigned int> tm_id_index2;
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		const PBD::SceneLoader::TetModelData &tmd = data.m_tetModelData[i];

		pair<string, string> fileNames = { tmd.m_modelFileNodes, tmd.m_modelFileElements };
		auto geo = tetFiles.find(fileNames);
		if (geo == tetFiles.end())
			continue;

		tm_id_index2[tmd.m_id] = i;

		vector<PBD::Vector3r> vertices = geo->second.first;
		vector<unsigned int> &tets = geo->second.second;

		const PBD::Matrix3r R = tmd.m_q.matrix();
		for (unsigned int j = 0; j < vertices.size(); j++)
		{
			vertices[j] = R * (vertices[j].cwiseProduct(tmd.m_scale)) + tmd.m_x;
		}

		m_model.addTetModel((unsigned int)vertices.size(), (unsigned int)tets.size() / 4, vertices.data(), tets.data());

		PBD::TetModel *tm = tetModels[tetModels.size() - 1];
		PBD::ParticleData &pd = m_model.getParticles();
		unsigned int offset = tm->getIndexOffset();

		for (unsigned int j = 0; j < tmd.m_staticParticles.size(); j++)
		{
			const unsigned int index = tmd.m_staticParticles[j] + offset;
			pd.setMass(index, 0.0);
		}

		// read visualization mesh
		if (tmd.m_modelFileVis != "")
		{
			if (objFiles.find(tmd.m_modelFileVis) != objFiles.end())
			{
				PBD::IndexedFaceMesh &visMesh = tm->getVisMesh();
				PBD::VertexData &vdVis = tm->getVisVertices();
				vdVis = objFiles[tmd.m_modelFileVis].first;
				visMesh = objFiles[tmd.m_modelFileVis].second;

				for (unsigned int j = 0; j < vdVis.size(); j++)
					vdVis.getPosition(j) = R * (vdVis.getPosition(j).cwiseProduct(tmd.m_scale)) + tmd.m_x;

				tm->updateMeshNormals(pd);
				tm->attachVisMesh(pd);
				tm->updateVisMesh(pd);
			}
		}

		tm->setRestitutionCoeff(tmd.m_restitutionCoeff);
		tm->setFrictionCoeff(tmd.m_frictionCoeff);

		tm->updateMeshNormals(pd);
	}

	initTetModelConstraints();

	// init collision objects for deformable models
	PBD::ParticleData &pd = m_model.getParticles();
	for (unsigned int i = 0; i < data.m_triangleModelData.size(); i++)
	{
		PBD::TriangleModel *tm = triModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		m_cd.addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TriangleModelCollisionObjectType, &pd.getPosition(offset), nVert);

	}
	for (unsigned int i = 0; i < data.m_tetModelData.size(); i++)
	{
		PBD::TetModel *tm = tetModels[i];
		unsigned int offset = tm->getIndexOffset();
		const unsigned int nVert = tm->getParticleMesh().numVertices();
		m_cd.addCollisionObjectWithoutGeometry(i, PBD::CollisionDetection::CollisionObject::TetModelCollisionObjectType, &pd.getPosition(offset), nVert);
	}

	//////////////////////////////////////////////////////////////////////////
	// joints
	//////////////////////////////////////////////////////////////////////////

	for (unsigned int i = 0; i < data.m_ballJointData.size(); i++)
	{
		const PBD::SceneLoader::BallJointData &jd = data.m_ballJointData[i];
		m_model.addBallJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position);
	}

	for (unsigned int i = 0; i < data.m_ballOnLineJointData.size(); i++)
	{
		const PBD::SceneLoader::BallOnLineJointData &jd = data.m_ballOnLineJointData[i];
		m_model.addBallOnLineJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_hingeJointData.size(); i++)
	{
		const PBD::SceneLoader::HingeJointData &jd = data.m_hingeJointData[i];
		m_model.addHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_universalJointData.size(); i++)
	{
		const PBD::SceneLoader::UniversalJointData &jd = data.m_universalJointData[i];
		m_model.addUniversalJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis[0], jd.m_axis[1]);
	}

	for (unsigned int i = 0; i < data.m_sliderJointData.size(); i++)
	{
		const PBD::SceneLoader::SliderJointData &jd = data.m_sliderJointData[i];
		m_model.addSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
	}

	for (unsigned int i = 0; i < data.m_rigidBodyParticleBallJointData.size(); i++)
	{
		const PBD::SceneLoader::RigidBodyParticleBallJointData &jd = data.m_rigidBodyParticleBallJointData[i];
		m_model.addRigidBodyParticleBallJoint(id_index[jd.m_bodyID[0]], jd.m_bodyID[1]);
	}

	for (unsigned int i = 0; i < data.m_targetAngleMotorHingeJointData.size(); i++)
	{
		const PBD::SceneLoader::TargetAngleMotorHingeJointData &jd = data.m_targetAngleMotorHingeJointData[i];
		m_model.addTargetAngleMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((PBD::TargetAngleMotorHingeJoint*)constraints[constraints.size() - 1])->setTargetAngle(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorHingeJointData.size(); i++)
	{
		const PBD::SceneLoader::TargetVelocityMotorHingeJointData &jd = data.m_targetVelocityMotorHingeJointData[i];
		m_model.addTargetVelocityMotorHingeJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((PBD::TargetVelocityMotorHingeJoint*)constraints[constraints.size() - 1])->setTargetAngularVelocity(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetPositionMotorSliderJointData.size(); i++)
	{
		const PBD::SceneLoader::TargetPositionMotorSliderJointData &jd = data.m_targetPositionMotorSliderJointData[i];
		m_model.addTargetPositionMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((PBD::TargetPositionMotorSliderJoint*)constraints[constraints.size() - 1])->setTargetPosition(jd.m_target);
	}

	for (unsigned int i = 0; i < data.m_targetVelocityMotorSliderJointData.size(); i++)
	{
		const PBD::SceneLoader::TargetVelocityMotorSliderJointData &jd = data.m_targetVelocityMotorSliderJointData[i];
		m_model.addTargetVelocityMotorSliderJoint(id_index[jd.m_bodyID[0]], id_index[jd.m_bodyID[1]], jd.m_position, jd.m_axis);
		((PBD::TargetVelocityMotorSliderJoint*)constraints[constraints.size() - 1])->setTargetVelocity(jd.m_target);
	}

	m_cd.updateAABBs(m_model);//对每个Rigidbody,TriangleModel,TetModel创建包围盒。其大小正好包围一个Mesh内的所有顶点
}

void PBDWrapper::initModel (const Real timeStepSize)
{
	SPH::TimeManager::getCurrent ()->setTimeStepSize(timeStepSize);

	m_sim.setCollisionDetection(m_model, &m_cd);
}


 void PBDWrapper::initTriangleModelConstraints()
 {
	// init constraints
	for (unsigned int cm = 0; cm < m_model.getTriangleModels().size(); cm++)
	{
		const unsigned int offset = m_model.getTriangleModels()[cm]->getIndexOffset();
		if (m_clothSimulationMethod == 1)
		{
			const unsigned int nEdges = m_model.getTriangleModels()[cm]->getParticleMesh().numEdges();
			const PBD::IndexedFaceMesh::Edge *edges = m_model.getTriangleModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				m_model.addDistanceConstraint(v1, v2);
			}
		}
		else if (m_clothSimulationMethod == 2)
		{

			PBD::TriangleModel::ParticleMesh &mesh = m_model.getTriangleModels()[cm]->getParticleMesh();
			const unsigned int *tris = mesh.getFaces().data();
			const unsigned int nFaces = mesh.numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				const unsigned int v1 = tris[3 * i] + offset;
				const unsigned int v2 = tris[3 * i + 1] + offset;
				const unsigned int v3 = tris[3 * i + 2] + offset;
				m_model.addFEMTriangleConstraint(v1, v2, v3);
			}
		}
		else if (m_clothSimulationMethod == 3)
		{
			PBD::TriangleModel::ParticleMesh &mesh = m_model.getTriangleModels()[cm]->getParticleMesh();
			const unsigned int *tris = mesh.getFaces().data();
			const unsigned int nFaces = mesh.numFaces();
			for (unsigned int i = 0; i < nFaces; i++)
			{
				const unsigned int v1 = tris[3 * i] + offset;
				const unsigned int v2 = tris[3 * i + 1] + offset;
				const unsigned int v3 = tris[3 * i + 2] + offset;
				m_model.addStrainTriangleConstraint(v1, v2, v3);
			}
		}
		if (m_bendingMethod != 0)
		{
			PBD::TriangleModel::ParticleMesh &mesh = m_model.getTriangleModels()[cm]->getParticleMesh();
			unsigned int nEdges = mesh.numEdges();
			const PBD::TriangleModel::ParticleMesh::Edge *edges = mesh.getEdges().data();
			const unsigned int *tris = mesh.getFaces().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const int tri1 = edges[i].m_face[0];
				const int tri2 = edges[i].m_face[1];
				if ((tri1 != 0xffffffff) && (tri2 != 0xffffffff))
				{
					// Find the triangle points which do not lie on the axis
					const int axisPoint1 = edges[i].m_vert[0];
					const int axisPoint2 = edges[i].m_vert[1];
					int point1 = -1;
					int point2 = -1;
					for (int j = 0; j < 3; j++)
					{
						if ((tris[3 * tri1 + j] != axisPoint1) && (tris[3 * tri1 + j] != axisPoint2))
						{
							point1 = tris[3 * tri1 + j];
							break;
						}
					}
					for (int j = 0; j < 3; j++)
					{
						if ((tris[3 * tri2 + j] != axisPoint1) && (tris[3 * tri2 + j] != axisPoint2))
						{
							point2 = tris[3 * tri2 + j];
							break;
						}
					}
					if ((point1 != -1) && (point2 != -1))
					{
						const unsigned int vertex1 = point1 + offset;
						const unsigned int vertex2 = point2 + offset;
						const unsigned int vertex3 = edges[i].m_vert[0] + offset;
						const unsigned int vertex4 = edges[i].m_vert[1] + offset;
						if (m_bendingMethod == 1)
							m_model.addDihedralConstraint(vertex1, vertex2, vertex3, vertex4);
						else if (m_bendingMethod == 2)
							m_model.addIsometricBendingConstraint(vertex1, vertex2, vertex3, vertex4);
					}
				}
			}
		}
	}
 }

 void PBDWrapper::initTetModelConstraints()
 {
	// init constraints
	for (unsigned int cm = 0; cm < m_model.getTetModels().size(); cm++)
	{
		const unsigned int offset = m_model.getTetModels()[cm]->getIndexOffset();
		const unsigned int nTets = m_model.getTetModels()[cm]->getParticleMesh().numTets();
		const unsigned int *tets = m_model.getTetModels()[cm]->getParticleMesh().getTets().data();
		const PBD::IndexedTetMesh::VertexTets *vTets = m_model.getTetModels()[cm]->getParticleMesh().getVertexTets().data();
		if (m_solidSimulationMethod == 1)
		{
			const unsigned int offset = m_model.getTetModels()[cm]->getIndexOffset();
			const unsigned int nEdges = m_model.getTetModels()[cm]->getParticleMesh().numEdges();
			const PBD::IndexedTetMesh::Edge *edges = m_model.getTetModels()[cm]->getParticleMesh().getEdges().data();
			for (unsigned int i = 0; i < nEdges; i++)
			{
				const unsigned int v1 = edges[i].m_vert[0] + offset;
				const unsigned int v2 = edges[i].m_vert[1] + offset;

				m_model.addDistanceConstraint(v1, v2);
			}

			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				m_model.addVolumeConstraint(v1, v2, v3, v4);
			}
		}
		else if (m_solidSimulationMethod == 2)
		{
			PBD::TetModel::ParticleMesh &mesh = m_model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				m_model.addFEMTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (m_solidSimulationMethod == 3)
		{
			PBD::TetModel::ParticleMesh &mesh = m_model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v1 = tets[4 * i] + offset;
				const unsigned int v2 = tets[4 * i + 1] + offset;
				const unsigned int v3 = tets[4 * i + 2] + offset;
				const unsigned int v4 = tets[4 * i + 3] + offset;

				m_model.addStrainTetConstraint(v1, v2, v3, v4);
			}
		}
		else if (m_solidSimulationMethod == 4)
		{
			PBD::TetModel::ParticleMesh &mesh = m_model.getTetModels()[cm]->getParticleMesh();
			for (unsigned int i = 0; i < nTets; i++)
			{
				const unsigned int v[4] = { tets[4 * i] + offset,
											tets[4 * i + 1] + offset,
											tets[4 * i + 2] + offset,
											tets[4 * i + 3] + offset };
				// Important: Divide position correction by the number of clusters
				// which contain the vertex.
				const unsigned int nc[4] = { vTets[v[0]].m_numTets, vTets[v[1]].m_numTets, vTets[v[2]].m_numTets, vTets[v[3]].m_numTets };
				m_model.addShapeMatchingConstraint(4, v, nc);
			}
		}
	}
 }
