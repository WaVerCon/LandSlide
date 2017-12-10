#ifndef __RigidBodyObject_h__
#define __RigidBodyObject_h__

#include <Common/Common.h>

namespace SPH 
{	
	/** \brief Base class for rigid body objects. 
	*/

	class RigidBodyObject 
	{
	public:
		virtual ~RigidBodyObject() {};

		virtual bool isDynamic() const = 0;

		virtual Real const getMass() const = 0;
		virtual PBD::Vector3r const& getPosition() = 0;
		virtual PBD::Vector3r const& getVelocity() const = 0;
		virtual PBD::Matrix3r const& getRotation() const = 0;
		virtual PBD::Vector3r const& getAngularVelocity() const = 0;
		virtual void addForce(const PBD::Vector3r &f) = 0;
		virtual void addTorque(const PBD::Vector3r &t) = 0;
	};
	struct RigidBodyParticleObject{
		std::vector<PBD::Vector3r> x0;
		RigidBodyObject *rigidBody;
		unsigned int numberOfParticles;
	};
}

#endif 