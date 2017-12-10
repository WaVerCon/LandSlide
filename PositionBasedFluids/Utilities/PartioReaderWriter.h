#ifndef __PartioReaderWriter_h__
#define __PartioReaderWriter_h__

#include <Common/Common.h>
#include <vector>

namespace SPH
{
	/** \brief Class for reading and writing partio files.
	*/
	using PBD::Vector3r;
	using PBD::Matrix3r; 
	class PartioReaderWriter
	{
	public:
		static bool readParticles(const std::string &fileName, const Vector3r &translation, const Matrix3r &rotation, const Real scale,
			std::vector<Vector3r> &pos, std::vector<Vector3r> &vel);

		static bool readParticles(const std::string &fileName, const Vector3r &translation, const Matrix3r &rotation, const Real scale,
			std::vector<Vector3r> &positions, std::vector<Vector3r> &velocities, Real &particleRadius);

		static bool readParticles(const std::string &fileName, const Vector3r &translation, const Matrix3r &rotation, const Real scale,
			std::vector<Vector3r> &pos);

		static void writeParticles(const std::string &fileName, const unsigned int numParticles, const Vector3r *particlePositions,
			const Vector3r *particleVelocities, const Real particleRadius);
	};

}

#endif
