#ifndef RENDERER_H
#define RENDERER_H

#include "common.h"
#include "Shader.hpp"
#include<Demos\Utils\Timing.h>
#include "Camera.h"
#include"PositionBasedDynamicsWrapper\PBDRigidBody.h"
#include"Model.h"


class Renderer {
public:
	Renderer(std::vector<PBD::RigidBody*> rigidBodies);
	~Renderer();
	void initVBOS(int numParticles, int numDiffuse, std::vector<int> triangles);
	void run(int numParticles, int numDiffuse, int numCloth, std::vector<int> triangles, Camera &cam);

	cudaGraphicsResource *resources[4];
	GLuint velocityVBO;//增加速度的绘制
	GLuint positionVBO;
	GLuint indicesVBO;
	GLuint diffusePosVBO;
	GLuint diffuseVelVBO;
	std::vector<GLuint> rigidBodyVAOs;
	std::vector<GLuint> rigidBodyVBOs;
	std::vector<GLuint> rigidBodyEBOs;
	std::vector<PBD::RigidBody*> rigidBodies;

	Shader rigidBody;
	Shader particle;
	Shader quad;
	//Shader rock;
	Shader plane;
	Shader cloth;
	Shader depth;
	BlurShader blur;
	Shader thickness;
	Shader fluidFinal;
	Shader foamDepth;
	Shader foamThickness;
	Shader foamIntensity;
	Shader foamRadiance;
	Shader finalFS;

private:
	void renderWater(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numParticles, int numCloth);
	void renderFoam(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numDiffuse);
	void renderCloth(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numCloth, std::vector<int> triangles);
	//void renderObjects(glm::mat4 &projection, glm::mat4 &mView, std::vector<Model*>& models);

	void renderRigidBodies(glm::mat4 &projection, glm::mat4 &mView);
	void renderSphere(const glm::mat4 &projection, const glm::mat4 &mView, Camera &cam, int numParticles, int numCloth,bool renderWall);
	void initFramebuffers();
	void setInt(Shader &shader, const int &x, const GLchar* name);
	void setFloat(Shader &shader, const float &x, const GLchar* name);
	void setVec2(Shader &shader, const glm::vec2 &v, const GLchar* name);
	void setVec3(Shader &shader, const glm::vec3 &v, const GLchar* name);
	void setVec4(Shader &shader, const glm::vec4 &v, const GLchar* name);
	void setMatrix(Shader &shader, const glm::mat4 &m, const GLchar* name);
	void initRigidBodies();
	
};

#endif
