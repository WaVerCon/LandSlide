#include "Renderer.h"
#include <cuda_gl_interop.h>

using namespace std;

static const int width = 1280;
static const int height = 720;
static const float zFar = 200.0f;
static const float zNear = 0.1f;
static const float aspectRatio = (float)width / (float)height;
static const glm::vec2 screenSize = glm::vec2(width, height);
static const glm::vec2 blurDirX = glm::vec2(1.0f / screenSize.x, 0.0f);
static const glm::vec2 blurDirY = glm::vec2(0.0f, 1.0f / screenSize.y);
static const glm::vec4 color = glm::vec4(.275f, 0.65f, 0.85f, 0.9f);
static float filterRadius = 3;
static const float radius = 0.05f;
static const float clothRadius = 0.03f;
static const float foamRadius = 0.01f;

PBD::Vector3r position[] = {
	PBD::Vector3r(-0.5f, 3.5f, 0.5f),
	PBD::Vector3r(0.5f, 3.5f, 0.5f),
	PBD::Vector3r(-0.5f, 4.5f, 0.5f),
	PBD::Vector3r(0.5f, 4.5f, 0.5f),
	PBD::Vector3r(-0.5f, 4.5f, -0.5f),
	PBD::Vector3r(0.5f, 4.5f, -0.5f),

	PBD::Vector3r(-0.5f, 3.5f, -0.5f),
	PBD::Vector3r(0.5f, 3.5f, -0.5f)

};
PBD::Vector3r normal[] = {
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1),
	PBD::Vector3r(0,0,1)
};

Renderer::Renderer(std::vector<PBD::RigidBody*> rigidBodies) :
	quad(Shader("Shaders/canvas.vert", "Shaders/canvas.frag")),
	rigidBody(Shader("Shaders/vs_smooth.glsl", "Shaders/fs_smooth.glsl")),
	particle(Shader("Shaders/vs_points.glsl", "Shaders/fs_points.glsl")),
	plane(Shader("plane.vert", "plane.frag")),
	cloth(Shader("clothMesh.vert", "clothMesh.frag")),
	depth(Shader("depth.vert", "depth.frag")),
	blur(BlurShader("blur.vert", "blur.frag")),
	thickness(Shader("depth.vert", "thickness.frag")),
	fluidFinal(Shader("fluidFinal.vert", "fluidFinal.frag")),
	foamDepth(Shader("foamDepth.vert", "foamDepth.frag")),
	foamThickness(Shader("foamThickness.vert", "foamThickness.frag")),
	foamIntensity(Shader("foamIntensity.vert", "foamIntensity.frag")),
	foamRadiance(Shader("radiance.vert", "radiance.frag")),
	finalFS(Shader("final.vert", "final.frag"))
{
	initFramebuffers();
	this->rigidBodies = rigidBodies;
	initRigidBodies();
}

Renderer::~Renderer() {
	cudaDeviceSynchronize();
	cudaGraphicsUnregisterResource(resources[0]);
	cudaGraphicsUnregisterResource(resources[1]);
	cudaGraphicsUnregisterResource(resources[2]);
	glDeleteBuffers(1, &positionVBO);
	glDeleteBuffers(1, &diffusePosVBO);
	glDeleteBuffers(1, &diffuseVelVBO);
}
void Renderer::initRigidBodies() {
	for (PBD::RigidBody* body : rigidBodies) {
		const PBD::VertexData &vd = body->getGeometry().getVertexData();
		const PBD::IndexedFaceMesh &mesh = body->getGeometry().getMesh();
		const PBD::Vector3r *vertexNormals = mesh.getVertexNormals().data();

		GLuint vao, vbo, ebo;
		glGenVertexArrays(1, &vao);
		glBindVertexArray(vao);
		glGenBuffers(1, &vbo);
		glBindBuffer(GL_ARRAY_BUFFER, vbo);
		glGenBuffers(1, &ebo);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,ebo);
		glBufferData(GL_ARRAY_BUFFER, (vd.size() +mesh.getVertexNormals().size()) * sizeof(PBD::Vector3r), NULL, GL_DYNAMIC_DRAW);
	
		glBufferData(GL_ELEMENT_ARRAY_BUFFER,3* mesh.numFaces()*sizeof(GLuint),mesh.getFaces().data(), GL_STATIC_DRAW);
	
		glEnableVertexAttribArray(0);

		glVertexAttribPointer(0, 3, GL_DOUBLE,GL_FALSE, 0, (GLvoid*)0);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 3, GL_DOUBLE,GL_FALSE, 0, (GLvoid*)0);

		glBindVertexArray(0);
		rigidBodyVAOs.push_back(vao);
		rigidBodyVBOs.push_back(vbo);
		rigidBodyEBOs.push_back(ebo);

	}




}
void Renderer::initVBOS(int numParticles, int numDiffuse, vector<int> triangles) {
	glGenBuffers(1, &positionVBO);
	glBindBuffer(GL_ARRAY_BUFFER, positionVBO);
	glBufferData(GL_ARRAY_BUFFER, numParticles * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	cudaGraphicsGLRegisterBuffer(&resources[0], positionVBO, cudaGraphicsRegisterFlagsWriteDiscard);

	glGenBuffers(1, &diffusePosVBO);
	glBindBuffer(GL_ARRAY_BUFFER, diffusePosVBO);
	glBufferData(GL_ARRAY_BUFFER, numDiffuse * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	cudaGraphicsGLRegisterBuffer(&resources[1], diffusePosVBO, cudaGraphicsRegisterFlagsWriteDiscard);

	glGenBuffers(1, &diffuseVelVBO);
	glBindBuffer(GL_ARRAY_BUFFER, diffuseVelVBO);
	glBufferData(GL_ARRAY_BUFFER, numDiffuse * 3 * sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	cudaGraphicsGLRegisterBuffer(&resources[2], diffuseVelVBO, cudaGraphicsRegisterFlagsWriteDiscard);

	glGenBuffers(1, &velocityVBO);
	glBindBuffer(GL_ARRAY_BUFFER, velocityVBO);
	glBufferData(GL_ARRAY_BUFFER,numParticles * 3 * sizeof(float), 0, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	cudaGraphicsGLRegisterBuffer(&resources[3], velocityVBO, cudaGraphicsRegisterFlagsWriteDiscard);

	if (triangles.size() > 0){//triangles may be empty.
		glGenBuffers(1, &indicesVBO);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesVBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * triangles.size(), &triangles[0], GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	}
}

void Renderer::run(int numParticles, int numDiffuse, int numCloth, vector<int> triangles, Camera &cam) {
	//Set camera
	glm::mat4 mView = cam.GetViewMatrix();
	glm::mat4 normalMatrix = glm::inverseTranspose(mView);
	glm::mat4 projection = glm::perspective(cam.Zoom, aspectRatio, zNear, zFar);
	//glm::mat4 projection = glm::infinitePerspective(cam.Zoom, aspectRatio, zNear);

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	//Clear buffer
	glClearColor(0.0f,0.0f, 0.0f, 1.0f);
	
	//----------------------Infinite Plane---------------------
	glUseProgram(plane.program);
	glBindFramebuffer(GL_FRAMEBUFFER, plane.fbo);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	

	plane.shaderVAOInfinitePlane();

	setMatrix(plane, mView, "mView");
	setMatrix(plane, projection, "projection");

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	
	
	//--------------------RIGIDBODIES--------------------
	renderRigidBodies(projection, mView);

	//--------------------RIGIDBODY_PARTICLES-----------------
	//renderSphere(projection, mView, cam, numParticles, numCloth,true);

	//--------------------FLUID_PARTICLES------------------
	renderSphere(projection, mView, cam, numParticles-numCloth, numCloth,false);

	glUseProgram(quad.program);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	quad.shaderVAOQuad();
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, plane.tex);
	glUniform1i(glGetUniformLocation(quad.program, "tex"),0);

	glDisable(GL_DEPTH_TEST);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	
	//--------------------CLOTH-------------------------
	//renderCloth(projection, mView, cam, numCloth, triangles);
	
	//--------------------WATER-------------------------
	//renderWater(projection, mView, cam, numParticles - numCloth, numCloth);

	//--------------------FOAM--------------------------
	//renderFoam(projection, mView, cam, numDiffuse);

	//--------------------Final - WATER & DIFFUSE-------------------------
	//
	//glUseProgram(finalFS.program);
	//glBindFramebuffer(GL_FRAMEBUFFER, 0);

	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//finalFS.shaderVAOQuad();

	//glActiveTexture(GL_TEXTURE0);
	//glBindTexture(GL_TEXTURE_2D, fluidFinal.tex);
	//GLint fluidMap = glGetUniformLocation(finalFS.program, "fluidMap");
	//glUniform1i(fluidMap, 0);

	//glActiveTexture(GL_TEXTURE1);
	//glBindTexture(GL_TEXTURE_2D, foamIntensity.tex);
	//GLint foamIntensityMap = glGetUniformLocation(finalFS.program, "foamIntensityMap");
	//glUniform1i(foamIntensityMap, 1);

	//glActiveTexture(GL_TEXTURE2);
	//glBindTexture(GL_TEXTURE_2D, foamRadiance.tex);
	//GLint foamRadianceMap = glGetUniformLocation(finalFS.program, "foamRadianceMap");
	//glUniform1i(foamRadianceMap, 2);

	//setVec2(foamRadiance, screenSize, "screenSize");

	//glEnable(GL_DEPTH_TEST);
	//glDepthMask(GL_TRUE);

	//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	
}

void Renderer::initFramebuffers() {
	//Infinite Plane buffer
	plane.initFBO(plane.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, plane.fbo);
	plane.initTexture(width, height, GL_RGBA, GL_RGBA32F, plane.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, plane.tex, 0);
	plane.initRenderBuffer(width, height, GL_DEPTH24_STENCIL8, plane.rbo);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, plane.rbo);

	quad.initFBO(quad.fbo);
	//rigidBody.initFBO(rigidBody.fbo);

	particle.initFBO(particle.fbo);

	//Cloth buffer
	cloth.initFBO(cloth.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, cloth.fbo);
	cloth.initTexture(width, height, GL_RGBA, GL_RGBA32F, cloth.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, cloth.tex, 0);

	//Depth buffer
	depth.initFBO(depth.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, depth.fbo);
	depth.initTexture(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, depth.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth.tex, 0);
	
	//Thickness buffer
	thickness.initFBO(thickness.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, thickness.fbo);
	thickness.initTexture(width, height, GL_RGBA, GL_RGBA32F, thickness.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, thickness.tex, 0);

	//Blur buffer
	blur.initFBO(blur.fboV);
	glBindFramebuffer(GL_FRAMEBUFFER, blur.fboV);
	blur.initTexture(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, blur.texV);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, blur.texV, 0);

	blur.initFBO(blur.fboH);
	glBindFramebuffer(GL_FRAMEBUFFER, blur.fboH);
	blur.initTexture(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, blur.texH);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, blur.texH, 0);

	//fluidFinal buffer
	fluidFinal.initFBO(fluidFinal.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fluidFinal.fbo);
	fluidFinal.initTexture(width, height, GL_RGBA, GL_RGBA32F, fluidFinal.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fluidFinal.tex, 0);

	//Foam Depth buffer
	foamDepth.initFBO(foamDepth.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, foamDepth.fbo);
	foamDepth.initTexture(width, height, GL_RGBA, GL_RGBA32F, foamDepth.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, foamDepth.tex, 0);
	foamDepth.initTexture(width, height, GL_DEPTH_COMPONENT, GL_DEPTH_COMPONENT, foamDepth.tex2);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, foamDepth.tex2, 0);

	//Foam Thickness buffer
	foamThickness.initFBO(foamThickness.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, foamThickness.fbo);
	foamThickness.initTexture(width, height, GL_RGBA, GL_RGBA32F, foamThickness.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, foamThickness.tex, 0);

	//Foam Intensity
	foamIntensity.initFBO(foamIntensity.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, foamIntensity.fbo);
	foamIntensity.initTexture(width, height, GL_RGBA, GL_RGBA32F, foamIntensity.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, foamIntensity.tex, 0);

	//Foam Radiance
	foamRadiance.initFBO(foamRadiance.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, foamRadiance.fbo);
	foamRadiance.initTexture(width, height, GL_RGBA, GL_RGBA32F, foamRadiance.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, foamRadiance.tex, 0);

	//Final buffer
	finalFS.initFBO(finalFS.fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, finalFS.fbo);
	finalFS.initTexture(width, height, GL_RGBA, GL_RGBA32F, finalFS.tex);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, finalFS.tex, 0);
}

void Renderer::setInt(Shader &shader, const int &x, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniform1i(loc, x);
}

void Renderer::setFloat(Shader &shader, const float &x, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniform1f(loc, x);
}

void Renderer::setVec2(Shader &shader, const glm::vec2 &v, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniform2f(loc, v.x, v.y);
}

void Renderer::setVec3(Shader &shader, const glm::vec3 &v, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniform3f(loc, v.x, v.y, v.z);
}

void Renderer::setVec4(Shader &shader, const glm::vec4 &v, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniform4f(loc, v.x, v.y, v.z, v.w);
}

void Renderer::setMatrix(Shader &shader, const glm::mat4 &m, const GLchar* name) {
	GLint loc = glGetUniformLocation(shader.program, name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}

void Renderer::renderWater(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numParticles, int numCloth) {
	//----------------------Particle Depth----------------------
	glUseProgram(depth.program);
	glBindFramebuffer(GL_FRAMEBUFFER, depth.fbo);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	glClear(GL_DEPTH_BUFFER_BIT);

	depth.bindPositionVAO(positionVBO, numCloth);
	
	setMatrix(depth, mView, "mView");
	setMatrix(depth, projection, "projection");
	setFloat(depth, radius, "pointRadius");
	setFloat(depth, width / aspectRatio * (1.0f / tanf(cam.Zoom * 0.5f)), "pointScale");
	
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	
	glDrawArrays(GL_POINTS, 0, (GLsizei)numParticles);

	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);

	//--------------------Particle Blur-------------------------
	glUseProgram(blur.program);

	//Vertical blur
	glBindFramebuffer(GL_FRAMEBUFFER, blur.fboV);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	glClear(GL_DEPTH_BUFFER_BIT);
	
	blur.shaderVAOQuad();
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, depth.tex);
	GLint depthMap = glGetUniformLocation(blur.program, "depthMap");
	glUniform1i(depthMap, 0);

	setMatrix(blur, projection, "projection");
	setVec2(blur, screenSize, "screenSize");
	setVec2(blur, blurDirY, "blurDir");
	setFloat(blur, filterRadius, "filterRadius");
	//setFloat(blur, width / aspectRatio * (1.0f / (tanf(cam.Zoom*0.5f))), "blurScale");
	setFloat(blur, 0.1f, "blurScale");

	glEnable(GL_DEPTH_TEST);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	//Horizontal blur
	glBindFramebuffer(GL_FRAMEBUFFER, blur.fboH);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	glClear(GL_DEPTH_BUFFER_BIT);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, blur.texV);
	depthMap = glGetUniformLocation(blur.program, "depthMap");
	glUniform1i(depthMap, 0);

	setVec2(blur, screenSize, "screenSize");
	setMatrix(blur, projection, "projection");
	setVec2(blur, blurDirX, "blurDir");
	setFloat(blur, filterRadius, "filterRadius");
	//setFloat(blur, width / aspectRatio * (1.0f / (tanf(cam.Zoom*0.5f))), "blurScale");
	setFloat(blur, 0.1f, "blurScale");

	glEnable(GL_DEPTH_TEST);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glDisable(GL_DEPTH_TEST);

	//--------------------Particle Thickness-------------------------
	glUseProgram(thickness.program);
	glBindFramebuffer(GL_FRAMEBUFFER, thickness.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	thickness.bindPositionVAO(positionVBO, numCloth);

	setMatrix(thickness, mView, "mView");
	setMatrix(thickness, projection, "projection");
	setFloat(depth, radius * 2.0f, "pointRadius");
	setFloat(depth, width / aspectRatio * (1.0f / tanf(cam.Zoom * 0.5f)), "pointScale");

	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);
	glBlendEquation(GL_FUNC_ADD);
	glDepthMask(GL_FALSE);
	//glEnable(GL_DEPTH_TEST);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	glDrawArrays(GL_POINTS, 0, (GLsizei)numParticles);

	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);

	//--------------------Particle fluidFinal-------------------------
	glUseProgram(fluidFinal.program);
	glBindFramebuffer(GL_FRAMEBUFFER, fluidFinal.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	fluidFinal.shaderVAOQuad();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, blur.texH);
	depthMap = glGetUniformLocation(fluidFinal.program, "depthMap");
	glUniform1i(depthMap, 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, thickness.tex);
	GLint thicknessMap = glGetUniformLocation(fluidFinal.program, "thicknessMap");
	glUniform1i(thicknessMap, 1);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, plane.tex);
	GLint sceneMap = glGetUniformLocation(fluidFinal.program, "sceneMap");
	glUniform1i(sceneMap, 2);

	setMatrix(fluidFinal, projection, "projection");
	setMatrix(fluidFinal, mView, "mView");
	setVec4(fluidFinal, color, "color");
	setVec2(fluidFinal, glm::vec2(1.0f / width, 1.0f / height), "invTexScale");

	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	glDisable(GL_DEPTH_TEST);
}

void Renderer::renderFoam(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numDiffuse) {
	//--------------------Foam Depth-------------------------
	glUseProgram(foamDepth.program);
	glBindFramebuffer(GL_FRAMEBUFFER, foamDepth.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	foamDepth.bindPositionVAO(diffusePosVBO, 0);

	setMatrix(foamDepth, projection, "projection");
	setMatrix(foamDepth, mView, "mView");
	setFloat(foamDepth, foamRadius, "pointRadius");
	setFloat(foamDepth, width / aspectRatio * (1.0f / tanf(cam.Zoom * 0.5f)), "pointScale");
	setFloat(foamDepth, tanf(cam.Zoom * 0.5f), "fov");

	glEnable(GL_DEPTH_TEST);
	//glDepthMask(GL_TRUE);
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

	glDrawArrays(GL_POINTS, 0, (GLsizei)numDiffuse);

	glDisable(GL_DEPTH_TEST);

	//--------------------Foam Thickness----------------------
	glUseProgram(foamThickness.program);
	glBindFramebuffer(GL_FRAMEBUFFER, foamThickness.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	foamThickness.bindPositionVAO(diffusePosVBO, 0);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, foamDepth.tex2);
	GLint foamD = glGetUniformLocation(foamThickness.program, "foamDepthMap");
	glUniform1i(foamD, 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depth.tex);
	GLint fluidDepth = glGetUniformLocation(foamThickness.program, "fluidDepthMap");
	glUniform1i(fluidDepth, 1);

	setMatrix(foamThickness, projection, "projection");
	setMatrix(foamThickness, mView, "mView");
	setFloat(foamThickness, foamRadius, "pointRadius");
	setFloat(foamThickness, width / aspectRatio * (1.0f / tanf(cam.Zoom * 0.5f)), "pointScale");
	setFloat(foamThickness, tanf(cam.Zoom * 0.5f), "fov");
	setVec2(foamThickness, screenSize, "screenSize");
	setFloat(foamThickness, zNear, "zNear");
	setFloat(foamThickness, zFar, "zFar");

	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);
	glBlendEquation(GL_FUNC_ADD);
	glDepthMask(GL_FALSE);

	glDrawArrays(GL_POINTS, 0, (GLsizei)numDiffuse);

	glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
	glDisable(GL_BLEND);

	//--------------------Foam Intensity----------------------
	glUseProgram(foamIntensity.program);
	glBindFramebuffer(GL_FRAMEBUFFER, foamIntensity.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	foamIntensity.shaderVAOQuad();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, foamThickness.tex);
	GLint thickness = glGetUniformLocation(foamIntensity.program, "thickness");
	glUniform1i(thickness, 0);

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

	//--------------------Foam Radiance----------------------
	glUseProgram(foamRadiance.program);
	glBindFramebuffer(GL_FRAMEBUFFER, foamRadiance.fbo);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	foamRadiance.shaderVAOQuad();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, foamDepth.tex2);
	foamD = glGetUniformLocation(foamRadiance.program, "foamDepthMap");
	glUniform1i(foamD, 0);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, depth.tex);
	fluidDepth = glGetUniformLocation(foamRadiance.program, "fluidDepthMap");
	glUniform1i(fluidDepth, 1);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, foamDepth.tex);
	GLint foamNormalH = glGetUniformLocation(foamRadiance.program, "foamNormalHMap");
	glUniform1i(foamNormalH, 2);

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, foamIntensity.tex);
	GLint foamIntensity = glGetUniformLocation(foamRadiance.program, "foamIntensityMap");
	glUniform1i(foamIntensity, 3);

	setMatrix(foamRadiance, mView, "mView");
	setMatrix(foamRadiance, projection, "projection");
	setFloat(foamRadiance, zNear, "zNear");
	setFloat(foamRadiance, zFar, "zFar");

	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void Renderer::renderCloth(glm::mat4 &projection, glm::mat4 &mView, Camera &cam, int numCloth, std::vector<int> triangles) {
	glUseProgram(cloth.program);
	glBindFramebuffer(GL_FRAMEBUFFER, plane.fbo);

	cloth.bindPositionVAO(positionVBO, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indicesVBO);

	setMatrix(cloth, projection, "projection");
	setMatrix(cloth, mView, "mView");
	
	glDrawElements(GL_TRIANGLES, triangles.size(), GL_UNSIGNED_INT, 0);
}
//void Renderer::renderObjects(glm::mat4 &projection, glm::mat4 &mView, std::vector<Model*>& models){
//	
//	glUseProgram(rigidBody.program);
//	glBindFramebuffer(GL_FRAMEBUFFER, plane.fbo);
//	
//	glm::mat4 model;
//	model= glm::scale(model, glm::vec3(10.0f));
//	model = glm::translate(model, glm::vec3(0, 1000, 0));
//	setMatrix(rigidBody, model, "model");
//	setMatrix(rigidBody, projection, "projection");
//	setMatrix(rigidBody, mView, "view");
//	setVec4(rigidBody, glm::vec4(0.3, 0.3f, 0.3, 1.0f), "default_color");
//	for (int i = 0; i < models.size(); ++i) {
//		models[i]->Draw(rigidBody);
//	}
//	
//
//}
void Renderer::renderRigidBodies(glm::mat4 &projection, glm::mat4 &mView) {
	glUseProgram(rigidBody.program);
	glBindFramebuffer(GL_FRAMEBUFFER,plane.fbo);
	glm::vec3 surfaceColor = glm::vec3(0.1f, 0.4f, 0.8f);
	
	setVec3(rigidBody, surfaceColor, "surface_color");
	setFloat(rigidBody, 5.0, "shininess");
	setFloat(rigidBody, 0.2f, "specular_factor");
	setMatrix(rigidBody, mView, "modelview_matrix");
	setMatrix(rigidBody, projection, "projection_matrix");
	
	//glBindVertexArray(rigidBodyVAOs[0]);
	//glBindBuffer(GL_ARRAY_BUFFER, rigidBodyVBOs[0]);
	//glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(position), position);
	//glBufferSubData(GL_ARRAY_BUFFER, sizeof(position), sizeof(normal), normal);
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rigidBodyEBOs[0]);
	//glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
	//glBindVertexArray(0);


	for (size_t i = 0; i < rigidBodies.size(); ++i) {
		const PBD::VertexData &vd = rigidBodies[i]->getGeometry().getVertexData();
		const PBD::IndexedFaceMesh &mesh = rigidBodies[i]->getGeometry().getMesh();
		const PBD::Vector3r *vertexNormals = mesh.getVertexNormals().data();
		glBindVertexArray(rigidBodyVAOs[i]);

		glBindBuffer(GL_ARRAY_BUFFER, rigidBodyVBOs[i]);
		glBufferSubData(GL_ARRAY_BUFFER, 0, vd.size() * sizeof(PBD::Vector3r), &vd.getPosition(0)[0]);
		glBufferSubData(GL_ARRAY_BUFFER, vd.size()*sizeof(PBD::Vector3r),mesh.getVertexNormals().size()*sizeof(PBD::Vector3r),&vertexNormals[0][0]);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rigidBodyEBOs[i]);
		glDrawElements(GL_TRIANGLES,3*mesh.numFaces(), GL_UNSIGNED_INT, 0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}	
}
void Renderer::renderSphere(const glm::mat4 &projection, const glm::mat4 &mView, Camera &cam, int numParticles, int numCloth,bool renderWall){
	
	float fluidColor[4] = { 0.3f, 0.5f, 0.9f, 1.0f };
	float surfaceColor[4] = { 0.2f, 0.6f, 0.8f, 1 };
	float speccolor[4] = { 1.0, 1.0, 1.0, 1.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, surfaceColor);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, speccolor);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 100.0);
	glColor3fv(surfaceColor);

	
	glUseProgram(particle.program);
	glBindFramebuffer(GL_FRAMEBUFFER, plane.fbo);

	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	setFloat(particle, (float)viewport[2], "viewport_width");
	float particleRadius = radius*0.5f;
	setFloat(particle, particleRadius, "radius");
	setFloat(particle, 25.0, "max_velocity");
	setVec3(particle, glm::vec3(fluidColor[0], fluidColor[1], fluidColor[2]), "color");


	setMatrix(particle, projection, "projection_matrix");
	setMatrix(particle, mView, "modelview_matrix");

	glDisable(GL_BLEND);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SPRITE);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glPointParameterf(GL_POINT_SPRITE_COORD_ORIGIN, GL_LOWER_LEFT);
	//glClear(GL_DEPTH_BUFFER_BIT);


	particle.bindPositionVAO(positionVBO, renderWall?0:numCloth);
	glBindVertexArray(particle.vao);

	glBindBuffer(GL_ARRAY_BUFFER, velocityVBO);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)(renderWall?0:numCloth * sizeof(float3)));
	glEnableVertexAttribArray(1);
	//particle.bindPositionVAO(positionVBO, 0);

	glDrawArrays(GL_POINTS, 0,renderWall? numCloth:numParticles);
	//glDrawArrays(GL_POINTS, 0, numCloth);
	
	glBindVertexArray(0);
	glUseProgram(0);

}