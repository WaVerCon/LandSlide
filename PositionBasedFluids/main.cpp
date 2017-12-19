#define GLEW_STATIC
#include <GL/glew.h>
#include <cuda_gl_interop.h>
#include <GLFW/glfw3.h>
#include "ParticleSystem.h"
#include "Renderer.h"
#include "Scene.hpp"

#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>

//#include <IL\il.h>
//#include <IL\ilut.h>

//SPlishSPLASH
#include <Demos\Utils\Timing.h>
#include <iostream>
#include <Demos\Simulation\TimeManager.h>


#define cudaCheck(x) { cudaError_t err = x; if (err != cudaSuccess) { printf("Cuda error: %d in %s at %s:%d\n", err, #x, __FILE__, __LINE__); assert(0); } }


static const int width = 1280;
static const int height =720;
static const GLfloat lastX = (width / 2);
static const GLfloat lastY = (height / 2);
static float deltaTime = 0.0f;
static float lastFrame = 0.0f;
static int w = 0;
static bool ss = false;

static bool running = false;//step start or not

void initializeState(Scene &scene,ParticleSystem &system, tempSolver &tp, solverParams &tempParams);
void handleInput(GLFWwindow* window, ParticleSystem &system, Camera &cam);
//void saveVideo();
void mainUpdate(ParticleSystem &system, Renderer &render, Camera &cam, tempSolver &tp, solverParams &tempParams, LandSlide &scene);



int main() {
	//Checks for memory leaks in debug mode
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	cudaGLSetGLDevice(0);

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow* window = glfwCreateWindow(width, height, "Position Based Fluids", nullptr, nullptr);
	glfwMakeContextCurrent(window);

	//Set callbacks for keyboard and mouse
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();
	
	// Define the viewport dimensions
	glViewport(0, 0, width, height);



	//ilutInit();
	//ilInit();
	//ilutRenderer(ILUT_OPENGL);
	
	Camera cam = Camera();
	ParticleSystem system = ParticleSystem();
	tempSolver tp;
	solverParams tempParams;
	LandSlide scene("LandSlide");
	initializeState(scene,system, tp, tempParams);
	PBDWrapper& pbdWrapper = scene.getPBDWrapper();
	std::vector<PBD::RigidBody *> bodies = pbdWrapper.getSimulationModel().getRigidBodies();
	Renderer render(bodies);

	render.initVBOS(tempParams.numParticles, tempParams.numDiffuse, tp.triangles);
	
	//std::vector<Model*> models = scene.getModels();

	Timing::printAverageTimes();
	Timing::printTimeSums();

	while (!glfwWindowShouldClose(window)) {
		//Set frame times
		float currentFrame = float(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Check and call events
		glfwPollEvents();
		handleInput(window, system, cam);

		//Update physics and render
		mainUpdate(system, render, cam, tp, tempParams,scene);
		//saveVideo();

		// Swap the buffers
		glfwSwapBuffers(window);

		glfwSetCursorPos(window, lastX, lastY);
	}

	glfwTerminate();

	//cudaDeviceReset();

	return 0;
}

void handleInput(GLFWwindow* window, ParticleSystem &system, Camera &cam) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cam.ProcessKeyboard(FORWARD, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cam.ProcessKeyboard(BACKWARD, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cam.ProcessKeyboard(RIGHT, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cam.ProcessKeyboard(LEFT, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		cam.ProcessKeyboard(UP, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		cam.ProcessKeyboard(DOWN, deltaTime);
		
	if (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS)
		system.running = false;

	if (glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS) {
		running = !running;
		system.running = !system.running;
	}

	if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS)
		system.moveWall = true;

	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
		system.moveWall = false;

	if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS)
		ss = true;

	double xpos, ypos;
	glfwGetCursorPos(window, &xpos, &ypos);
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
		cam.ProcessMouseMovement((float(xpos) - lastX),lastY- float(ypos), deltaTime);
}

/*
void saveVideo() {
	if (ss == true) {
		ILuint imageID = ilGenImage();
		ilBindImage(imageID);
		ilutGLScreen();
		ilEnable(IL_FILE_OVERWRITE);
		std::string str = std::to_string(w) + ".png";
		const char * c = str.c_str();
		std::cout << c << std::endl;
		const size_t cSize = strlen(c) + 1;
		wchar_t* wc = new wchar_t[cSize];
		mbstowcs(wc, c, cSize);

		ilSaveImage(wc);
		delete wc;
		//ilutGLScreenie();
		ilDeleteImage(imageID);
		w++;
	}
}*/

void initializeState(Scene& scene,ParticleSystem &system, tempSolver &tp, solverParams &tempParams) {
	
	//FluidCloth scene("FluidCloth");
	scene.init(&tp, &tempParams);
	system.initialize(tp, tempParams);
	PBD::TimeManager::getCurrent()->setTimeStepSize(0.0083f);//set initial timestep
	//PBD::TimeManager::getCurrent()->setTimeStepSize(0.00083f);
}

void mainUpdate(ParticleSystem &system, Renderer &render, Camera &cam, tempSolver &tp, solverParams &tempParams,LandSlide &scene) {
	//Update the VBO
	void* positionsPtr;
	void* diffusePosPtr;
	void* diffuseVelPtr;
	void* velocitiesPtr;

	cudaCheck(cudaGraphicsMapResources(4, render.resources));
	size_t size;
	cudaGraphicsResourceGetMappedPointer(&positionsPtr, &size, render.resources[0]);
	cudaGraphicsResourceGetMappedPointer(&diffusePosPtr, &size, render.resources[1]);
	cudaGraphicsResourceGetMappedPointer(&diffuseVelPtr, &size, render.resources[2]);
	cudaGraphicsResourceGetMappedPointer(&velocitiesPtr, &size, render.resources[3]);
	system.getPositions((float*)positionsPtr, tempParams.numParticles);
	system.getDiffuse((float*)diffusePosPtr, (float*)diffuseVelPtr, tempParams.numDiffuse);
	system.getVelocities((float*)velocitiesPtr, tempParams.numParticles);
	cudaGraphicsUnmapResources(4, render.resources, 0);
	

	if (running) {
		PBDWrapper& pbdWrapper = scene.getPBDWrapper();
		std::vector<PBD::RigidBody *> bodies = pbdWrapper.getSimulationModel().getRigidBodies();
		//std::vector<Model*> models = scene.getModels();
		std::vector<SPH::RigidBodyParticleObject*> rigidBodies = scene.getRigidBodies();

		//Step physics
		system.updateWrapper(tempParams);

		std::vector<float3> tmp(tempParams.numParticles);

		cudaCheck(cudaMemcpy(tmp.data(), velocitiesPtr, tempParams.numParticles * sizeof(float3), cudaMemcpyDeviceToHost));

		system.updateBoundaryForces(rigidBodies, tp, tempParams);

		START_TIMING("SimStep - PBD");
		pbdWrapper.timeStep();
		STOP_TIMING_AVG;

		system.updateBoundaryParticles(rigidBodies, tp, tempParams);

	}
	//Render
	render.run(tempParams.numParticles, tempParams.numDiffuse, tempParams.numRigidParticles, tp.triangles, cam);
}