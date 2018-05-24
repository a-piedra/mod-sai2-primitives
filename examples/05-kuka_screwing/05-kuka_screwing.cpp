/*
 * Example of a controller for a Kuka arm made with the surface surface alignment primitive
 * 
 */

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "primitives/RedundantArmMotion.h" //ADDED
#include "primitives/SurfaceSurfaceAlignment.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of Sai2Graphics

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "resources/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string bottle_file = "resources/bottle.urdf";
const string bottle_name = "Bottle";

const string camera_name = "camera";

// simulation and control loop
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* bottle, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim);

// control link and position in link
const string link_name = "link6";
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.20);
const Eigen::Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.05);
Eigen::Vector3d sensed_force;
Eigen::Vector3d sensed_moment;

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.4);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	auto robot = new Sai2Model::Sai2Model(robot_file, false, world_gravity, sim->getRobotBaseTransform(robot_name));

	sim->getJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// load bottle
	auto bottle = new Sai2Model::Sai2Model(bottle_file, false, world_gravity, sim->getRobotBaseTransform(bottle_name));

	// load simulated force sensor
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	auto fsensor = new ForceSensorSim(robot_name, link_name, T_sensor, robot);
	auto fsensor_display = new ForceSensorDisplay(fsensor, graphics);


	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, bottle, fsensor, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

    	fsensor_display->update();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(bottle_name, bottle);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Eigen::Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fTransZp) {
	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;
	    }	    
	    if (fTransZn) {
	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	
	robot->updateModel();
	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
	sensor_frame_in_link.translation() = pos_in_link;

	// Surface Alignment primitive
	Sai2Primitives::SurfaceSurfaceAlignment* surf_alignment_primitive = new Sai2Primitives::SurfaceSurfaceAlignment(robot, link_name, control_frame_in_link, sensor_frame_in_link);
	Eigen::VectorXd surf_alignment_primitive_torques;
	surf_alignment_primitive->enableGravComp();

	// Motion arm primitive  //ADDED
	Sai2Primitives::RedundantArmMotion* motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, link_name, pos_in_link);
	Eigen::VectorXd motion_primitive_torques;
	motion_primitive->enableGravComp();

	Eigen::Matrix3d initial_orientation;
	Eigen::Vector3d initial_position;
	robot->rotation(initial_orientation, motion_primitive->_link_name);
	robot->position(initial_position, motion_primitive->_link_name, motion_primitive->_control_frame.translation());
	//


	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long controller_counter = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// update tasks model
		surf_alignment_primitive->updatePrimitiveModel();
		motion_primitive->updatePrimitiveModel(); //ADDED

		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		// update sensed values (need to put them back in sensor frame)
		Eigen::Matrix3d R_link;
		robot->rotation(R_link, link_name);
		Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
		surf_alignment_primitive->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);

		// cout << surf_alignment_primitive_torques << endl;

		// position part //ADDED
		// double circle_radius = 0.05;
		// double circle_freq = 0.33;
		// motion_primitive->_desired_position = initial_position + circle_radius * Eigen::Vector3d(0.0, sin(2*M_PI*circle_freq*time), 1-cos(2*M_PI*circle_freq*time));
		// motion_primitive->_desired_velocity = 2*M_PI*circle_freq*0.001*Eigen::Vector3d(0.0, cos(2*M_PI*circle_freq*time), sin(2*M_PI*circle_freq*time));
		Eigen::Matrix3d R;
			double theta = M_PI/2.0/1000.0 * (controller_counter);
			// R << cos(theta) , 0 , sin(theta),
			//           0     , 1 ,     0     ,
			//     -sin(theta) , 0 , cos(theta);
			R << cos(theta) , -sin(theta), 0,
			     sin(theta) , cos(theta) , 0,
			          0     ,      0     , 1;

			motion_primitive->_desired_orientation = R*initial_orientation;
			// motion_primitive->_desired_angular_velocity = Eigen::Vector3d::Zero();

		// motion_primitive->_desired_orientation = ?? ADD STUFF HERE

		// torques
		surf_alignment_primitive->computeTorques(surf_alignment_primitive_torques);
		motion_primitive->computeTorques(motion_primitive_torques); //ADDED

		//------ Final torques
		// command_torques = surf_alignment_primitive_torques;
		// command_torques = motion_primitive_torques;
		command_torques = surf_alignment_primitive_torques + motion_primitive_torques; //ADDED

		cout << "Command Torques" << endl;
		cout << command_torques << endl;

		// command_torques.setZero();

		// -------------------------------------------
		sim->setJointTorques(robot_name, command_torques);
		

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			// cout << time << endl;
			// cout << endl;
		}

		controller_counter++;

		// -------------------------------------------
		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* bottle, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// bottle controller
	Eigen::Vector2d bottle_qd = Eigen::Vector2d::Zero();
	Eigen::Vector2d bottle_torques = Eigen::Vector2d::Zero();

	// create a timer
	unsigned long long sim_counter = 0;
	double sim_freq = 2000.0;
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(sim_freq); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		double time = sim_counter/sim_freq;

		// force sensor update
		fsensor->update(sim);
		fsensor->getForce(sensed_force);
		fsensor->getMoment(sensed_moment);

		// bottle controller
		sim->getJointPositions (bottle_name, bottle->_q);
		sim->getJointVelocities (bottle_name, bottle->_dq);
	 bottle->updateKinematics();

		//desired velocity of the bottle - set to 0
	 		// bottle_qd(0) = 5.0/180.0*M_PI*sin(2*M_PI*0.12*time);
	 		// bottle_qd(1) = 7.0/180.0*M_PI*sin(2*M_PI*0.08*time);
	 bottle_qd(0) = 0;	//desired velocity of the bottle - set to 0
	 bottle_qd(1) = 0;

	 bottle_torques = -1000.0* (bottle->_q - bottle_qd) - 75.0*bottle->_dq;

		sim->setJointTorques (bottle_name, bottle_torques);

		// integrate forward
		sim->integrate(0.0005);

		sim_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}


//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
		switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}