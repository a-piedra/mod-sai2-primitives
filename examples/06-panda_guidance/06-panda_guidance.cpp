/*======================================================================================
 * 06-panda_guidance.cpp
 *
 * Example of a controller for a panda arm made with the screwing alignment primitive
 * ScrewingAlignment controller is used to align a bottle cap with a bottle before using
 * the RedundantArmMotion primitive to perform the screwing action.
 *
 *
 *======================================================================================*/


/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <math.h>
#include <stdio.h>
#include <time.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>

#include "primitives/RedundantArmMotion.h"
#include "primitives/ScrewingAlignment.h"
#include "timer/LoopTimer.h"
#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of Sai2Graphics
#include <signal.h>
#include <stdlib.h> //includes capability for exit

/* --------------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/

bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;

const string world_file = "resources/world_panda.urdf";
const string robot_file = "resources/panda_arm.urdf";
const string robot1_name = "panda1";
const string robot2_name = "panda2";
const string camera_name = "camera_top";

// simulation and control loop
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim);

// control link and position in link
const string link_name = "link7";
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.20);
const Eigen::Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.05);
Eigen::Vector3d sensed_force;
Eigen::Vector3d sensed_moment;

//*
// Recording data
//*
// Get current date/time, format is YYYY-MM-DD_HH-mm-ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

    return buf;
}
// file name for recording data
const string record_file = "../../../src/dataPlotter/Data/dataPandaScrew_" + currentDateTime() + ".csv";
// helper function to combine data into vector
void recordData(double curr_time, int dof, Eigen::Vector3d sensed_force, Eigen::Vector3d sensed_moment, Eigen::VectorXd command_torques);
// helper function to record data to CSV file
void recordToCSV(Eigen::VectorXd &v, const std::string &filename);

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

double theta_deg = 0;
unsigned long long controller_counter = 0;
unsigned long long curr_control_time = 0;
Eigen::Vector3d curr_Pos1;
Eigen::Vector3d curr_Pos2;
Eigen::Vector3d goal_delPos1;
Eigen::Vector3d goal_delPos2;

#define DELTA_GOAL_POS  0.005    // change in goal position after key input

/* =======================================================================================
   MAIN LOOP
========================================================================================== */

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
	sim->setCoeffFrictionStatic(0.05); //might need to increase friction when screws are added?
	sim->setCoeffFrictionDynamic(0);

	// load robots
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
    auto robot1 = new Sai2Model::Sai2Model(robot_file, false, world_gravity, sim->getRobotBaseTransform(robot1_name));
    auto robot2 = new Sai2Model::Sai2Model(robot_file, false, world_gravity, sim->getRobotBaseTransform(robot2_name));

    sim->getJointPositions(robot1_name, robot1->_q);
    sim->getJointPositions(robot2_name, robot2->_q);
    robot1->updateModel();
    robot2->updateModel();

	// load simulated force sensor
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
    auto fsensor = new ForceSensorSim(robot1_name, link_name, T_sensor, robot1);
	auto fsensor_display = new ForceSensorDisplay(fsensor, graphics);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
    thread sim_thread(simulation, robot1, robot2, fsensor, sim);

	// next start the control thread
    thread ctrl_thread(control, robot1, robot2, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
        // robot->updateModel();

    	fsensor_display->update();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot1_name, robot1);
        graphics->updateGraphics(robot2_name, robot2);
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
//	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
//	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;

            goal_delPos1[1] += DELTA_GOAL_POS;
	    }
	    if (fTransXn) {
//	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
//	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;

            goal_delPos1[1] -= DELTA_GOAL_POS;
	    }
	    if (fTransYp) {
//	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
//	    	camera_pos = camera_pos + 0.05*cam_up_axis;
//	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;

            goal_delPos1[0] -= DELTA_GOAL_POS;
	    }
	    if (fTransYn) {
//	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
//	    	camera_pos = camera_pos - 0.05*cam_up_axis;
//	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;

            goal_delPos1[0] += DELTA_GOAL_POS;
	    }
	    if (fTransZp) {
//	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
//	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;

            goal_delPos1[2] += DELTA_GOAL_POS;
	    }	    
	    if (fTransZn) {
//	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
//	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;

            goal_delPos1[2] -= DELTA_GOAL_POS;
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

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
void recordData(double curr_time, int dof, Eigen::Vector3d sensed_force, Eigen::Vector3d sensed_moment, Eigen::VectorXd command_torques)
{
	// 1 value for time, 3 values for forces, 3 values for moments, and dof values for command torques  
	Eigen::VectorXd data = Eigen::VectorXd::Zero(7 + dof);
	data(0) = curr_time;
	
	for (int i = 0; i < 3; i++) {
		data(1 + i) = sensed_force(i);
		data(4 + i) = sensed_moment(i);
	}
	for (int i = 0; i < dof; i++) {
		data(7 + i) = command_torques(i);
	}
	recordToCSV(data, record_file);
}
//------------------------------------------------------------------------------
void recordToCSV(Eigen::VectorXd &v, const std::string &filename)
{
    std::ofstream file(filename,  std::ofstream::out | std::ofstream::app);
    if (file.is_open())
    {
        for(int i=0; i < v.rows(); ++i)
        {
            file << v(i);
            if(i!=v.rows()-1) file << ", ";
        }
        file << "\n";
    }
    //    file.close();
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Simulation::Sai2Simulation* sim) {
	
    robot1->updateModel();
    robot2->updateModel();
    int dof = robot1->dof();
    Eigen::VectorXd command_torques1 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd command_torques2 = Eigen::VectorXd::Zero(dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
	sensor_frame_in_link.translation() = pos_in_link;

	// Motion arm primitive - for initial alignment
    Sai2Primitives::RedundantArmMotion* motion_primitive1 = new Sai2Primitives::RedundantArmMotion(robot1, link_name, pos_in_link);
    Sai2Primitives::RedundantArmMotion* motion_primitive2 = new Sai2Primitives::RedundantArmMotion(robot2, link_name, pos_in_link);
    motion_primitive1->enableGravComp();
    motion_primitive2->enableGravComp();


	// Screwing primitive 
    Sai2Primitives::ScrewingAlignment* screwing_primitive1 = new Sai2Primitives::ScrewingAlignment(robot1, link_name, control_frame_in_link, sensor_frame_in_link);
//    Sai2Primitives::ScrewingAlignment* screwing_primitive2 = new Sai2Primitives::ScrewingAlignment(robot1, link_name, control_frame_in_link, sensor_frame_in_link);
    screwing_primitive1->enableGravComp();
//    screwing_primitive2->enableGravComp();

    Eigen::Matrix3d current_orientation;
	Eigen::Matrix3d initial_orientation;
    Eigen::Vector3d initial_position1;
    Eigen::Vector3d initial_position2;
    robot1->rotation(initial_orientation, motion_primitive1->_link_name);
    robot1->position(initial_position1, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
    robot2->rotation(initial_orientation, motion_primitive2->_link_name);
    robot2->position(initial_position2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long screw_counter = 0;
	bool setup_flag = false;		// triggered when setup of position and orientation are complete
	double theta_deg = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		double time = controller_counter/control_freq;

		// read joint positions, velocities, update model
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot1->updateModel();
        robot2->updateModel();

		// update tasks model
        screwing_primitive1->updatePrimitiveModel(); //ADDED
        motion_primitive1->updatePrimitiveModel();
        motion_primitive2->updatePrimitiveModel();

		// update sensed values (need to put them back in sensor frame)
		Eigen::Matrix3d R_link;
        robot1->rotation(R_link, link_name);
		Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
        screwing_primitive1->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force, - R_sensor.transpose() * sensed_moment);

        // orientation part
    //	if(theta_deg <= APPROACH_ANGLE)
    //	{
    //		Eigen::Matrix3d R;
    //		double theta = -M_PI/2.0/500.0 * (controller_counter);
    //		theta_deg = -theta*180/M_PI;
    //		R <<      1     ,      0      ,      0     ,
    //	   		      0     ,  cos(theta) , -sin(theta),
    //	       		  0     ,  sin(theta) ,  cos(theta);

    //		motion_primitive->_desired_orientation = R*initial_orientation;
    //    }

        // set leader position and velocity
        motion_primitive1->_desired_position = initial_position1 + goal_delPos1;
        motion_primitive1->_desired_velocity = Eigen::Vector3d(0,0,0);

        // set follower position (under collision or in free space)
        robot2->position(curr_Pos2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
        // check for collision
        if ( (sensed_force.array() != 0.0).any() ) {
//            goal_delPos2 = goal_delPos1;
            Eigen::Vector3d proj_goal_on_F = (sensed_force.dot(goal_delPos2) / (sensed_force.squaredNorm())) * (sensed_force);
//            motion_primitive2->_desired_position = initial_position2 + (goal_delPos2 - proj_goal_on_F);
//            motion_primitive2->_desired_position = initial_position2 + goal_delPos2;
            motion_primitive2->_desired_position = curr_Pos2;
        }
        // otherwise, operate in free space
        else {
            goal_delPos2 = goal_delPos1;
            motion_primitive2->_desired_position = initial_position2 + goal_delPos2;
        }

        // set follower velocity
        motion_primitive2->_desired_velocity = Eigen::Vector3d(0,0,0);

        // set leader torques
        motion_primitive1->computeTorques(command_torques1);
        sim->setJointTorques(robot1_name, command_torques1);

        // set follower torques
        motion_primitive2->computeTorques(command_torques2);
        sim->setJointTorques(robot2_name, command_torques2);

    //	if (controller_counter >= 1500){
    //		theta_deg = 0;
    //		return FINISHED;
    //	}

        // update counter and timer
		controller_counter++;
		curr_control_time++;
		last_time = curr_time;

		//*
		// Recording data
		//*
//		if (curr_time <= 8.0)
//		{
//			recordData(curr_time, dof, sensed_force, sensed_moment, command_torques);
//		}
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */

void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, ForceSensorSim* fsensor, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

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
        cout << sensed_force << endl;
        fsensor->getMoment(sensed_moment);

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
    int windowW = 1.0 * screenH;
    int windowH = 0.7 * screenH;
    int windowPosY = (screenH - windowH*2) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Bottle Cap Screwing Demo", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

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
