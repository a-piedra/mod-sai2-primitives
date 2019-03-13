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

#include <GLFW/glfw3.h> // must be loaded after loading opengl/glew as part of Sai2Graphics
#include <signal.h>
#include <stdlib.h> // includes capability for exit

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
const string robot3_name = "panda3";
const string camera_name = "camera_top";

// simulation and control loop
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* robot3, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* robot3, ForceSensorSim* fsensor1, ForceSensorSim* fsensor2, ForceSensorSim* fsensor3, Simulation::Sai2Simulation* sim);

// control link and position in link
const string link_name = "link7";
//const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.20);
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.15);
const Eigen::Vector3d sensor_pos_in_link = Eigen::Vector3d(0.0,0.0,0.05);
Eigen::Vector3d sensed_force1;
Eigen::Vector3d sensed_force2;
Eigen::Vector3d sensed_force3;
Eigen::Vector3d sensed_moment1;
Eigen::Vector3d sensed_moment2;
Eigen::Vector3d sensed_moment3;

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
bool fRotCCW = false;
bool fRotCW = false;
bool guidance = true;
int guideRobot = 1;

bool changeOri = false;

double theta_deg = 0;
unsigned long long controller_counter = 0;
unsigned long long curr_control_time = 0;
Eigen::Vector3d currPos1;               // end-effector position of guidance robot 1
Eigen::Vector3d currPos2;               // end-effector position of guidance robot 2
Eigen::Vector3d currPos3;               // end-effector position of guidance robot 3
Eigen::Vector3d globalCurrPos1;
Eigen::Vector3d globalCurrPos2;
Eigen::Vector3d globalCurrPos3;
Eigen::Vector3d goalDelPos1;
Eigen::Vector3d goalDelPos2;
Eigen::Matrix<double, 2, 3> goalDelPos;
double goalDelOriZ1 = 0;
double goal_delOriZ2 = 0;

int numCollisions = 0;
Eigen::Vector3d globalPosGuide;
Eigen::MatrixXd collisionPoints;

#define DELTA_GOAL_ORI_Z            10      // change in goal end-effector z-rotation after key input (in degrees)
#define DELTA_GOAL_POS              0.005   // change in goal position after key input
#define POS_ERROR_THRESHOLD         0.3     // position error norm required to determine contact
#define MIN_CIRCLE_DIST             0.15    // minimum distance between origin of obstacle circles
#define REPEL_GAIN                  0.1     // gain for repelling payload away from collision spheres
#define MIN_REPEL_VEC_NORM          1e-3    // minimum norm of repel vector to perform normalization (to avoid singularities)
#define FOLLOWER_Z_PERTURBATION     0.01    // amount to perturb follower in z-direction to escape collision spheres mapped by follower

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
    auto robot3 = new Sai2Model::Sai2Model(robot_file, false, world_gravity, sim->getRobotBaseTransform(robot3_name));

    sim->getJointPositions(robot1_name, robot1->_q);
    sim->getJointPositions(robot2_name, robot2->_q);
    sim->getJointPositions(robot3_name, robot3->_q);
    robot1->updateModel();
    robot2->updateModel();
    robot3->updateModel();

    // add guidelines for active following
    auto guideline31 = new chai3d::cShapeLine(globalCurrPos3, globalCurrPos1);
    guideline31->setLineWidth(5);
    guideline31->m_colorPointA.setGreen();
    guideline31->m_colorPointB.setGreen();
    auto guideline32 = new chai3d::cShapeLine(globalCurrPos3, globalCurrPos2);
    guideline32->setLineWidth(5);
    guideline32->m_colorPointA.setGreen();
    guideline32->m_colorPointB.setGreen();
    graphics->_world->addChild(guideline31);
    graphics->_world->addChild(guideline32);

    // load simulated force sensors
	Eigen::Affine3d T_sensor = Eigen::Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
    auto fsensor1 = new ForceSensorSim(robot1_name, link_name, T_sensor, robot1);
    auto fsensor1_display = new ForceSensorDisplay(fsensor1, graphics);
    auto fsensor2 = new ForceSensorSim(robot2_name, link_name, T_sensor, robot2);
    auto fsensor2_display = new ForceSensorDisplay(fsensor2, graphics);
    auto fsensor3 = new ForceSensorSim(robot3_name, link_name, T_sensor, robot3);
    auto fsensor3_display = new ForceSensorDisplay(fsensor3, graphics);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
    thread sim_thread(simulation, robot1, robot2, robot3, fsensor1, fsensor2, fsensor3, sim);

	// next start the control thread
    thread ctrl_thread(control, robot1, robot2, robot3, sim);

    int prevNumCollisions = 0;
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
        // robot->updateModel();

        fsensor1_display->update();
        fsensor2_display->update();
        fsensor3_display->update();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
        graphics->updateGraphics(robot1_name, robot1);
        graphics->updateGraphics(robot2_name, robot2);
        graphics->updateGraphics(robot3_name, robot3);

        // add collision spheres to the environment when new collisions are detected
        if (numCollisions != prevNumCollisions)
        {
            auto collisionSphere = new chai3d::cShapeSphere(MIN_CIRCLE_DIST);
            collisionSphere->setUseTransparency(true);
            collisionSphere->setTransparencyLevel(0.9);
            collisionSphere->m_material->setBlueCornflower();
            graphics->_world->addChild(collisionSphere);
            collisionSphere->setLocalPos(collisionPoints.row(numCollisions - 1));
            prevNumCollisions = numCollisions;
        }

        // update guidelines for active following
        guideline31->m_pointA.set(globalCurrPos3[0], globalCurrPos3[1], globalCurrPos3[2]);
        guideline31->m_pointB.set(globalCurrPos1[0], globalCurrPos1[1], globalCurrPos1[2]);
        guideline32->m_pointA.set(globalCurrPos3[0], globalCurrPos3[1], globalCurrPos3[2]);
        guideline32->m_pointB.set(globalCurrPos2[0], globalCurrPos2[1], globalCurrPos2[2]);

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

//            goalDelPos1[1] += DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 1) += DELTA_GOAL_POS;
	    }
	    if (fTransXn) {
//	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
//	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;

//            goalDelPos1[1] -= DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 1) -= DELTA_GOAL_POS;
	    }
	    if (fTransYp) {
//	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
//	    	camera_pos = camera_pos + 0.05*cam_up_axis;
//	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;

//            goalDelPos1[0] -= DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 0) -= DELTA_GOAL_POS;
	    }
	    if (fTransYn) {
//	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
//	    	camera_pos = camera_pos - 0.05*cam_up_axis;
//	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;

//            goalDelPos1[0] += DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 0) += DELTA_GOAL_POS;
	    }
	    if (fTransZp) {
//	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
//	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;

//            goalDelPos1[2] += DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 2) += DELTA_GOAL_POS;
	    }	    
	    if (fTransZn) {
//	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
//	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;

//            goalDelPos1[2] -= DELTA_GOAL_POS;
            goalDelPos(guideRobot-1, 2) -= DELTA_GOAL_POS;
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
        if (fRotCCW) {
            changeOri = true;
            goalDelOriZ1 -= (M_PI/180.0) * (DELTA_GOAL_ORI_Z);
        }
        if (fRotCW) {
            changeOri = true;
            goalDelOriZ1 += (M_PI/180.0) * (DELTA_GOAL_ORI_Z);
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
//------------------------------------------------------------------------------
// this function checks if a line intersects a sphere
bool checkLineSphereIntersection(Eigen::Vector3d linePointA, Eigen::Vector3d linePointB, Eigen::Vector3d sphereCenter, double sphereRadius)
{
    double a = 0;
    double b = 0;
    double c = 0;
    for (int i = 0; i < 3; i++)
    {
        a += pow((linePointB[i] - linePointA[i]), 2);
        b += 2 * (linePointB[i] - linePointA[i]) * (linePointA[i] - sphereCenter[i]);
        c += pow(linePointA[i], 2) + pow(sphereCenter[i], 2) - (2 * (linePointA[i] * sphereCenter[i]));
    }
    c -= pow(sphereRadius, 2);

    double discriminant = pow(b, 2) - (4 * a * c);

    if (discriminant > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
//------------------------------------------------------------------------------
//Simple Point structure
//struct Point {
//     Point( double X, double Y ): x(X), y(Y) {}
//     double x;
//     double y;
//};

//Point Center(/*Center Coordinates*/);
//double radius = 5;

//for (double angle=0; angle<=2*PI; angle+=0.001)//You are using radians so you will have to increase by a very small amount
//     //This will have the coordinates  you want to draw a point at
//     Point( Center.x + radius*cos( angle ), Center.y + radius*sin( angle ) );

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* robot3, Simulation::Sai2Simulation* sim) {
	
    robot1->updateModel();
    robot2->updateModel();
    robot3->updateModel();
    int dof = robot1->dof();
    Eigen::VectorXd command_torques1 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd command_torques2 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd command_torques3 = Eigen::VectorXd::Zero(dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
    sensor_frame_in_link.translation() = sensor_pos_in_link;

	// Motion arm primitive - for initial alignment
    Sai2Primitives::RedundantArmMotion* motion_primitive1 = new Sai2Primitives::RedundantArmMotion(robot1, link_name, pos_in_link);
    Sai2Primitives::RedundantArmMotion* motion_primitive2 = new Sai2Primitives::RedundantArmMotion(robot2, link_name, pos_in_link);
    Sai2Primitives::RedundantArmMotion* motion_primitive3 = new Sai2Primitives::RedundantArmMotion(robot3, link_name, pos_in_link);
    motion_primitive1->enableGravComp();
    motion_primitive1->_posori_task->_kp_pos = 300;
    motion_primitive1->_posori_task->_kv_pos = 25;
    motion_primitive2->enableGravComp();
    motion_primitive2->_posori_task->_kp_pos = 300;
    motion_primitive2->_posori_task->_kv_pos = 25;
    motion_primitive3->enableGravComp();
    motion_primitive3->_posori_task->_kp_pos = 300;
    motion_primitive3->_posori_task->_kv_pos = 25;

	// Screwing primitive 
    Sai2Primitives::ScrewingAlignment* screwing_primitive1 = new Sai2Primitives::ScrewingAlignment(robot1, link_name, control_frame_in_link, sensor_frame_in_link);
    Sai2Primitives::ScrewingAlignment* screwing_primitive2 = new Sai2Primitives::ScrewingAlignment(robot2, link_name, control_frame_in_link, sensor_frame_in_link);
    Sai2Primitives::ScrewingAlignment* screwing_primitive3 = new Sai2Primitives::ScrewingAlignment(robot3, link_name, control_frame_in_link, sensor_frame_in_link);
    screwing_primitive1->enableGravComp();
    screwing_primitive2->enableGravComp();
    screwing_primitive3->enableGravComp();

    Eigen::Matrix3d R;
    Eigen::Matrix3d current_orientation1;
    Eigen::Matrix3d initial_orientation1;
    Eigen::Vector3d initial_position1;
    Eigen::Matrix3d current_orientation2;
    Eigen::Matrix3d initial_orientation2;
    Eigen::Vector3d initial_position2;
    Eigen::Matrix3d current_orientation3;
    Eigen::Matrix3d initial_orientation3;
    Eigen::Vector3d initial_position3;
    robot1->rotation(initial_orientation1, motion_primitive1->_link_name);
    robot1->position(initial_position1, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
    robot2->rotation(initial_orientation2, motion_primitive2->_link_name);
    robot2->position(initial_position2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
    robot3->rotation(initial_orientation3, motion_primitive3->_link_name);
    robot3->position(initial_position3, motion_primitive3->_link_name, motion_primitive3->_control_frame.translation());
    current_orientation1 = initial_orientation1;
    current_orientation2 = initial_orientation2;
    current_orientation3 = initial_orientation3;

    // prepare environment mapping
    // Eigen::VectorXd collisionPoints;
    // vector<vector<double> > collisionPoints;
    bool contact = false; // flag for sensing contact
//    Eigen::MatrixXd collisionPoints;
//    int numCollisions = 0;
    bool newCollisionPoint = false;
//    Eigen::Vector3d lastContactPos1;
//    for (int i = 0; i <= 3; i++)
//    {
//        lastContactPos1(i) = numeric_limits<double>::infinity();
//    }

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
        sim->getJointPositions(robot3_name, robot3->_q);
        sim->getJointVelocities(robot3_name, robot3->_dq);
        robot1->updateModel();
        robot2->updateModel();
        robot3->updateModel();

        // read end-effector positions
        robot1->position(currPos1, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
        robot1->positionInWorld(globalCurrPos1, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
        robot2->position(currPos2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
        robot2->positionInWorld(globalCurrPos2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
        robot3->position(currPos3, motion_primitive3->_link_name, motion_primitive3->_control_frame.translation());
        robot3->positionInWorld(globalCurrPos3, motion_primitive3->_link_name, motion_primitive3->_control_frame.translation());
        Eigen::Vector3d posGuide;
        Eigen::Vector3d posError;
        Eigen::Vector3d posFollower;

		// update tasks model
        screwing_primitive1->updatePrimitiveModel();
        screwing_primitive2->updatePrimitiveModel();
        screwing_primitive3->updatePrimitiveModel();
        motion_primitive1->updatePrimitiveModel();
        motion_primitive2->updatePrimitiveModel();
        motion_primitive3->updatePrimitiveModel();

		// update sensed values (need to put them back in sensor frame)
		Eigen::Matrix3d R_link;
        robot1->rotation(R_link, link_name);
		Eigen::Matrix3d R_sensor = R_link*sensor_frame_in_link.rotation();
        screwing_primitive1->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force1, - R_sensor.transpose() * sensed_moment1);
        screwing_primitive2->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force2, - R_sensor.transpose() * sensed_moment2);
        screwing_primitive3->updateSensedForceAndMoment(- R_sensor.transpose() * sensed_force3, - R_sensor.transpose() * sensed_moment3);

        if (guidance == true)
        {
            if ( guideRobot == 1 )
            {
                if (changeOri)
                {
                    changeOri = false;
                    R <<    cos(goalDelOriZ1),     -sin(goalDelOriZ1),        0,
                            sin(goalDelOriZ1),      cos(goalDelOriZ1),        0,
                                             0,                       0,        1;
                    motion_primitive1->_desired_orientation = R*initial_orientation1;
                }

                // set leader position and velocity
                motion_primitive1->_desired_position = initial_position1 + goalDelPos.row(0).transpose();
                motion_primitive1->_desired_velocity = Eigen::Vector3d(0,0,0);

//                posGuide = currPos1;
                robot1->position(posGuide, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
                robot1->positionInWorld(globalPosGuide, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
                posError = posGuide - motion_primitive1->_desired_position;
            }
            else if ( guideRobot == 2 )
            {
                // TODO: fix orientation change for second guidance end-effector
                if (changeOri)
                {
                    changeOri = false;
                    R <<    cos(goalDelOriZ1),     -sin(goalDelOriZ1),        0,
                            sin(goalDelOriZ1),      cos(goalDelOriZ1),        0,
                                             0,                       0,        1;
                    motion_primitive2->_desired_orientation = R*initial_orientation2;
                }

                // set leader position and velocity
                motion_primitive2->_desired_position = initial_position2 + goalDelPos.row(1).transpose();
                motion_primitive2->_desired_velocity = Eigen::Vector3d(0,0,0);

//                posGuide = currPos2;
                robot2->position(posGuide, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
                robot2->positionInWorld(globalPosGuide, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
                posError = posGuide - motion_primitive2->_desired_position;
            }

            // set follower position (under collision or in free space)
            // robot2->position(currPos2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
            // check for collision
            // Eigen::Vector3d posError1 = motion_primitive1->_posori_task->_current_position - motion_primitive1->_desired_position;
    //        if (posError1.norm() >= POS_ERROR_THRESHOLD)

            if ( (posError.array().abs() >= POS_ERROR_THRESHOLD).any() or (sensed_force1.array() != 0.0).any()
                 or (sensed_force2.array() != 0.0).any() )
            {
                contact = true;
                // set current collision point
                // ensure that current collision point is different enough from previous contact point
                if ( numCollisions == 0 )
                {
                    cout << "adding first collision" << endl;
                    numCollisions += 1;
                    collisionPoints.conservativeResize(numCollisions, 3);
                    collisionPoints.row(numCollisions - 1) = globalPosGuide;
                }
                else
                {
                    newCollisionPoint = true;
                    cout << "LOOK HERE: " << globalPosGuide.transpose() << endl << numCollisions << endl << endl;
                    // loop over collisionPoints and check if any is within MIN_CIRCLE_DIST from currPos1
                    for (int i = 0; i < numCollisions; i++)
                    {
                        cout << collisionPoints.row(i) << endl;
                        cout << (globalPosGuide.transpose() - collisionPoints.row(i)).norm() << endl;
                        if ( (globalPosGuide.transpose() - collisionPoints.row(i)).norm() < MIN_CIRCLE_DIST )
                        {
                            newCollisionPoint = false;
    //                        cout << "new collision detected" << endl;
                            break;
                        }
                    }
                    if ( newCollisionPoint == true )
                    {
    //                    cout << "new collision!" << endl;
                        numCollisions += 1;
                        collisionPoints.conservativeResize(numCollisions, 3);
                        collisionPoints.row(numCollisions - 1) = globalPosGuide;
                        newCollisionPoint = false;
                    }
                }
            }

            if ( (sensed_force3.array() != 0.0).any() )
            {
                contact = true;
                // set current collision point
                // ensure that current collision point is different enough from previous contact point
                if ( numCollisions == 0 )
                {
                    cout << "adding first collision" << endl;
                    numCollisions += 1;
                    collisionPoints.conservativeResize(numCollisions, 3);
                    collisionPoints.row(numCollisions - 1) = globalCurrPos3;
                }
                else
                {
                    newCollisionPoint = true;
                    cout << "LOOK HERE: " << globalCurrPos3.transpose() << endl << numCollisions << endl << endl;
                    // loop over collisionPoints and check if any is within MIN_CIRCLE_DIST from currPos1
                    for (int i = 0; i < numCollisions; i++)
                    {
                        cout << collisionPoints.row(i) << endl;
                        cout << (globalCurrPos3.transpose() - collisionPoints.row(i)).norm() << endl;
                        if ( (globalCurrPos3.transpose() - collisionPoints.row(i)).norm() < MIN_CIRCLE_DIST )
                        {
                            newCollisionPoint = false;
                            break;
                        }
                    }
                    if ( newCollisionPoint == true )
                    {
                        numCollisions += 1;
                        collisionPoints.conservativeResize(numCollisions, 3);
                        collisionPoints.row(numCollisions - 1) = globalCurrPos3;
                        newCollisionPoint = false;
                    }
                }
            }

            // enable active following by moving along average of guidelines
            // while remaining outside of collision spheress

            Eigen::Vector3d repelVec; repelVec.setZero();
            int offendingSpheres = 0;
            double weight1 = 0.5;
            double weight2 = 0.5;

            for (int i = 0; i < numCollisions; i++)
            {
                // ensure that follower is outside of collision spheres
                if ( (globalCurrPos3.transpose() - collisionPoints.row(i)).norm() < MIN_CIRCLE_DIST )
                {
                    repelVec += globalCurrPos3.transpose() - collisionPoints.row(i);
                    offendingSpheres++;
                }

                // if a guideline intersects with a collision sphere, give that guideline zero weight
                if ( checkLineSphereIntersection(globalCurrPos1.transpose(), globalCurrPos3.transpose(), collisionPoints.row(i), MIN_CIRCLE_DIST) == true )
                {
                    weight1 = 0;
                    cout << "guideline 1 interference" << endl;
                }
                if ( checkLineSphereIntersection(globalCurrPos2.transpose(), globalCurrPos3.transpose(), collisionPoints.row(i), MIN_CIRCLE_DIST) == true )
                {
                    weight2 = 0;
                    cout << "guideline 2 interference" << endl;
                }
            }
            // normalize the repelling vector if it is non-zero
            if (repelVec.norm() >= MIN_REPEL_VEC_NORM)
            {
                repelVec.normalize();
            }
            // perturb the follower if the repelling vector is zero
            else
            {
                motion_primitive3->_desired_position[2] += FOLLOWER_Z_PERTURBATION;
            }

            // move along the average of the guidelines and away from collision points
            // TODO: change this so the behavior is not jerky (do not use goalDelPos; instead, use something relative to globalCurrPos3)
            posFollower = weight1*goalDelPos.row(0).transpose() + weight2*goalDelPos.row(1).transpose();
            motion_primitive3->_desired_position = initial_position3 + posFollower + REPEL_GAIN*repelVec/max(1, offendingSpheres);
        }

        // otherwise, operate in free space
//        else
//        {
//            contact = false;
//            goalDelPos2 = goalDelPos1;
//            motion_primitive2->_desired_position = initial_position2 + goalDelPos.row(1).transpose();
//            motion_primitive2->_desired_orientation = motion_primitive1->_posori_task->_current_orientation;
//        }

        // set follower velocity
        motion_primitive3->_desired_velocity = Eigen::Vector3d(0,0,0);

        // set leader torques
        motion_primitive1->computeTorques(command_torques1);
        sim->setJointTorques(robot1_name, command_torques1);
        motion_primitive2->computeTorques(command_torques2);
        sim->setJointTorques(robot2_name, command_torques2);

        // set follower torques
        motion_primitive3->computeTorques(command_torques3);
        sim->setJointTorques(robot3_name, command_torques3);

        // update counter and timer
		controller_counter++;
		curr_control_time++;
		last_time = curr_time;

        if (controller_counter % 500 == 0)
        {
//            cout << contact << endl;
//            cout << collisionPoints << endl;
//            cout << currPos1 << endl;
//            cout << collisionPoints.row(collisionPoints.rows()-1) << endl;
//            cout << motion_primitive1->_posori_task->_current_position << endl;
//            if (numCollisions > 0){
//                cout << (currPos1.transpose() - collisionPoints.row(collisionPoints.rows()-1)).norm() << endl;
//            }
//            cout << newCollisionPoint << endl;
        }

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
void simulation(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* robot3, ForceSensorSim* fsensor1, ForceSensorSim* fsensor2, ForceSensorSim* fsensor3, Simulation::Sai2Simulation* sim) {
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

        // force sensor updates
        fsensor1->update(sim);
        fsensor1->getForce(sensed_force1);
        fsensor1->getMoment(sensed_moment1);
        fsensor2->update(sim);
        fsensor2->getForce(sensed_force2);
        fsensor2->getMoment(sensed_moment2);
        fsensor3->update(sim);
        fsensor3->getForce(sensed_force3);
        fsensor3->getMoment(sensed_moment3);

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
        case GLFW_KEY_Q:
            fRotCCW = set;
            break;
        case GLFW_KEY_E:
            fRotCW = set;
            break;
        case GLFW_KEY_G:
            guidance = true;
            break;
        case GLFW_KEY_F:
            guidance = false;
            break;
        case GLFW_KEY_1:
            guideRobot = 1;
            break;
        case GLFW_KEY_2:
            guideRobot = 2;
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

//        if ( (sensed_force.array() != 0.0).any() ) {
//            goalDelPos2 = goalDelPos;
//            Eigen::Vector3d proj_goal_on_F = (sensed_force.dot(goalDelPos2) / (sensed_force.squaredNorm())) * (sensed_force);
//            motion_primitive2->_desired_position = initial_position2 + (goalDelPos2 - proj_goal_on_F);
//            motion_primitive2->_desired_position = initial_position2 + goalDelPos2;
//            motion_primitive2->_desired_position = currPos2;
//        }

//            else if ( (currPos1 - lastContactPos1).norm() >= MIN_CIRCLE_DIST )
//            {
//                cout << "Hello World" << endl;
//                numCollisions += 1;
//                collisionPoints.conservativeResize(numCollisions, 3);
//                collisionPoints.row(numCollisions - 1) = motion_primitive1->_posori_task->_current_position;
//                // collisionPoints(numCollisions - 1) = motion_primitive1->_posori_task->_current_position;
//                // collisionPoints.push_back(motion_primitive1->_posori_task->_current_position);
//                lastContactPos1 = currPos1;
//            }

//        if (guideRobot == 1)
//        {
//            fsensor1->update(sim);
//            fsensor1->getForce(sensed_force1);
//            fsensor1->getMoment(sensed_moment1);
//        }
//        else
//        {
//            fsensor2->update(sim);
//            fsensor2->getForce(sensed_force2);
//            fsensor2->getMoment(sensed_moment2);
//        }
