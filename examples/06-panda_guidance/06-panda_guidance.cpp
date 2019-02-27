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
using namespace Eigen;

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
//const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.20);
const Eigen::Vector3d pos_in_link = Eigen::Vector3d(0.0,0.0,0.1);
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
//    auto pos_task = new Sai2Primitives::PosOriTask(robot1, link_name, pos_in_link);

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
	
    // prepare controller
    robot1->updateModel();
    robot2->updateModel();
    int dof = robot1->dof();
    Eigen::VectorXd command_torques1 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd command_torques2 = Eigen::VectorXd::Zero(dof);

	Eigen::Affine3d control_frame_in_link = Eigen::Affine3d::Identity();
	control_frame_in_link.translation() = pos_in_link;
	Eigen::Affine3d sensor_frame_in_link = Eigen::Affine3d::Identity();
	sensor_frame_in_link.translation() = pos_in_link;

    Eigen::MatrixXd N_prec = Eigen::MatrixXd::Identity(dof, dof);

    // Motion arm primitive - for initial alignment
    Sai2Primitives::RedundantArmMotion* motion_primitive1 = new Sai2Primitives::RedundantArmMotion(robot1, link_name, pos_in_link);
    Sai2Primitives::RedundantArmMotion* motion_primitive2 = new Sai2Primitives::RedundantArmMotion(robot2, link_name, pos_in_link);
    motion_primitive1->enableGravComp();
    motion_primitive2->enableGravComp();

    // joint tasks
    auto joint_task1 = new Sai2Primitives::JointTask(robot1);
    auto joint_task2 = new Sai2Primitives::JointTask(robot2);

    Eigen::VectorXd joint_task_torques1 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd joint_task_torques_np1 = Eigen::VectorXd::Zero(dof);
//    motion_primitive1->_joint_task->_kp = 50.0;
//    motion_primitive1->_joint_task->_kv = 11.0;
//    motion_primitive1->_joint_task->_ki = 50.0;

    Eigen::VectorXd joint_task_torques2 = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd joint_task_torques_np2 = Eigen::VectorXd::Zero(dof);
    joint_task2->_kp = 50.0;
    joint_task2->_kv = 11.0;
    joint_task2->_ki = 50.0;

//    VectorXd q_init_desired = VectorXd::Zero(dof);
//    // q_init_desired << 0, 0, 0, -10, 0, 90, 0;
//    q_init_desired << 0, 15, 0, -95, 0, 125, 0;
//    q_init_desired *= M_PI/180;
//    motion_primitive1->_joint_task->_desired_position = q_init_desired;

    // pos tasks
//    auto motion_primitive1->_posori_task = new Sai2Primitives::PosOriTask(robot1, link_name, pos_in_link);
    auto pos_task2 = new Sai2Primitives::PosOriTask(robot2, link_name, pos_in_link);

    VectorXd pos_task_torques1 = VectorXd::Zero(dof);
//    motion_primitive1->_posori_task->_kp_pos = 300.0;
//    motion_primitive1->_posori_task->_kv_pos = 25.0;
    // pos_task->_ki = 50.0;

    VectorXd pos_task_torques2 = VectorXd::Zero(dof);
    pos_task2->_kp_pos = 300.0;
    pos_task2->_kv_pos = 25.0;
    // pos_task->_ki = 50.0;

	// Screwing primitive 
    Sai2Primitives::ScrewingAlignment* screwing_primitive1 = new Sai2Primitives::ScrewingAlignment(robot1, link_name, control_frame_in_link, sensor_frame_in_link);
//    Sai2Primitives::ScrewingAlignment* screwing_primitive2 = new Sai2Primitives::ScrewingAlignment(robot1, link_name, control_frame_in_link, sensor_frame_in_link);
    screwing_primitive1->enableGravComp();
//    screwing_primitive2->enableGravComp();

    Eigen::Matrix3d current_orientation;
	Eigen::Matrix3d initial_orientation;
    Eigen::Vector3d initial_position1;
    Eigen::Vector3d initial_position2;
//    robot1->rotation(initial_orientation, motion_primitive1->_link_name);
//    robot1->position(initial_position1, motion_primitive1->_link_name, motion_primitive1->_control_frame.translation());
//    robot2->rotation(initial_orientation, motion_primitive2->_link_name);
//    robot2->position(initial_position2, motion_primitive2->_link_name, motion_primitive2->_control_frame.translation());
    robot1->rotation(initial_orientation, motion_primitive1->_posori_task->_link_name);
    robot1->position(initial_position1, motion_primitive1->_posori_task->_link_name, motion_primitive1->_posori_task->_control_frame.translation());
    robot2->rotation(initial_orientation, pos_task2->_link_name);
    robot2->position(initial_position2, pos_task2->_link_name, pos_task2->_control_frame.translation());

    // prepare observers
    VectorXd gravity1(dof), gravity2(dof), coriolis1(dof), coriolis2(dof);
    VectorXd r_mom = VectorXd::Zero(dof);
    VectorXd p = VectorXd::Zero(dof);
    VectorXd beta = VectorXd::Zero(dof);
    MatrixXd M_prev = MatrixXd::Zero(dof,dof);
    MatrixXd M_dot = MatrixXd::Zero(dof,dof);
    VectorXd integral_mom = VectorXd::Zero(dof);

    VectorXd K = VectorXd::Zero(dof);
    // K << 25, 25, 25, 25, 25, 25, 25;
    // K << 10, 15, 20, 25, 30, 50, 80;
    // K << 25, 25, 25, 25, 15, 10, 5;
    K << 2, 2, 2, 2, 1, 1, 1;
    MatrixXd K0 = MatrixXd::Identity(dof,dof);
    for(int i=0 ; i<dof ; i++)
    {
        K0(i,i) = K(i);
    }
    bool first_iteration = true;
    double contact_detection_treshold = 1;
    double link_detection_treshold = 0.15;
    int link_in_contact = -1;
    VectorXd r_filtered = VectorXd::Zero(dof);
    bool in_contact = false;
    bool in_contact_prev = false;

    MatrixXd Lambda_contact = MatrixXd::Zero(1,1);
    MatrixXd Jbar_contact = MatrixXd::Zero(dof,1);
    MatrixXd Jbar_contact_np = MatrixXd::Zero(dof,1);
    MatrixXd J_contact = MatrixXd::Zero(1,dof);
    MatrixXd J_contact_control = MatrixXd::Zero(1,dof);
    MatrixXd N_contact = MatrixXd::Identity(dof,dof);

    ButterworthFilter filter_r = ButterworthFilter(dof, 0.015);

    // link centers
    MatrixXd J_sample = MatrixXd::Zero(3,dof);
    MatrixXd Lambda_sample = MatrixXd::Zero(3,3);
    MatrixXd N_sample = MatrixXd::Zero(dof,dof);
    MatrixXd Jbar_sample = MatrixXd::Zero(dof,3);
    double J_norm_estimate = 1;

    vector<string> link_names;
    link_names.push_back("link1");
    link_names.push_back("link2");
    link_names.push_back("link3");
    link_names.push_back("link4");
    link_names.push_back("link5");
    link_names.push_back("link6");
    link_names.push_back("link7");

    vector<Vector3d> center_points;
    center_points.push_back(Vector3d(0.0, 0.0, -0.1));
    center_points.push_back(Vector3d(0.0, -0.07, 0.0));
    center_points.push_back(Vector3d(0.0, 0.0, -0.07));
    center_points.push_back(Vector3d(-0.088, 0.05, 0.0));
    center_points.push_back(Vector3d(0.0, 0.0, -0.1));
    center_points.push_back(Vector3d(0.088, 0.0, 0.0));
    center_points.push_back(Vector3d(0.0, 0.0, 0.075));

    int r_buffer_length = 20;
    MatrixXd r_buffer = MatrixXd::Zero(dof,r_buffer_length);
    VectorXd r_norm_buffer = VectorXd::Zero(r_buffer_length);
    VectorXd r_link_in_contact_buffer = -1*VectorXd::Ones(r_buffer_length);
    VectorXd r_link_in_contact_count = VectorXd::Zero(dof);
    VectorXd r_mean = VectorXd::Zero(dof);

    // torques to compensate contact at task level
    VectorXd contact_compensation_torques = VectorXd::Zero(dof);
    VectorXd contact_force_control_torques = VectorXd::Zero(dof);
    double direction = 0;

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

        M_prev = robot1->_M;

        // read joint positions, velocities, update model + gravity + coriolis
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot1->updateModel();
        robot1->gravityVector(gravity1);
        robot1->coriolisForce(coriolis1);
        robot2->updateModel();
        robot2->gravityVector(gravity2);
        robot2->coriolisForce(coriolis2);

        M_dot = (robot1->_M - M_prev) / (loop_dt);

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

//        motion_primitive1->_posori_task->_desired_position = initial_position1 + goal_delPos1;
//        motion_primitive1->_posori_task->_desired_velocity = Eigen::Vector3d(0,0,0);

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

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // NEW CODE //
        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // update tasks model
        N_prec.setIdentity();
        motion_primitive1->_posori_task->updateTaskModel(N_prec);
        N_prec = motion_primitive1->_posori_task->_N;

        // for direction monitoring
        motion_primitive1->_joint_task->updateTaskModel(N_prec);
        motion_primitive1->_joint_task->computeTorques(joint_task_torques_np1);

        if(in_contact)
        {
            Eigen::VectorXd Fc_est = VectorXd::Zero(3);
            if(link_in_contact == 0)
            {
                J_norm_estimate = 1;
                J_contact_control = r_filtered.transpose()/r_filtered.norm() * J_norm_estimate;
            }
            else
            {
                robot1->Jv(J_sample, link_names[link_in_contact], center_points[link_in_contact]);
                // robot->Jv(J_sample.transpose(), link_names[link_in_contact], center_points[link_in_contact]);
                JacobiSVD<MatrixXd> svd(J_sample, ComputeFullU | ComputeFullV);
                J_norm_estimate = svd.singularValues()(0);

                Eigen::MatrixXd J_inv = MatrixXd::Zero(dof,3);
                Eigen::MatrixXd Sigma_inv = MatrixXd::Zero(dof, 3);
                for(int i=0 ; i<3 ; i++)
                {
                    if(svd.singularValues()(i) > 1e-3)
                    {
                        Sigma_inv(i,i) = 1/svd.singularValues()(i);
                        // cout << svd.matrixV().col(i)*svd.matrixU().col(i).transpose()/svd.singularValues()(i) << endl;
                        // J_inv += svd.matrixV().col(i)*svd.matrixU().col(i).transpose()/svd.singularValues()(i);
                    }
                }
                J_inv = svd.matrixV() * Sigma_inv * svd.matrixU().transpose();
                // robot->operationalSpaceMatrices(Lambda_sample, Jbar_sample, N_sample, J_sample);
                // Fc_est = -Jbar_sample.transpose()*r_mom;
                Fc_est = J_inv.transpose()*r_mom;
                // Fc_est = -svd.solve(r_mom);
                J_contact_control = Fc_est.transpose()*J_sample/Fc_est.norm();
            }

            // robot->Jv(J_sample, link_names[link_in_contact], center_points[link_in_contact]);
            // robot->dynConsistentInverseJacobian(Jbar_sample, J_sample);

            J_contact = r_filtered.transpose()/r_filtered.norm();
            robot1->operationalSpaceMatrices(Lambda_contact, Jbar_contact_np, N_contact, J_contact, N_prec);
            J_contact = J_contact * N_prec;
            robot1->operationalSpaceMatrices(Lambda_contact, Jbar_contact, N_contact, J_contact, N_prec);
            N_prec = N_prec * N_contact;


            motion_primitive1->_joint_task->updateTaskModel(N_prec);
        }

        // compute momentum-based observer
        if (!first_iteration)
        {
            p = robot1->_M * robot1->_dq;
            beta = coriolis1 - M_dot * robot1->_dq;
            // beta = coriolis;
            integral_mom += (command_torques1 - beta + r_mom) * loop_dt;
            r_mom = K0 * (p - integral_mom);
        }

        // process velocity observer
        r_filtered = filter_r.update(r_mom);
        int i_buffer = controller_counter % r_buffer_length;
        r_norm_buffer(i_buffer) = r_filtered.norm();
        in_contact_prev = in_contact;
        if(r_link_in_contact_buffer[i_buffer] != -1)
        {
            r_link_in_contact_count(r_link_in_contact_buffer[i_buffer]) -= 1;
        }
        link_in_contact = -1;
        for(int i=0 ; i<dof ; i++)
        {
            if(r_filtered(i) > link_detection_treshold || -r_filtered(i) > link_detection_treshold)
            {
                link_in_contact = i;
            }
        }
        r_link_in_contact_buffer(i_buffer) = link_in_contact;
        if(link_in_contact > -1)
        {
            r_link_in_contact_count(link_in_contact) += 1;
        }

        bool buffer_all_true = true;
        bool buffer_all_false = true;

        for (int i = 0; i < r_buffer_length; i++)
        {
            buffer_all_true = buffer_all_true && (r_link_in_contact_buffer(i) != -1);
            buffer_all_false = buffer_all_false && (r_link_in_contact_buffer(i) == -1);
        }

        // for(int i=0 ; i<r_buffer_length ; i++)
        // {
        // 	buffer_all_true = buffer_all_true && r_norm_buffer(i) > contact_detection_treshold;
        // 	buffer_all_false = buffer_all_false && r_norm_buffer(i) < contact_detection_treshold;
        // }
        if(buffer_all_true)
        {
            in_contact = true;
        }
        else if(buffer_all_false)
        {
            in_contact = false;
        }
        else
        {
            in_contact = in_contact_prev;
        }

        if(in_contact)
        {
            int lc = 0;
            for(int i=0 ; i<dof ; i++)
            {
                if(r_link_in_contact_count(i) > r_link_in_contact_count(lc))
                {
                    lc = i;
                }
            }
            link_in_contact = lc;
        }

        if(!in_contact && in_contact_prev)
        {
            cout << "lost contact" << endl;
//            motion_primitive1->_joint_task->_desired_position = robot1->_q;
        }

        // compute torques
        motion_primitive1->_posori_task->computeTorques(pos_task_torques1);
        motion_primitive1->_joint_task->computeTorques(joint_task_torques1);

        // contact_compensation_torques.setZero(dof);
        contact_compensation_torques = motion_primitive1->_posori_task->_projected_jacobian.transpose() * motion_primitive1->_posori_task->_Jbar.transpose() * r_mom;
        contact_force_control_torques.setZero(dof);
        if(in_contact)
        {
            double target_contact_force = 0;
            direction = (double) (Jbar_contact_np.transpose() * (pos_task_torques1 + joint_task_torques_np1))(0);
            if(direction < 0)
            {
                target_contact_force = 5;
            }
            contact_force_control_torques = target_contact_force*J_contact_control.transpose();
            // contact_force_control_torques = r_mom;
        }

//        command_torques1 = pos_task_torques1 + joint_task_torques1 + coriolis1 - contact_compensation_torques - contact_force_control_torques;
         command_torques1 = pos_task_torques1 + joint_task_torques1 + coriolis1;

        if(first_iteration)
        {
            first_iteration = false;
        }

        // set follower velocity
        motion_primitive2->_desired_velocity = Eigen::Vector3d(0,0,0);

        // set leader torques
         motion_primitive1->computeTorques(command_torques1);
//        cout << command_torques1 << endl;
        sim->setJointTorques(robot1_name, command_torques1);

        // set follower torques
        motion_primitive2->computeTorques(command_torques2);
        sim->setJointTorques(robot2_name, command_torques2);

    //	if (controller_counter >= 1500){
    //		theta_deg = 0;
    //		return FINISHED;
    //	}

        if(controller_counter % 500 == 0)
        {
            // cout << in_contact << endl;
            // cout << r_filtered.transpose() * (command_torques + contact_compensation_torques) << endl;
            cout << "filtered observer :\n" << r_filtered.transpose() << endl;
            cout << "link in contact buffer :\n" << r_link_in_contact_buffer.transpose() << endl;
            cout << "link in contact count :\n" << r_link_in_contact_count.transpose() << endl;
            cout << "link in contact : " << link_in_contact << endl;
//            cout << "direction : " << Jbar_contact_np.transpose() * (pos_task_torques + joint_task_torques_np) << endl;
            // cout << J_contact * robot->_dq << endl;
            // cout << "velocity based observer :\n" << r_vel.transpose() << endl;
            // cout << "norm of r_vel : " << r_filtered.norm() << endl;
            // cout << "sensed torques :\n" << (filtered_sensed_torques - gravity).transpose() << endl;
            // cout << "link in contact : " << link_in_contact << endl;
            // cout << "norm J : " << J_norm_estimate << endl;
            cout << "position error :\n" << (motion_primitive1->_posori_task->_current_position - motion_primitive1->_posori_task->_desired_position).transpose() << endl;
            cout << endl;
        }

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
//        cout << sensed_force << endl;
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
