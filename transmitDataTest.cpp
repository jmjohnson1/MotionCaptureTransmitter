/*
* Program for logging the Phasespace motion capture data.
* Usage:
* 	From the command line, enter ./logDataTest [ip] >> logfile.txt.
* 	ip is the ip address of the Phasespace server.
*/

// TODO: add quit handler

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <cstring>
#include <fstream>
#include <chrono>

#include <mavlink/common/mavlink.h>
#include "mavlink/common/mavlink_msg_vicon_position_estimate.h"
#include "owl.hpp"
#include "serial_port.h"
#include "udp_port.h"
#include "Eigen/Dense"

using std::string;
using namespace std;

int transmitPosition(Generic_Port *port, Eigen::Vector3f pos, uint64_t timestamp) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint8_t sysID = 50;
	uint8_t compID = 0;
	mavlink_msg_vicon_position_estimate_pack(sysID, compID, &msg, timestamp, pos[0], pos[1], pos[2], 0, 0, 0, 0);
  port->write_message(msg);
	return 0;
}

Eigen::Vector3f quat2eul(float qw, float qx, float qy, float qz) {
	Eigen::Vector3f eul;
	eul[0] = atan2(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy));
	eul[1] = -M_PI/2.0f + 2.0f*atan2(sqrt(1.0f + 2.0f*(qw*qy - qx*qz)), sqrt(1.0f - 2.0f*(qw*qy - qx*qz)));
	eul[2] = atan2(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
	return eul*180.0f/M_PI;
	}

int main(int argc, const char **argv)
{
  string address = argc > 1 ? argv[1] : "160.94.220.134";
  OWL::Context owl;
  OWL::Markers markers;
	OWL::Rigids rigids;

  // Serial port setup
  const char *uartName = "/dev/ttyUSB0";
  int baudrate = 57600;

	// UDP setup
	bool use_udp = true;
	const char *udp_ip = (char*)"127.0.0.1";
	const int udp_port = 14540;
	

  // Timer for metering data transmission
  chrono::time_point<chrono::system_clock> previousTime, currentTime;
	// Time for logging
	chrono::time_point<chrono::system_clock> startTime;

	Eigen::Vector3f position;
	Eigen::Quaternionf attitude;

	Eigen::Vector3f eulUnchanged;
	Eigen::Vector3f eulRotated;

	// DCM and quaternion for roatation from the motion capture system's reference frame to
	// North-East-Down
	Eigen::Matrix3f dcm_cam2ned; dcm_cam2ned << -1.0f, 0.0f, 0.0f,
																					     0.0f, 0.0f, -1.0f,
																							 0.0f, -1.0f, 0.0f;
	Eigen::Quaternionf dq_cam2ned(0.0f, 0.0f, -0.7071f, 0.7071f);

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;
  owl.frequency(30.0);
	
	// Define a rigid body tracker
	uint32_t trackerID = 0;
	owl.createTracker(trackerID, "rigid", "quadcopter");

	// Assign markers to rigid body (location in mm)
	// front, starboard, down
	owl.assignMarker(trackerID, 0, "0", "pos=48.5438,98.3303,33.2116");
	owl.assignMarker(trackerID, 2, "2", "pos=0,0,-0");
	owl.assignMarker(trackerID, 3, "3", "pos=50.0671,4.76837e-07,0.0730972");
	owl.assignMarker(trackerID, 4, "4", "pos=69.0167,-7.36591,3.25385");
	owl.assignMarker(trackerID, 5, "5", "pos=67.336,22.7949,2.81343");
	owl.assignMarker(trackerID, 6, "6", "pos=65.0018,-82.2399,39.0174");
	owl.assignMarker(trackerID, 7, "7", "pos=-69.5877,-96.1139,35.4213");

  // start streaming
  owl.streaming(1);

	Generic_Port *port;
  // Initialize serial
	if (use_udp) {
		port = new UDP_Port(udp_ip, udp_port);
	} else {
		port = new Serial_Port(uartName, baudrate);
	}

	mavlink_message_t dummy;

  port->start();
  if (!port->is_running()) {
    printf("\n");
    printf("ERROR INITIALIZING SERIAL");
    printf("\n");
    return 1;
  }

	cout << "Port running" << endl;
	
	//Gets UDP to move on
	port->read_message(dummy);

	cout << "dummy done";

  ofstream logfile;
  logfile.open("output.csv");
	logfile << "time,x,y,z,qw,qx,qy,qz" << endl;

  // Start recording time for message streaming rate
  previousTime = chrono::system_clock::now(); 
	startTime = chrono::system_clock::now();

  // main loop
  cout << "Main loop starting" << endl;
  while(owl.isOpen() && owl.property<int>("initialized")) {
		currentTime = chrono::system_clock::now();
		const OWL::Event *event = owl.nextEvent(1000);
		if(!event) continue;

		if(event->type_id() == OWL::Type::ERROR) {
				cerr << event->name() << ": " << event->str() << endl;
		}

		else if(event->type_id() == OWL::Type::FRAME) {
			// Check if there is rigid body data
			if (event->find("rigids", rigids) > 0) {
				if (rigids[0].cond > 0) {
					position << rigids[0].pose[0], rigids[0].pose[1], rigids[0].pose[2];
					position = dcm_cam2ned*position;
					position = position/1000.0f;
					attitude = Eigen::Quaternionf(rigids[0].pose[3], rigids[0].pose[4], rigids[0].pose[5], rigids[0].pose[6]);
					attitude = attitude.conjugate()*dq_cam2ned;

					//eulUnchanged = quat2eul(rigids[0].pose[3], rigids[0].pose[4], rigids[0].pose[5], rigids[0].pose[6]);
					//eulRotated = quat2eul(attitude.w(), attitude.x(), attitude.y(), attitude.z());

					//cout << "original: " <<eulUnchanged << std::endl;
					//cout << "rotated: " << eulRotated << endl;

					logfile << (chrono::duration_cast<chrono::milliseconds>(currentTime - startTime)).count() << "," 
									<< position[0] << "," << position[1] << "," << position[2] << "," 
									<< attitude.w() << "," << attitude.x() << "," << attitude.y() << "," << attitude.z()
									<< endl;

					// Check if it's time to send a position update
					auto elapsedMilliseconds = chrono::duration_cast<chrono::milliseconds>(currentTime - previousTime);
					if (elapsedMilliseconds.count() >= 100) {
						previousTime = currentTime;
						transmitPosition(port, position, (chrono::duration_cast<chrono::microseconds>(currentTime - startTime)).count());
					}
				}
			}

		}

  } // while

  logfile.close();
  owl.done();
  owl.close();

  return 0;
}
