/*
* Program for logging the Phasespace motion capture data.
* Usage:
* 	From the command line, enter ./logDataTest [ip] >> logfile.txt.
* 	ip is the ip address of the Phasespace server.
*/

// TODO: add quit handler

#include <bits/chrono.h>
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
#include "owl.hpp"
#include "serial_port.h"
#include "Eigen/Dense"

using std::string;
using namespace std;

int transmitPosition(Serial_Port *port, Eigen::Vector3f pos) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint8_t sysID = 50;
	uint8_t compID = 0;
	mavlink_msg_local_position_ned_pack(sysID, compID, &msg, 0, pos[0], pos[1], pos[2], 0, 0, 0);
  	port->write_message(msg);
	return 0;
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

  // Timer for metering data transmission
  chrono::time_point<chrono::system_clock> previousTime, currentTime;
	// Time for logging
	chrono::time_point<chrono::system_clock> startTime;

	Eigen::Vector3f position;
	Eigen::Quaternionf attitude;

	// DCM and quaternion for roatation from the motion capture system's reference frame to
	// North-East-Down
	Eigen::Matrix3f dcm_cam2ned; dcm_cam2ned << -1.0f, 0.0f, 0.0f,
																					     0.0f, 0.0f, -1.0f,
																							 0.0f, -1.0f, 0.0f;
	Eigen::Quaternionf dq_cam2ned(0.0f, 0.0f, 0.7071f, -0.7071f);

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;
  owl.frequency(30.0);
	
	// Define a rigid body tracker
	uint32_t trackerID = 0;
	owl.createTracker(trackerID, "rigid", "quadcopter");

	// Assign markers to rigid body (location in mm)
	// front, starboard, down
	//owl.assignMarker(trackerID, 5, "5", "pos=35,3,0");
	//owl.assignMarker(trackerID, 7, "7", "pos=35,-3,0");
	//owl.assignMarker(trackerID, 6, "6", "pos=-35,3,0");
	//owl.assignMarker(trackerID, 4, "4", "pos=-35,-3,0");
	//owl.assignMarker(trackerID, 3, "3", "pos=3,45,0");
	//owl.assignMarker(trackerID, 1, "1", "pos=-3,45,0");
	//owl.assignMarker(trackerID, 0, "0", "pos=3,-45,0");
	//owl.assignMarker(trackerID, 2, "2", "pos=-3,-45,0");
	
	owl.assignMarker(trackerID, 5, "5", "pos=-64,61,0");
	owl.assignMarker(trackerID, 7, "7", "pos=-64,66,0");
	owl.assignMarker(trackerID, 6, "6", "pos=-64,-66,0");
	owl.assignMarker(trackerID, 4, "4", "pos=-64,-61,0");
	owl.assignMarker(trackerID, 3, "3", "pos=47,74,0");
	owl.assignMarker(trackerID, 1, "1", "pos=53,74,0");
	owl.assignMarker(trackerID, 0, "0", "pos=53,-74,0");
	owl.assignMarker(trackerID, 2, "2", "pos=47,-74,0");

  // start streaming
  owl.streaming(1);

  // Initialize serial
  Serial_Port port(uartName, baudrate);
  port.start();
  if (!port.is_running()) {
    printf("\n");
    printf("ERROR INITIALIZING SERIAL");
    printf("\n");
    return 1;
  }

  ofstream logfile;
  logfile.open("output.csv");

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
					position = dcm_cam2ned*position.transpose();
					attitude = Eigen::Quaternionf(rigids[0].pose[3], rigids[0].pose[4], rigids[0].pose[5], rigids[0].pose[6])*dq_cam2ned;
					logfile << (chrono::duration_cast<chrono::milliseconds>(currentTime - startTime)).count() << "," 
									<< position[0] << "," << position[1] << "," << position[2] << "," 
									<< attitude.w() << "," << attitude.x() << "," << attitude.y() << "," << attitude.z()
									<< endl;

					// Check if it's time to send a position update
					auto elapsedMilliseconds = chrono::duration_cast<chrono::milliseconds>(currentTime - previousTime);
					if (elapsedMilliseconds.count() >= 1000) {
						previousTime = currentTime;
						transmitPosition(&port, position);
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
