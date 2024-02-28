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
#include "owl.hpp"
#include "serial_port.h"
#include "udp_port.h"
#include "Eigen/Dense"

using std::string;
using namespace std;

int transmitPosition(Generic_Port *port, Eigen::Vector3f pos, uint64_t timestamp) {
	mavlink_message_t msg;
	uint8_t sysID = 50;
	uint8_t compID = 0;
	mavlink_msg_vicon_position_estimate_pack(sysID, compID, &msg, timestamp, pos[0], pos[1], pos[2], 0, 0, 0, 0);
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

	// UDP setup
	bool use_udp = true;
	const char *udp_ip = (char*)"127.0.0.1";
	const int udp_port = 14540;

	int64_t timeOffset = 0; // Time offset relative to quad in us
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
	owl.assignMarker(trackerID, 0, "0", "pos=49.7728,91.5584,33.6128");
	owl.assignMarker(trackerID, 2, "2", "pos=0,0,-0");
	owl.assignMarker(trackerID, 3, "3", "pos=48.5332,0,3.24794");
	owl.assignMarker(trackerID, 4, "4", "pos=66.694,-10.9626,4.64607");
	owl.assignMarker(trackerID, 5, "5", "pos=65.6165,18.3679,2.10233");
	owl.assignMarker(trackerID, 6, "6", "pos=56.4292,-80.1433,33.7672");
	owl.assignMarker(trackerID, 7, "7", "pos=-71.8717,-89.6447,33.0399");

  // start streaming
  owl.streaming(1);

	// Setup port for MAVLink messages
	Generic_Port *port;
  // Initialize serial or udp
	if (use_udp) {
		port = new UDP_Port(udp_ip, udp_port);
	} else {
		port = new Serial_Port(uartName, baudrate);
	}

  port->start();
  if (!port->is_running()) {
    printf("\n");
    printf("ERROR INITIALIZING SERIAL");
    printf("\n");
    return 1;
  }

	//Gets UDP to move on
	mavlink_message_t dummy;
	port->read_message(dummy);

	// Logging setup
  ofstream logfile;
  logfile.open("mocap_.csv");
	logfile << "time_us,x,y,z,qw,qx,qy,qz,msgTransmitted" << endl;


	// Record server properties
	ofstream properties;
	properties.open("currentProperties");
	// int
	properties << "Opened: " << owl.property<int>("opened") << endl;
  properties << "Initialized: " << owl.property<int>("initialized") << endl;
	properties << "Streaming: " << owl.property<int>("streaming") << endl;
	properties << "SystemTimebase: " << owl.property<int>("systemtimebase") << endl;
	properties << "Timebase: " << owl.property<int>("timebase") << endl;
	// float
	properties << "Frequency: " << owl.property<float>("frequency") << endl;
	properties << "Scale: " << owl.property<float>("scale") << endl;
	// string
	properties << "Profile: " << owl.property<string>("profile") << endl;
	properties << "filters: " << owl.property<string>("filters") << endl;
	properties.close();

  // Start recording time for message streaming rate
  previousTime = chrono::system_clock::now(); 
	startTime = chrono::system_clock::now();


	// Let's figure out the difference between our clock and the quad's. We're
	// assuming the offset is constant and so is the transmission time. Bad
	// assumptions!
	//chrono::time_point<chrono::system_clock> timeout;
	//while (timeOffset==0) {
		
		//mavlink_message_t incommingMsg;
		//port->read_message(incommingMsg);


	//}


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
					position = dcm_cam2ned*position; // Change to a local NED frame
					position = position/1000.0f;  // Change units from mm to m
					attitude = Eigen::Quaternionf(rigids[0].pose[3], rigids[0].pose[4], rigids[0].pose[5], rigids[0].pose[6]);
					attitude = (attitude.conjugate()*dq_cam2ned).normalized();

					// Check if it's time to send a position update
					bool transmission = false;  // Records if a transmission is sent
					auto elapsedMilliseconds = chrono::duration_cast<chrono::milliseconds>(currentTime - previousTime);
					if (elapsedMilliseconds.count() >= 100) {
						previousTime = currentTime;
						transmitPosition(port, position, (chrono::duration_cast<chrono::microseconds>(currentTime - startTime)).count());
						transmission = true;
						cout << "Position transmitted" << endl;
					}

					// Record to logfile
					logfile << (chrono::duration_cast<chrono::microseconds>(currentTime - startTime)).count() << ","; 
					logfile << position[0] << "," << position[1] << "," << position[2] << ",";
					logfile << attitude.w() << "," << attitude.x() << "," << attitude.y() << "," << attitude.z();
					logfile << transmission;
					logfile << endl;
				}
			}

		}

  } // while

  logfile.close();
  owl.done();
  owl.close();

  return 0;
}
