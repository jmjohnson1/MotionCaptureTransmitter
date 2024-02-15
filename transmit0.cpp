#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <chrono>
#include <mavlink/common/mavlink.h>
#include "Eigen/Dense"

#include "serial_port.h"
#include "udp_port.h"

using std::string;
using namespace std;

// MAVLink system settings
uint8_t sysID = 51;
uint8_t compID = MAV_COMP_ID_GPS;
uint8_t mavType = MAV_TYPE_GPS;
uint8_t autopilot = MAV_AUTOPILOT_INVALID;
uint8_t mavMode = MAV_MODE_FLAG_SAFETY_ARMED;
uint8_t mavState = MAV_STATE_ACTIVE;

int transmitPosition(Generic_Port *port, Eigen::Vector3f pos) {
	mavlink_message_t msg;
	mavlink_msg_local_position_ned_pack(sysID, compID, &msg, 0, pos[0], pos[1], pos[2], 0, 0, 0);
  port->write_message(msg);
	return 0;
}

void SendHeartbeat(Generic_Port *port) {
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(sysID, compID, &msg, mavType, autopilot, mavMode, 0, mavState);
	port->write_message(msg);
}

int main(int argc, const char **argv)
{

  // Serial port setup
  const char *uartName = "/dev/ttyUSB0";
  int baudrate = 57600;

	// UDP Setup
	bool use_udp = true;
	const char *udp_ip = (char*)"127.0.0.1";
	const int udp_port = 14540;


  // Timer for metering data transmission
  chrono::time_point<chrono::system_clock> previousTime, currentTime, prevTime_hb, currTime_hb;
	// Time for logging
	chrono::time_point<chrono::system_clock> startTime;

	Eigen::Vector3f position = Eigen::Vector3f::Zero();

	Generic_Port *port;
  // Initialize serial
	if (use_udp) {
		port = new UDP_Port(udp_ip, udp_port);
	} else {
		port = new Serial_Port(uartName, baudrate);
	}

	mavlink_message_t garbage;

  port->start();
  if (!port->is_running()) {
    printf("\n");
    printf("ERROR INITIALIZING SERIAL");
    printf("\n");
    return 1;
  }

	port->read_message(garbage);

	// Connect to port

  // Start recording time for message streaming rate
  previousTime = chrono::system_clock::now(); 
	prevTime_hb = chrono::system_clock::now();
	startTime = chrono::system_clock::now();

  // main loop
  cout << "Main loop starting" << endl;
	while(1) {
		currentTime = chrono::system_clock::now();
		currTime_hb = chrono::system_clock::now();
		// Check if it's time to send a position update
		auto elapsedMilliseconds = chrono::duration_cast<chrono::milliseconds>(currentTime - previousTime);
		auto elapsedMilliseconds_hb = chrono::duration_cast<chrono::milliseconds>(currTime_hb - prevTime_hb);
		if (elapsedMilliseconds.count() >= 1000) {
			previousTime = currentTime;
			transmitPosition(port, position);
		}
		if (elapsedMilliseconds_hb.count() >= 1000) {
			prevTime_hb = currTime_hb;
			SendHeartbeat(port);
		}
  } // while

  return 0;
}
