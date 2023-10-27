/*
* Program for logging the Phasespace motion capture data.
* Usage:
* 	From the command line, enter ./logDataTest [ip] >> logfile.txt.
* 	ip is the ip address of the Phasespace server.
*/

// TODO: add quit handler
// TODO: Change output to csv
// TODO: Define a rigid body

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

using std::string;
using namespace std;

int transmitPosition(Serial_Port *port, float x, float y, float z) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint8_t sysID = 50;
	uint8_t compID = 0;
	mavlink_msg_local_position_ned_pack(sysID, compID, &msg, 0, x, y, z, 0, 0, 0);
  port->write_message(msg);
	return 0;
}

//int recordToLogfile(OWL::Markers *markers, const OWL::Event *event) {
//  ofstream logfile;
//  logfile.open("output.csv");
//  logfile << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
//  if(event->find("markers", *markers) > 0) {
//    logfile << " markers=" << markers->size() << ":" << endl;
//    for(OWL::Markers::iterator m = markers->begin(); m != markers->end(); m++) {
//      if(m->cond > 0) {
//        logfile << "  " << m->id << ") " << m->x << "," << m->y << "," << m->z << endl;
//      }
//    }
//  }
//}

//int recordToLogfile(const OWL::Rigids *rigids, const OWL::Event *event) {
//  ofstream logfile;
//  logfile.open("output.csv");
//  logfile << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
//  if(event->find("rigids", *rigids) > 0) {
//    logfile << " rigids=" << rigids->size() << ":" << endl;
//    for(OWL::Rigids::iterator r = rigids->begin(); r != rigids->end(); r++) {
//      if(r->cond > 0) {
//				logfile << "  " << r->id << ") " << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
//						 << "," << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
//						 << endl;
//      }
//    }
//  }
//}

int main(int argc, const char **argv)
{
  string address = argc > 1 ? argv[1] : "localhost";
  OWL::Context owl;
  OWL::Markers markers;
	OWL::Rigids rigids;

  // Serial port setup
  const char *uartName = "/dev/ttyUSB0";
  int baudrate = 57600;

  // Timer for metering data transmission
  chrono::time_point<chrono::system_clock> previousTime, currentTime;

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;
	

	// Define a rigid body tracker
	uint32_t trackerID = 1;
	owl.createTracker(trackerID, "rigid", "quadcopter");

	// Assign markers to rigid body (location in mm)
	owl.assignMarker(trackerID, 0, "0", "pos = 0,0");
	owl.assignMarker(trackerID, 1, "1", "pos = 0,0");
	owl.assignMarker(trackerID, 2, "2", "pos = 0,0");
	owl.assignMarker(trackerID, 3, "3", "pos = 0,0");
	owl.assignMarker(trackerID, 4, "4", "pos = 0,0");
	owl.assignMarker(trackerID, 5, "5", "pos = 0,0");
	owl.assignMarker(trackerID, 6, "6", "pos = 0,0");
	owl.assignMarker(trackerID, 7, "7", "pos = 0,0");

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

  // main loop
  while(owl.isOpen() && owl.property<int>("initialized")) {
		const OWL::Event *event = owl.nextEvent(1000);
		if(!event) continue;

		if(event->type_id() == OWL::Type::ERROR) {
				cerr << event->name() << ": " << event->str() << endl;
		}

		else if(event->type_id() == OWL::Type::FRAME) {
			int64_t	frameTime = event->time();

			// Check if there is rigid body data
			if (event->find("rigids", rigids) > 0 && rigids[0].cond > 0) {
				logfile << "  " << rigids[0].id << ") " << rigids[0].pose[0] << "," << rigids[0].pose[1] << "," << rigids[0].pose[2]
						 << "," << rigids[0].pose[3] << "," << rigids[0].pose[4] << "," << rigids[0].pose[5] << "," << rigids[0].pose[6]
						 << endl;

				// Check if it's time to send a position update
				currentTime = chrono::system_clock::now();
				auto elapsedMilliseconds = chrono::duration_cast<chrono::milliseconds>(currentTime - previousTime);


				if (elapsedMilliseconds.count() >= 1000) {
					previousTime = currentTime;
					//transmitPosition(&port, rigids[0].pose[0], rigids[0].pose[1], rigids[0].pose[2]);
				}
			}

		}

  } // while

	logfile.close();
  owl.done();
  owl.close();

  return 0;
}
