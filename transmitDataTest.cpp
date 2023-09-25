/*
* Program for logging the Phasespace motion capture data.
* Usage:
* 	From the command line, enter ./logDataTest [ip] >> logfile.txt.
* 	ip is the ip address of the Phasespace server.
*/

#include <iostream>
#include <cstring>
#include <mavlink/common/mavlink.h>

#include "owl.hpp"

using namespace std;

int transmitPosition(float x, float y, float z);

int main(int argc, const char **argv)
{
  string address = argc > 1 ? argv[1] : "localhost";
  OWL::Context owl;
  OWL::Markers markers;

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;

  // start streaming
  owl.streaming(1);

  // main loop
  while(owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      if(event->type_id() == OWL::Type::ERROR)
        {
          cerr << event->name() << ": " << event->str() << endl;
        }
      else if(event->type_id() == OWL::Type::FRAME)
        {
          cout << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
          if(event->find("markers", markers) > 0)
            {
              cout << " markers=" << markers.size() << ":" << endl;
              for(OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++)
                if(m->cond > 0)
                  cout << "  " << m->id << ") " << m->x << "," << m->y << "," << m->z << endl;
            }
        }
    } // while

  owl.done();
  owl.close();

  return 0;
}

int transmitPosition(float x, float y, float z) {
	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint8_t sysID = 50;
	uint8_t compID = 0;
	mavlink_msg_local_position_ned_pack(sysID, compID, &msg, 0, x, y, z, 0, 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
	return 0;
}

