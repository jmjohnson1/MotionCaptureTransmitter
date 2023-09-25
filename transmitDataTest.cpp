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

int32_t ConvertPosition(float position[3]);

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

int32_t ConvertPosition(float position[3]) {
	/* Converts position from local coordinates to LLH
	* Inputs
	*  position   - float -  Array of x, y, z position in local coordinate frame
	* Output
	* 	position_llh  - int32_t -  Array of lat., lon., altitude
	*/

	int32_t originLocation_lat = 0; // degE7
 	int32_t originLocation_lon = 0;  // degE7
	int32_t originLocation_alt = 0;  // mm
	

	return 0;

}
