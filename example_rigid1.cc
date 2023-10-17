// example_rigid1.cc -*- C++ -*-
// simple rigid tracking program

/***
Copyright (c) PhaseSpace, Inc 2017

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL PHASESPACE, INC
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
***/

#include <iostream>
#include <cstring>

#include "owl.hpp"

using namespace std;

int main(int argc, const char **argv)
{
  string address = argc > 1 ? argv[1] : "localhost";
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Rigids rigids;

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;

  // create the rigid tracker
  uint32_t tracker_id = 0;
  owl.createTracker(tracker_id, "rigid", "myrigid");

  // assign markers to the rigid and specify local coordinates in millimeters (taken from wand.json)
  owl.assignMarker(tracker_id, 4, "4", "pos=0,920,-7.615");
  owl.assignMarker(tracker_id, 5, "5", "pos=-7.615,795,0");
  owl.assignMarker(tracker_id, 6, "6", "pos=0,670,-7.615");
  owl.assignMarker(tracker_id, 7, "7", "pos=-7.615,545,0");

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
          if(event->find("rigids", rigids) > 0)
            {
              cout << " rigid=" << rigids.size() << ":" << endl;
              for(OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++)
                if(r->cond > 0)
                  cout << "  " << r->id << ") " << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
                       << "," << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
                       << endl;
            }
        }
    } // while

  owl.done();
  owl.close();

  return 0;
}
