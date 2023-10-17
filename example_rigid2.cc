// example_rigi2.cc -*- C++ -*-
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
#include <vector>

#include "owl.hpp"

using namespace std;

////

inline void printInfo(const OWL::Markers &markers)
{
  for(OWL::Markers::const_iterator m = markers.begin(); m != markers.end(); m++)
    if(m->cond > 0)
      cout << "   " << m->id << ") pos=" << m->x << "," << m->y << "," << m->z << endl;
}

inline void printInfo(const OWL::Rigids &rigids)
{
  for(OWL::Rigids::const_iterator r = rigids.begin(); r != rigids.end(); r++)
    if(r->cond > 0)
      cout << "   " << r->id << ") pose=" << r->pose[0] << "," << r->pose[1] << "," << r->pose[2]
           << " " << r->pose[3] << "," << r->pose[4] << "," << r->pose[5] << "," << r->pose[6]
           << endl;
}

////

int main(int argc, const char **argv)
{
  string address = argc > 1 ? argv[1] : "localhost";
  OWL::Context owl;
  OWL::Markers markers;
  OWL::Rigids rigids;

  if(owl.open(address) <= 0 || owl.initialize() <= 0) return 0;

  // create the rigid tracker
  uint32_t tracker_id = 0;
  vector<OWL::TrackerInfo> tracker_info;
  tracker_info.push_back(OWL::TrackerInfo(tracker_id, "rigid", "myrigid", ""));
  owl.createTrackers(tracker_info.data(), tracker_info.data()+tracker_info.size());

  // assign markers to the rigid and specify local coordinates in millimeters (taken from wand.json)
  vector<OWL::MarkerInfo> marker_info;
  marker_info.push_back(OWL::MarkerInfo(4, tracker_id, "4", "pos=0,920,-7.615"));
  marker_info.push_back(OWL::MarkerInfo(5, tracker_id, "5", "pos=-7.615,795,0"));
  marker_info.push_back(OWL::MarkerInfo(6, tracker_id, "6", "pos=0,670,-7.615"));
  marker_info.push_back(OWL::MarkerInfo(7, tracker_id, "7", "pos=-7.615,545,0"));
  owl.assignMarkers(marker_info.data(), marker_info.data()+marker_info.size());

  // start streaming
  owl.streaming(true);

  // main loop
  while(owl.isOpen() && owl.property<int>("initialized"))
    {
      const OWL::Event *event = owl.nextEvent(1000);
      if(!event) continue;

      switch(event->type_id())
        {
        case OWL::Type::ERROR:
          {
            cerr << event->name() << ": " << event->str() << endl;
          }
          break;
        case OWL::Type::FRAME:
          {
            cout << "time=" << event->time() << " " << event->type_name() << " " << event->name() << "=" << event->size<OWL::Event>() << ":" << endl;
            for(const OWL::Event *e = event->begin(); e != event->end(); e++)
              {
                switch(e->type_id())
                  {
                  case OWL::Type::MARKER:
                    {
                      if(e->get(markers) > 0)
                        {
                          cout << " " << e->type_name() << " " << e->name() << "=" << markers.size() << ":" << endl;
                          printInfo(markers);
                        }
                    }
                    break;
                  case OWL::Type::RIGID:
                    {
                      if(e->get(rigids) > 0)
                        {
                          cout << " " << e->type_name() << " " << e->name() << "=" << rigids.size() << ":" << endl;
                          printInfo(rigids);
                        }
                    }
                    break;
                  } // switch
              }
          }
          break;
        } // switch
    } // while

  owl.done();
  owl.close();

  return 0;
}
