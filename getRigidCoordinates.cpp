#include <iostream>
#include <cstring>
#include <cmath>

#include "owl.hpp"
using namespace std;

int main(int argc, const char **argv) {
    std::string address = argc > 1 ? argv[1] : "160.94.220.134";
    OWL::Context owl;
    OWL::Markers markers;
    
    if (owl.open(address) <= 0 || owl.initialize() <= 0){
        return 0;
    }

    owl.streaming(1);

    int bodyOriginMarkerID = 2;
    int axisMarkerID = 3;

    int numberSamples = 10000;
    int numberMarkers = 8;
    float markerPositions[numberMarkers][numberSamples][3];
    // Initalize all positions to NAN
    for (int i = 0; i < numberMarkers; i++) {
        for (int j = 0; j < numberSamples; j++) {
            for (int k = 0; k < 3; k++) {
                markerPositions[i][j][k] = NAN;
            }
        }
    }

    int numberLogged = 0;

    // Main loop
    while (owl.isOpen() && owl.property<int>("initialized") && numberLogged < numberSamples) {
        const OWL::Event *event = owl.nextEvent(1000);
        if (!event) {
            continue;
        }
        if (event->type_id() == OWL::Type::ERROR) {
            std::cerr << event->name() << ": " << event->str() << std::endl;
        } else if (event->type_id() == OWL::Type::FRAME) {
            if (event->find("markers", markers) > 0) {
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++) {
                    if (m->cond > 0) {
                        markerPositions[m->id][numberLogged][0] = m->x;
                        markerPositions[m->id][numberLogged][1] = m->y;
                        markerPositions[m->id][numberLogged][2] = m->z;
                    }
                }
                numberLogged += 1;
            } 
        }
    }

    // Take the average position of available markers
    float markerPositionsAvg[numberMarkers][3];
    // Initialize all positions to NAN
    for (int i = 0; i < numberMarkers; i++) {
        for (int j = 0; j < 3; j++) {
            markerPositionsAvg[i][j] = NAN;
        }
    }

    for (int i = 0; i < numberMarkers; i++) {
        int numberCollectedThisMarker = 0;
        float x_bar = 0;
        float y_bar = 0;
        float z_bar = 0;
        for (int j = 0; j < numberSamples; j++) {
            // Skip this if NAN row
            if (isnan(markerPositions[i][j][0])) continue;

            x_bar = x_bar + markerPositions[i][j][0];
            y_bar = y_bar + markerPositions[i][j][1];
            z_bar = z_bar + markerPositions[i][j][2];
            numberCollectedThisMarker += 1;
        }
        markerPositionsAvg[i][0] = x_bar/numberCollectedThisMarker;
        markerPositionsAvg[i][1] = y_bar/numberCollectedThisMarker;
        markerPositionsAvg[i][2] = z_bar/numberCollectedThisMarker;
    }

    // Check if the body origin is nan
    if (isnan(markerPositionsAvg[bodyOriginMarkerID][0])) {
        std::cerr << "Marker ID selected for body origin does not exist." << std::endl;
    }
    if (isnan(markerPositionsAvg[axisMarkerID][0])) {
        std::cerr << "Marker ID selected for body axis does not exist." << std::endl;
    }

    // Move the origin to the body origin marker. Call this frame a.
    float dx = markerPositionsAvg[bodyOriginMarkerID][0];
    float dy = markerPositionsAvg[bodyOriginMarkerID][1];
    float dz = markerPositionsAvg[bodyOriginMarkerID][2];
    float shift[3] = {-dx, -dy, -dz};

    for (int i = 0; i < numberMarkers; i++) {
        for (int j = 0; j < 3; j++) {
            markerPositionsAvg[i][j] = markerPositionsAvg[i][j] + shift[j];
        }
    }

    // Determine the angle between a1 and b1. b1 is aligned between the origin and the specified axisMarker
    float psi = atan(markerPositionsAvg[axisMarkerID][2]/markerPositionsAvg[axisMarkerID][0]);
    
    // Transform from the a frame to the b frame
    for (int i = 0; i < numberMarkers; i++) {
        float rb1 = markerPositionsAvg[i][0]*cos(psi) + markerPositionsAvg[i][2]*sin(psi);
        float rb2 = -markerPositionsAvg[i][0]*sin(psi) + markerPositionsAvg[i][2]*cos(psi);
        float rb3 = -markerPositionsAvg[i][1];
        markerPositionsAvg[i][0] = rb1;
        markerPositionsAvg[i][1] = rb2;
        markerPositionsAvg[i][2] = rb3;
    }

    // Output the body frame coordinates
    std::cout << "Body frame marker coordinates: " << std::endl;
    for (int i = 0; i < numberMarkers; i++) {
        std::cout << "owl.assignMarker(trackerID, " << i << ", \"" << i << "\", \"pos=" << markerPositionsAvg[i][0] << "," << markerPositionsAvg[i][1] << "," << markerPositionsAvg[i][2] << "\");" << std::endl;
    }

    owl.done();
    owl.close();
}
