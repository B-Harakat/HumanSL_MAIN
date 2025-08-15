#include "ViconInterface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>


using namespace std::chrono_literals;


int main() {
    ViconInterface vicon;
    
    if (!vicon.connect("192.168.128.206")) {
        std::cerr << "Failed to connect to Vicon system. Exiting." << std::endl;
        return -1;
    }
    std::cout<<"Vicon Connected.\n";

    int prev_frame_number = -1;

    std::vector<UnlabeledMarkerData> dummy_data;
    MarkerData labeled_data;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    

    while(true){

        int cur_frame_number = vicon.getFrameNumber(); // get current frame number

        if (cur_frame_number == prev_frame_number){
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // if no new frame, sleep a bit, saves CPU effort
            continue;
        }
        else{
            vicon.getFrame(); // update frame data
            prev_frame_number = cur_frame_number;
        }

        dummy_data = vicon.getUnlabeledMarkers(); // get all unlabelled marker data

        // for(auto dummy : dummy_data){
        // std::cout << dummy.x << " " << dummy.y << " " << dummy.z;
        // }

        labeled_data = vicon.getMarkerPosition("right1"); // Access marker position by marker name

        std::cout << labeled_data.x << " " << labeled_data.y << " " << labeled_data.z;

        auto tube_data = vicon.getMarkerPositions("tube");

        std::cout << "Tube data: \n";
        for(auto dummy : tube_data){
        std::cout << dummy.x << " " << dummy.y << " " << dummy.z << "\n";
        }
    }
    
    return 0;
}