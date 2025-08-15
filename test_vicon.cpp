#include "ViconInterface.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace std::chrono_literals;

int main() {
    // ViconInterface vicon;
    
    // if (!vicon.connect("192.168.128.211")) {
    //     std::cerr << "Failed to connect to Vicon system. Exiting." << std::endl;
    //     return -1;
    // }
    
    // std::cout << "Connected. Displaying framerate and unlabeled markers. Press Ctrl+C to exit." << std::endl;
    
    // auto now = std::chrono::system_clock::now();
    
    // while (true) {
    //     if (std::chrono::duration_cast<std::chrono::milliseconds>(
    //         std::chrono::system_clock::now() - now) > 1ms) {
            
    //         now = std::chrono::system_clock::now();
            
    //         if (!vicon.getFrame()) {
    //             std::this_thread::sleep_for(std::chrono::milliseconds(1));
    //             continue;
    //         }
            
    //         // Print framerate
    //         double framerate = vicon.getFrameRate();
    //         std::cout << "Framerate: " << std::fixed << std::setprecision(1) << framerate << " Hz | ";
            
    //         // Print unlabeled markers
    //         auto unlabeledMarkers = vicon.getUnlabeledMarkers();
    //         std::cout << "Unlabeled: " << unlabeledMarkers.size() << " markers";
            
    //         if (!unlabeledMarkers.empty()) {
    //             std::cout << " [";
    //             for (size_t i = 0; i < unlabeledMarkers.size(); ++i) {
    //                 const auto& marker = unlabeledMarkers[i];
    //                 std::cout << "(" << std::fixed << std::setprecision(0) 
    //                          << marker.x << "," << marker.y << "," << marker.z << ")";
    //                 if (i < unlabeledMarkers.size() - 1) std::cout << " ";
    //             }
    //             std::cout << "]";
    //         }
            
    //         std::cout << std::endl;
            
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    // }
    
    return 0;
}