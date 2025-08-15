#ifndef VICON_INTERFACE_H
#define VICON_INTERFACE_H

#include "DataStreamClient.h"
#include <string>
#include <vector>
#include <memory>
#include <chrono>



struct MarkerData {
    std::string name;
    double x, y, z;
    bool occluded;
};

struct UnlabeledMarkerData {
    double x, y, z;
    unsigned int id;
};

struct SegmentData {
    std::string name;
    std::vector<MarkerData> markers;
   
    // Require name and vector size when constructing the struct
    // For base frame and pipe related subjects, where size should be known
    SegmentData(const std::string& n, size_t marker_count) 
        : name(n), markers(marker_count) {}
    
    // Require name when constructing the struct
    // For huamn data points
    SegmentData(const std::string& n) 
        : name(n) {}
    
    // Delete default constructor to force name and/or size input
    SegmentData() = delete;
};

class ViconInterface {

private:
    std::unique_ptr<ViconDataStreamSDK::CPP::Client> client_;
    bool connected_;
    std::string hostname_;
    
public:
    ViconInterface();
    ~ViconInterface();
    
    // Connection management
    bool connect(const std::string& hostname = "localhost:801");
    void disconnect();
    bool isConnected() const { return connected_; }
    
    // Data acquisition
    bool getFrame();
    int getFrameNumber();

    MarkerData getMarkerPosition(const std::string& markerName);
    std::vector<MarkerData> getMarkerPositions(const std::vector<std::string>& markerNames);
    std::vector<MarkerData> getMarkerPositions(const std::string& prefix);
    std::vector<UnlabeledMarkerData> getUnlabeledMarkers();
    
    // Performance monitoring
    double getFrameRate() const;
    
};

#endif // VICON_INTERFACE_H