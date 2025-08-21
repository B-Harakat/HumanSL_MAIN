#include "ViconInterface.h"
#include <iostream>
#include <thread>
#include <chrono>

using namespace ViconDataStreamSDK::CPP;

ViconInterface::ViconInterface() 
    : client_(std::make_unique<Client>()), connected_(false) {
}

ViconInterface::~ViconInterface() {
    if (connected_) {
        disconnect();
    }
}

bool ViconInterface::connect(const std::string& hostname) {
    hostname_ = hostname;
    
    std::cout << "Connecting to Vicon at " << hostname << "..." << std::endl;
    auto output = client_->Connect(hostname);
    
    if (output.Result != Result::Success) {
        std::cerr << "Failed to connect to Vicon system: " << output.Result << std::endl;
        return false;
    }
    
    connected_ = true;
    
    // Enable marker data and unlabeled marker data
    if (connected_) {
        client_->EnableMarkerData();
        client_->EnableUnlabeledMarkerData();
    }
    
    // Wait for first frame
    while (client_->GetFrame().Result != Result::Success) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    std::cout << "Connected to Vicon successfully!" << std::endl;
    return true;
}

void ViconInterface::disconnect() {
    if (connected_) {
        client_->Disconnect();
        connected_ = false;
        std::cout << "Disconnected from Vicon" << std::endl;
    }
}

bool ViconInterface::getFrame() {
    if (!connected_) return false;
    return client_->GetFrame().Result == Result::Success;
}

int ViconInterface::getFrameNumber() {
    if (!connected_) return false;
    return client_->GetFrameNumber().FrameNumber;
}

double ViconInterface::getFrameRate() const {
    if (!connected_) return 0.0;
    
    Output_GetFrameRate frameRateOutput = client_->GetFrameRate();
    if (frameRateOutput.Result == Result::Success) {
        return frameRateOutput.FrameRateHz;
    }
    
    return 0.0;
}



MarkerData ViconInterface::getMarkerPosition(const std::string& markerName) {
    MarkerData marker;
    marker.name = markerName;
    marker.occluded = true; // Default to occluded if not found
    
    if (!connected_) return marker;
    
    unsigned int subjectCount = client_->GetSubjectCount().SubjectCount;
    
    // Search through all subjects for the marker
    for (unsigned int i = 0; i < subjectCount; ++i) {
        std::string subjectName = client_->GetSubjectName(i).SubjectName;
        unsigned int markerCount = client_->GetMarkerCount(subjectName).MarkerCount;
        
        for (unsigned int j = 0; j < markerCount; ++j) {
            std::string currentMarkerName = client_->GetMarkerName(subjectName, j).MarkerName;
            // std::cout << currentMarkerName << "\n";
            if (currentMarkerName == markerName) {
                Output_GetMarkerGlobalTranslation translation = 
                    client_->GetMarkerGlobalTranslation(subjectName, markerName);
                if(translation.Result == Result::Success){
                    marker.x = translation.Translation[0];
                    marker.y = translation.Translation[1];
                    marker.z = translation.Translation[2];
                    marker.occluded = translation.Occluded;
                    return marker;
                }
            }
        }
    }
    
    return marker; // Return with occluded=true if marker not found
}

std::vector<MarkerData> ViconInterface::getMarkerPositions(const std::vector<std::string>& markerNames) {
    std::vector<MarkerData> markers;
    markers.reserve(markerNames.size());
    
    for (const std::string& markerName : markerNames) {
        markers.push_back(getMarkerPosition(markerName));
    }
    
    return markers;
}

std::vector<MarkerData> ViconInterface::getMarkerPositions(const std::string& prefix) {
    std::vector<MarkerData> markers;
    
    if (!connected_) return markers;
    
    unsigned int subjectCount = client_->GetSubjectCount().SubjectCount;
    
    // Search through all subjects for markers starting with the prefix
    for (unsigned int i = 0; i < subjectCount; ++i) {
        std::string subjectName = client_->GetSubjectName(i).SubjectName;
        unsigned int markerCount = client_->GetMarkerCount(subjectName).MarkerCount;
        
        for (unsigned int j = 0; j < markerCount; ++j) {
            std::string currentMarkerName = client_->GetMarkerName(subjectName, j).MarkerName;
            
            // Check if marker name starts with the given prefix
            if (currentMarkerName.substr(0, prefix.length()) == prefix) {
                Output_GetMarkerGlobalTranslation translation = 
                    client_->GetMarkerGlobalTranslation(subjectName, currentMarkerName);
                
                if (translation.Result == Result::Success) {
                    MarkerData marker;
                    marker.name = currentMarkerName;
                    marker.x = translation.Translation[0];
                    marker.y = translation.Translation[1];
                    marker.z = translation.Translation[2];
                    marker.occluded = translation.Occluded;
                    markers.push_back(marker);
                }
            }
        }
    }
    
    return markers;
}

std::vector<UnlabeledMarkerData> ViconInterface::getUnlabeledMarkers() {
    std::vector<UnlabeledMarkerData> unlabeledMarkers;
    
    if (!connected_) return unlabeledMarkers;
    
    unsigned int unlabeledMarkerCount = client_->GetUnlabeledMarkerCount().MarkerCount;
    
    for (unsigned int i = 0; i < unlabeledMarkerCount; ++i) {
        Output_GetUnlabeledMarkerGlobalTranslation translation = 
            client_->GetUnlabeledMarkerGlobalTranslation(i);
        
        if (translation.Result == Result::Success) {
            UnlabeledMarkerData marker;
            marker.x = translation.Translation[0];
            marker.y = translation.Translation[1];
            marker.z = translation.Translation[2];
            marker.id = i;
            
            unlabeledMarkers.push_back(marker);
        }
    }
    
    return unlabeledMarkers;
}


