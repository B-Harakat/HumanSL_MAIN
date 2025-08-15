#pragma once

#define _USE_MATH_DEFINES

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>
#include <thread>
#include <iomanip>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <tuple>

#include <Eigen/Dense>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h> 
#include <gtsam/base/Vector.h>

#include <shared_mutex>

#include "ViconInterface.h"
#include "utils.h"

void triggerRightGrasp(HumanInfo& human_info, TubeInfo& tube_info, std::atomic<int>& phase_idx);
