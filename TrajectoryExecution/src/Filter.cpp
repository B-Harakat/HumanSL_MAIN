//----------------------------------------------------------
// Functions of the filter
//----------------------------------------------------------
// Description: functions used in filtering noise in accelerations
// Copyright: Yihan Liu 2024
//----------------------------------------------------------

#include "Filter.h"

// Parameters for the filter
const double b[] = {0.0001081, 0.0002161, 0.0001081}; // Butterworth filter numerator coefficients
const double a[] = {1.0, -1.9704, 0.9708};           // Butterworth filter denominator coefficients

// Parameters for velocity filter (higher cutoff frequency, less aggressive)
const double b_vel[] = {0.0976, 0.1953, 0.0976}; // Velocity filter numerator coefficients 
const double a_vel[] = {1.0, -0.6131, 0.2066};   // Velocity filter denominator coefficients

// Initialize filter state matrices for each dq element
Matrix<double, 6, 2> prev_dq = Matrix<double, 6, 2>::Zero();      // Previous dq inputs
Matrix<double, 6, 2> prev_output = Matrix<double, 6, 2>::Zero();  // Previous dq outputs

Matrix<double, 6, 2> prev_pose_error = Matrix<double, 6, 2>::Zero();
Matrix<double, 6, 2> prev_pose_output = Matrix<double, 6, 2>::Zero();

Matrix<double, 6, 2> prev_velocity = Matrix<double, 6, 2>::Zero();
Matrix<double, 6, 2> prev_velocity_output = Matrix<double, 6, 2>::Zero();


//----------------------------------------------------------
// Function to initialize the butterworth filter
//----------------------------------------------------------
void Filter::ini_butterworth(){
    prev_dq = Matrix<double, 6, 2>::Zero();
    prev_output = Matrix<double, 6, 2>::Zero();
}

void Filter::ini_butterworth_pose(){
    prev_pose_error = Matrix<double, 6, 2>::Zero();
    prev_pose_output = Matrix<double, 6, 2>::Zero();
}

void Filter::ini_butterworth_velocity(){
    prev_velocity = Matrix<double, 6, 2>::Zero();
    prev_velocity_output = Matrix<double, 6, 2>::Zero();
}



//----------------------------------------------------------
// Function of applying butterworth filter
//----------------------------------------------------------
// 1 input:
// current_ddp: noised acceleration
//
// 1 output:
// filtered_ddp: filtered accelerations
//----------------------------------------------------------
VectorXd Filter::butterworth_filter(const VectorXd& current_ddp) {
     VectorXd filtered_ddp(6); // Vector to store the filtered dq values

    // Apply the filter to each element in the dq vector
    for (int i = 0; i < 6; ++i) {
        // Compute the filtered output using the difference equation
        double output = b[0] * current_ddp[i]
                        + b[1] * prev_dq(i, 0) + b[2] * prev_dq(i, 1)
                        - a[1] * prev_output(i, 0) - a[2] * prev_output(i, 1);

        // Update the filter states for the next iteration
        prev_dq(i, 1) = prev_dq(i, 0);
        prev_dq(i, 0) = current_ddp[i];

        prev_output(i, 1) = prev_output(i, 0);
        prev_output(i, 0) = output;

        // Store the result
        if (i <= 2) {
            filtered_ddp[i] = output - fmod(output, 0.02);
        }
        else{
            filtered_ddp[i] = output - fmod(output, 0.04);
        }
    }

    return filtered_ddp;
}


VectorXd Filter::butterworth_filter_pose(const VectorXd& current_pose_error) {
    VectorXd filtered_pose_error(6);
    
    for (int i = 0; i < 6; ++i) {
        // Same filter coefficients as acceleration filter
        double output = b[0] * current_pose_error[i]
                        + b[1] * prev_pose_error(i, 0) + b[2] * prev_pose_error(i, 1)
                        - a[1] * prev_pose_output(i, 0) - a[2] * prev_pose_output(i, 1);

        // Update filter states
        prev_pose_error(i, 1) = prev_pose_error(i, 0);
        prev_pose_error(i, 0) = current_pose_error[i];
        
        prev_pose_output(i, 1) = prev_pose_output(i, 0);
        prev_pose_output(i, 0) = output;
        
        filtered_pose_error[i] = output;
    }
    
    return filtered_pose_error;
}

VectorXd Filter::butterworth_filter_velocity(const VectorXd& current_velocity) {
    VectorXd filtered_velocity(6);
    
    for (int i = 0; i < 6; ++i) {
        // Use velocity-specific filter coefficients
        double output = b_vel[0] * current_velocity[i]
                        + b_vel[1] * prev_velocity(i, 0) + b_vel[2] * prev_velocity(i, 1)
                        - a_vel[1] * prev_velocity_output(i, 0) - a_vel[2] * prev_velocity_output(i, 1);

        // Update filter states
        prev_velocity(i, 1) = prev_velocity(i, 0);
        prev_velocity(i, 0) = current_velocity[i];
        
        prev_velocity_output(i, 1) = prev_velocity_output(i, 0);
        prev_velocity_output(i, 0) = output;
        
        filtered_velocity[i] = output;
    }
    
    return filtered_velocity;
}