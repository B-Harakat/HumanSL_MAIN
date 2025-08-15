/**
 *  @file  JerkPenaltyFactor.h
 *  @brief Factor to penalize jerk (acceleration changes) in trajectory optimization
 *  @author Custom Implementation
 *  @date  2025
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <iostream>



/**
 * 3-way factor to penalize jerk (acceleration changes) between consecutive waypoints
 * Connects three consecutive position waypoints and computes jerk penalty
 */
class JerkPenaltyFactor : public gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector> {
 private:
  // typedefs
  typedef JerkPenaltyFactor This;
  typedef gtsam::NoiseModelFactorN<gtsam::Vector, gtsam::Vector, gtsam::Vector> Base;

  double delta_t_;  // time step between consecutive waypoints
  size_t dof_;      // degrees of freedom (number of joints)

 public:
  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<This> shared_ptr;

  /// Default constructor for serialization
  JerkPenaltyFactor() {}

  /**
   * Constructor
   * @param poseKey1 first pose key (time i-1)
   * @param poseKey2 second pose key (time i)
   * @param poseKey3 third pose key (time i+1)
   * @param cost_model jerk penalty weight
   * @param delta_t time step between consecutive waypoints
   */
  JerkPenaltyFactor(gtsam::Key poseKey1, gtsam::Key poseKey2, gtsam::Key poseKey3,
                    const gtsam::SharedNoiseModel& cost_model,
                    double delta_t)
      : Base(cost_model, poseKey1, poseKey2, poseKey3),
        delta_t_(delta_t),
        dof_(cost_model->dim()) {
    if (delta_t <= 0.0)
      throw std::runtime_error("[JerkPenaltyFactor] ERROR: delta_t must be > 0.");
  }

  virtual ~JerkPenaltyFactor() {}

  /// error function
  gtsam::Vector evaluateError(
      const gtsam::Vector& pose1, const gtsam::Vector& pose2, const gtsam::Vector& pose3,
      gtsam::OptionalMatrixType H1 = nullptr,
      gtsam::OptionalMatrixType H2 = nullptr,
      gtsam::OptionalMatrixType H3 = nullptr) const override {
    
    using namespace gtsam;
    
    // Compute finite difference approximations
    // velocity at time i-1/2: v_{i-1/2} = (pose2 - pose1) / delta_t
    // velocity at time i+1/2: v_{i+1/2} = (pose3 - pose2) / delta_t
    // acceleration at time i: a_i = (v_{i+1/2} - v_{i-1/2}) / delta_t
    //                              = (pose3 - 2*pose2 + pose1) / delta_t^2
    
    Vector acceleration = (pose3 - 2.0 * pose2 + pose1) / (delta_t_ * delta_t_);
    
    // For jerk penalty, we want to minimize the magnitude of acceleration changes
    // This is the second derivative of position (acceleration)
    // The error is simply the acceleration vector
    Vector error = acceleration;
    
    // Compute Jacobians if requested
    if (H1) {
      *H1 = Matrix::Identity(dof_, dof_) / (delta_t_ * delta_t_);
    }
    if (H2) {
      *H2 = -2.0 * Matrix::Identity(dof_, dof_) / (delta_t_ * delta_t_);
    }
    if (H3) {
      *H3 = Matrix::Identity(dof_, dof_) / (delta_t_ * delta_t_);
    }
    
    return error;
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /** print contents */
  void print(const std::string& s = "",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const {
    std::cout << s << "JerkPenaltyFactor :" << std::endl;
    Base::print("", keyFormatter);
    std::cout << "delta_t : " << delta_t_ << std::endl;
    std::cout << "dof : " << dof_ << std::endl;
  }

  /// equals specialized to this factor
  bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override {
    const This* e = dynamic_cast<const This*>(&expected);
    return e != nullptr && Base::equals(*e, tol) && 
           std::abs(this->delta_t_ - e->delta_t_) < tol;
  }

  /// access
  double delta_t() const { return delta_t_; }
  size_t dof() const { return dof_; }

 private:
#ifdef GPMP2_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int version) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor3", boost::serialization::base_object<Base>(*this));
    ar& BOOST_SERIALIZATION_NVP(delta_t_);
    ar& BOOST_SERIALIZATION_NVP(dof_);
  }
#endif
};

