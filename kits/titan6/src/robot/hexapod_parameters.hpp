#pragma once

#include <string>

namespace hebi {

struct HexapodParameters
{
  // startup 
  float startup_second;
  float bias_record_second;
  
  // Stance
  float stance_radius_;
  float default_body_height_;
  float min_z_;
  float max_z_;
  float max_r_; // max radius (for stance mode body weight shifting)

  // Step threshold
  float step_threshold_rotate_; // rad
  float step_threshold_shift_; // m

  // dynamic controller gains
  float Kp_base;
  float Kp_shoulder;
  float Kp_elbow;
  float Kd_base;
  float Kd_shoulder;
  float Kd_elbow;
 

  // Logging
  bool logging_enabled_;
  float high_log_frequency_hz_;
  float low_log_frequency_hz_;

  // true on success, false on failure
  bool loadFromFile(std::string file);
  bool saveToFile(std::string file) const;
  void resetToDefaults();
};

} // namespace hebi
