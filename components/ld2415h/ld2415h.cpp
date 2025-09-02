#include "ld2415h.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace ld2415h {

static const char *const TAG = "ld2415h";

static const uint8_t LD2415H_CMD_SET_SPEED_ANGLE_SENSE[] = {0x43, 0x46, 0x01, 0x01, 0x00, 0x05, 0x0d, 0x0a};
static const uint8_t LD2415H_CMD_SET_MODE_RATE_UOM[] = {0x43, 0x46, 0x02, 0x01, 0x01, 0x00, 0x0d, 0x0a};
static const uint8_t LD2415H_CMD_SET_ANTI_VIB_COMP[] = {0x43, 0x46, 0x03, 0x05, 0x00, 0x00, 0x0d, 0x0a};
static const uint8_t LD2415H_CMD_SET_RELAY_DURATION_SPEED[] = {0x43, 0x46, 0x04, 0x03, 0x01, 0x00, 0x0d, 0x0a};
static const uint8_t LD2415H_CMD_GET_CONFIG[] = {0x43, 0x46, 0x07, 0x00, 0x00, 0x00, 0x00,
                                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

LD2415HComponent::LD2415HComponent()
    : cmd_speed_angle_sense_{0x43, 0x46, 0x01, 0x01, 0x00, 0x05, 0x0d, 0x0a},
      cmd_mode_rate_uom_{0x43, 0x46, 0x02, 0x01, 0x01, 0x00, 0x0d, 0x0a},
      cmd_anti_vib_comp_{0x43, 0x46, 0x03, 0x05, 0x00, 0x00, 0x0d, 0x0a},
      cmd_relay_duration_speed_{0x43, 0x46, 0x04, 0x03, 0x01, 0x00, 0x0d, 0x0a},
      cmd_config_{0x43, 0x46, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} {}

void LD2415HComponent::setup() {
  // This triggers current sensor configurations to be dumped
  this->update_config_ = true;
  
  // Log which sensors are configured
  ESP_LOGI(TAG, "LD2415H Sensor Configuration:");
  ESP_LOGI(TAG, "  Speed sensor: %s", this->speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Approaching speed sensor: %s", this->approaching_speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Departing speed sensor: %s", this->departing_speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Velocity sensor: %s", this->velocity_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Approaching last max speed sensor: %s", this->approaching_last_max_speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Departing last max speed sensor: %s", this->departing_last_max_speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Approaching vehicle count sensor: %s", this->approaching_vehicle_count_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Departing vehicle count sensor: %s", this->departing_vehicle_count_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Current approaching vehicle speed sensor: %s", this->current_approaching_vehicle_speed_sensor_ ? "configured" : "NOT configured");
  ESP_LOGI(TAG, "  Current departing vehicle speed sensor: %s", this->current_departing_vehicle_speed_sensor_ ? "configured" : "NOT configured");
  
  ESP_LOGI(TAG, "LD2415H Component initialized");
  ESP_LOGI(TAG, "Ready for sensor data");
  
  // Log sensor entity details for debugging Home Assistant connectivity
  if (this->speed_sensor_) {
    ESP_LOGI(TAG, "Speed sensor details: object_id='%s', name='%s', internal=%s", 
             this->speed_sensor_->get_object_id().c_str(), 
             this->speed_sensor_->get_name().c_str(),
             this->speed_sensor_->is_internal() ? "true" : "false");
  }
  if (this->approaching_speed_sensor_) {
    ESP_LOGI(TAG, "Approaching speed sensor details: object_id='%s', name='%s', internal=%s", 
             this->approaching_speed_sensor_->get_object_id().c_str(), 
             this->approaching_speed_sensor_->get_name().c_str(),
             this->approaching_speed_sensor_->is_internal() ? "true" : "false");
  }
  if (this->velocity_sensor_) {
    ESP_LOGI(TAG, "Velocity sensor details: object_id='%s', name='%s', internal=%s", 
             this->velocity_sensor_->get_object_id().c_str(), 
             this->velocity_sensor_->get_name().c_str(),
             this->velocity_sensor_->is_internal() ? "true" : "false");
  }

#ifdef USE_NUMBER
  this->min_speed_threshold_number_->publish_state(this->min_speed_threshold_);
  this->compensation_angle_number_->publish_state(this->compensation_angle_);
  this->sensitivity_number_->publish_state(this->sensitivity_);
  this->vibration_correction_number_->publish_state(this->vibration_correction_);
  this->relay_trigger_duration_number_->publish_state(this->relay_trigger_duration_);
  this->relay_trigger_speed_number_->publish_state(this->relay_trigger_speed_);
  this->timeout_duration_number_->publish_state(this->timeout_duration_);
#endif
#ifdef USE_SELECT
  this->sample_rate_selector_->publish_state(this->i_to_s_(SAMPLE_RATE_STR_TO_INT, this->sample_rate_));
  this->tracking_mode_selector_->publish_state(this->i_to_s_(TRACKING_MODE_STR_TO_INT, this->tracking_mode_));
#endif
}

void LD2415HComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LD2415H:");
  ESP_LOGCONFIG(TAG, "  Firmware: %s", this->firmware_);
  ESP_LOGCONFIG(TAG, "  Minimum Speed Threshold: %u KPH", this->min_speed_threshold_);
  ESP_LOGCONFIG(TAG, "  Compensation Angle: %u", this->compensation_angle_);
  ESP_LOGCONFIG(TAG, "  Sensitivity: %u", this->sensitivity_);
  ESP_LOGCONFIG(TAG, "  Tracking Mode: %s", tracking_mode_to_s_(this->tracking_mode_));
  ESP_LOGCONFIG(TAG, "  Sampling Rate: %u", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Unit of Measure: %s", unit_of_measure_to_s_(this->unit_of_measure_));
  ESP_LOGCONFIG(TAG, "  Vibration Correction: %u", this->vibration_correction_);
  ESP_LOGCONFIG(TAG, "  Relay Trigger Duration: %u", this->relay_trigger_duration_);
  ESP_LOGCONFIG(TAG, "  Relay Trigger Speed: %u KPH", this->relay_trigger_speed_);
  ESP_LOGCONFIG(TAG, "  Timeout Duration: %u ms", this->timeout_duration_);
  ESP_LOGCONFIG(TAG, "  Negotiation Mode: %s", negotiation_mode_to_s_(this->negotiation_mode_));
}

void LD2415HComponent::loop() {
  // Simple UART processing (based on dermodmaster/cptskippy approach)
  while (this->available()) {
    if (this->fill_buffer_(this->read())) {
      this->parse_buffer_();
    }
  }

  // Simple vehicle tracking with timeouts (enhanced from dermodmaster)
  uint32_t now = millis();
  
  // Approaching vehicle timeout - publish max speed and count vehicle
  if (this->approaching_last_max_speed_sensor_ != nullptr &&
      this->last_max_approaching_speed_ > 0 && 
      now - this->last_approaching_update_time_ > this->timeout_duration_) {
    
    this->approaching_last_max_speed_sensor_->publish_state(this->last_max_approaching_speed_);
    this->last_approaching_update_time_ = now;
    this->last_max_approaching_speed_ = 0;
    
    // Count completed vehicle
    this->approaching_vehicle_count_++;
    if (this->approaching_vehicle_count_sensor_ != nullptr) {
      this->approaching_vehicle_count_sensor_->publish_state(this->approaching_vehicle_count_);
    }
    ESP_LOGD(TAG, "Approaching vehicle completed, count: %d", this->approaching_vehicle_count_);
  }

  // Departing vehicle timeout - publish max speed and count vehicle  
  if (this->departing_last_max_speed_sensor_ != nullptr &&
      this->last_max_departing_speed_ > 0 && 
      now - this->last_departing_update_time_ > this->timeout_duration_) {
    
    this->departing_last_max_speed_sensor_->publish_state(this->last_max_departing_speed_);
    this->last_departing_update_time_ = now;
    this->last_max_departing_speed_ = 0;
    
    // Count completed vehicle
    this->departing_vehicle_count_++;
    if (this->departing_vehicle_count_sensor_ != nullptr) {
      this->departing_vehicle_count_sensor_->publish_state(this->departing_vehicle_count_);
    }
    ESP_LOGD(TAG, "Departing vehicle completed, count: %d", this->departing_vehicle_count_);
  }

  // Configuration commands (from dermodmaster)
  if (this->update_speed_angle_sense_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_SPEED_ANGLE_SENSE");
    this->cmd_speed_angle_sense_[3] = this->min_speed_threshold_;
    this->cmd_speed_angle_sense_[4] = this->compensation_angle_;
    this->cmd_speed_angle_sense_[5] = this->sensitivity_;
    this->issue_command_(this->cmd_speed_angle_sense_, sizeof(this->cmd_speed_angle_sense_));
    this->update_speed_angle_sense_ = false;
    return;
  }

  if (this->update_mode_rate_uom_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_MODE_RATE_UOM");
    this->cmd_mode_rate_uom_[3] = static_cast<uint8_t>(this->tracking_mode_);
    this->cmd_mode_rate_uom_[4] = this->sample_rate_;
    this->issue_command_(this->cmd_mode_rate_uom_, sizeof(this->cmd_mode_rate_uom_));
    this->update_mode_rate_uom_ = false;
    return;
  }

  if (this->update_anti_vib_comp_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_ANTI_VIB_COMP");
    this->cmd_anti_vib_comp_[3] = this->vibration_correction_;
    this->issue_command_(this->cmd_anti_vib_comp_, sizeof(this->cmd_anti_vib_comp_));
    this->update_anti_vib_comp_ = false;
    return;
  }

  if (this->update_relay_duration_speed_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_RELAY_DURATION_SPEED");
    this->cmd_relay_duration_speed_[3] = this->relay_trigger_duration_;
    this->cmd_relay_duration_speed_[4] = this->relay_trigger_speed_;
    this->issue_command_(this->cmd_relay_duration_speed_, sizeof(this->cmd_relay_duration_speed_));
    this->update_relay_duration_speed_ = false;
    return;
  }

  if (this->update_config_) {
    ESP_LOGD(TAG, "LD2415H_CMD_GET_CONFIG");
    this->issue_command_(this->cmd_config_, sizeof(this->cmd_config_));
    this->update_config_ = false;
    return;
  }
}

bool LD2415HComponent::publish_sensor_state_(sensor::Sensor *sensor, float value, const char *sensor_name) {
  if (sensor == nullptr || std::isnan(value) || std::isinf(value)) {
    return false;
  }
  
  // Enhanced rate limiting to prevent memory overflow and reduce log spam
  uint32_t current_time = millis();
  uint32_t min_interval = this->memory_protection_mode_ ? MEMORY_PROTECTION_INTERVAL : MIN_SENSOR_UPDATE_INTERVAL;
  
  if ((current_time - this->last_sensor_update_time_) < min_interval) {
    // Reduce spam from rate limiting messages too
    static uint32_t last_rate_limit_log = 0;
    if ((current_time - last_rate_limit_log) > 5000) {  // Log rate limiting at most every 5 seconds
      ESP_LOGV(TAG, "Rate limiting sensor updates (interval: %u ms)", min_interval);
      last_rate_limit_log = current_time;
    }
    return false;  // Skip this update to prevent memory pressure
  }
  
  this->last_sensor_update_time_ = current_time;
  sensor->publish_state(value);
  return true;
}

// Simple Kalman filter for speed smoothing
double LD2415HComponent::apply_kalman_filter_(double measured_speed) {
  if (!this->filter_initialized_) {
    this->filtered_speed_ = measured_speed;
    this->filter_initialized_ = true;
    return measured_speed;
  }
  
  // Prediction step (no change expected)
  double predicted_speed = this->filtered_speed_;
  double predicted_variance = this->speed_variance_ + 1.0; // Process noise
  
  // Update step
  double kalman_gain = predicted_variance / (predicted_variance + this->measurement_variance_);
  this->filtered_speed_ = predicted_speed + kalman_gain * (measured_speed - predicted_speed);
  this->speed_variance_ = (1 - kalman_gain) * predicted_variance;
  
  return this->filtered_speed_;
}

// Check if a speed measurement is an outlier
bool LD2415HComponent::is_speed_outlier_(double speed) {
  // Basic sanity checks for impossible speeds
  if (speed > 200.0) {  // > 200 km/h is definitely an outlier for most radar applications
    ESP_LOGV(TAG, "Rejecting extreme outlier: %.1f km/h", speed);
    return true;
  }
  
  if (speed < 0.1) {  // Very low speeds are often noise
    return true;
  }
  
  // If we don't have enough history, accept the measurement
  if (this->speed_history_.size() < 3) {
    return false;
  }
  
  // Calculate median of recent measurements for robust comparison
  std::vector<double> sorted_history = this->speed_history_;
  std::sort(sorted_history.begin(), sorted_history.end());
  double median = sorted_history[sorted_history.size() / 2];
  
  // Calculate median absolute deviation (MAD) for robust standard deviation
  std::vector<double> deviations;
  for (double val : sorted_history) {
    deviations.push_back(std::abs(val - median));
  }
  std::sort(deviations.begin(), deviations.end());
  double mad = deviations[deviations.size() / 2];
  
  // Convert MAD to approximate standard deviation
  double robust_std = mad * 1.4826;  // MAD to standard deviation conversion factor
  
  // Reject if more than 3 standard deviations from median
  double deviation = std::abs(speed - median);
  if (robust_std > 0.1 && deviation > (3.0 * robust_std)) {
    ESP_LOGV(TAG, "Rejecting statistical outlier: %.1f km/h (median: %.1f, deviation: %.1f, threshold: %.1f)", 
             speed, median, deviation, 3.0 * robust_std);
    return true;
  }
  
  return false;
}

// Calculate moving average of recent speed measurements
double LD2415HComponent::calculate_moving_average_() {
  if (this->speed_history_.empty()) {
    return 0.0;
  }
  
  double sum = 0.0;
  for (double speed : this->speed_history_) {
    sum += speed;
  }
  
  return sum / this->speed_history_.size();
}

// Apply robust filtering: outlier detection + moving average + Kalman filter
double LD2415HComponent::apply_robust_filter_(double measured_speed) {
  // Step 1: Check for outliers
  if (this->is_speed_outlier_(measured_speed)) {
    // For outliers, return the current filtered value or moving average
    if (this->filter_initialized_) {
      ESP_LOGV(TAG, "Using previous filtered value %.1f instead of outlier %.1f", 
               this->filtered_speed_, measured_speed);
      return this->filtered_speed_;
    } else if (!this->speed_history_.empty()) {
      double avg = this->calculate_moving_average_();
      ESP_LOGV(TAG, "Using moving average %.1f instead of outlier %.1f", avg, measured_speed);
      return avg;
    }
    // If we have no history, reluctantly accept even the outlier but log it
    ESP_LOGW(TAG, "Accepting outlier %.1f km/h due to no history", measured_speed);
  }
  
  // Step 2: Add to history (only non-outliers)
  if (!this->is_speed_outlier_(measured_speed)) {
    this->speed_history_.push_back(measured_speed);
    
    // Limit history size for memory efficiency
    if (this->speed_history_.size() > MAX_HISTORY_SIZE) {
      this->speed_history_.erase(this->speed_history_.begin());
    }
  }
  
  // Step 3: For small changes, use moving average to reduce noise
  double moving_avg = this->calculate_moving_average_();
  double smoothed_speed = measured_speed;
  
  if (this->speed_history_.size() >= 3) {
    // Blend between raw measurement and moving average based on how much it differs
    double avg_diff = std::abs(measured_speed - moving_avg);
    if (avg_diff < 5.0) {  // For small differences (< 5 km/h), use more averaging
      double blend_factor = 0.3;  // 30% new measurement, 70% moving average
      smoothed_speed = blend_factor * measured_speed + (1.0 - blend_factor) * moving_avg;
      ESP_LOGV(TAG, "Smoothing: raw=%.1f, avg=%.1f, smoothed=%.1f", 
               measured_speed, moving_avg, smoothed_speed);
    }
  }
  
  // Step 4: Apply Kalman filter to the smoothed measurement
  double filtered_speed = this->apply_kalman_filter_(smoothed_speed);
  
  ESP_LOGV(TAG, "Robust filter: raw=%.1f → smoothed=%.1f → filtered=%.1f", 
           measured_speed, smoothed_speed, filtered_speed);
  
  return filtered_speed;
}

// Process a single speed measurement
void LD2415HComponent::process_single_measurement_(double speed, bool approaching) {
  // Apply basic debouncing to reduce excessive processing
  static double last_processed_speed = 0;
  static bool last_processed_approaching = false;
  static uint32_t last_process_time = 0;
  
  uint32_t current_time = millis();
  
  // Only process if speed changed significantly or enough time has passed
  double speed_change = std::abs(speed - last_processed_speed);
  bool direction_changed = (approaching != last_processed_approaching);
  uint32_t time_since_last = current_time - last_process_time;
  
  if (speed_change < 2.0 && !direction_changed && time_since_last < 300) {
    // Skip processing if speed change is minor and direction is same and recent
    ESP_LOGV(TAG, "Skipping minor speed change: %.1f km/h (change: %.1f)", speed, speed_change);
    return;
  }
  
  last_processed_speed = speed;
  last_processed_approaching = approaching;
  last_process_time = current_time;
  
  // Update current state for this measurement
  this->speed_ = speed;
  this->approaching_ = approaching;
  this->velocity_ = approaching ? speed : -speed;
  
  // Publish to general speed sensor (always use the most recent measurement)
  if (this->speed_sensor_ != nullptr) {
    this->publish_sensor_state_(this->speed_sensor_, this->speed_, "Speed Sensor");
  }

  if (approaching) {
    if (this->approaching_speed_sensor_ != nullptr) {
      this->publish_sensor_state_(this->approaching_speed_sensor_, this->speed_, "Approaching Speed Sensor");
      this->last_approaching_update_time_ = millis();
    
      // Update last max speed
      if (this->speed_ > this->last_max_approaching_speed_) {
        this->last_max_approaching_speed_ = this->speed_;
        
        // Publish to max speed sensor if available
        if (this->approaching_last_max_speed_sensor_ != nullptr) {
          this->publish_sensor_state_(this->approaching_last_max_speed_sensor_, this->last_max_approaching_speed_, "Approaching Max Speed");
        }
      }
    }
  } else {
    // Handle departing speed
    if (this->departing_speed_sensor_ != nullptr) {
      this->publish_sensor_state_(this->departing_speed_sensor_, this->speed_, "Departing Speed");
      this->last_departing_update_time_ = millis();
    
      // Update last max speed
      if (this->speed_ > this->last_max_departing_speed_) {
        this->last_max_departing_speed_ = this->speed_;
        
        // Publish to max speed sensor if available
        if (this->departing_last_max_speed_sensor_ != nullptr) {
          this->publish_sensor_state_(this->departing_last_max_speed_sensor_, this->last_max_departing_speed_, "Departing Max Speed");
        }
      }
    }
  }

  if (this->velocity_sensor_ != nullptr) {
    this->publish_sensor_state_(this->velocity_sensor_, this->velocity_, "Velocity Sensor");
  }
  
  // Process vehicle tracking for this measurement
  this->process_vehicle_detection_(this->speed_, this->approaching_);
}

#ifdef USE_NUMBER
void LD2415HComponent::update() {
  if (this->update_speed_angle_sense_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_SPEED_ANGLE_SENSE: ");
    this->cmd_speed_angle_sense_[3] = this->min_speed_threshold_;
    this->cmd_speed_angle_sense_[4] = this->compensation_angle_;
    this->cmd_speed_angle_sense_[5] = this->sensitivity_;

    this->issue_command_(this->cmd_speed_angle_sense_, sizeof(this->cmd_speed_angle_sense_));
    this->update_speed_angle_sense_ = false;
    return;
  }

  if (this->update_mode_rate_uom_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_MODE_RATE_UOM: ");
    this->cmd_mode_rate_uom_[3] = static_cast<uint8_t>(this->tracking_mode_);
    this->cmd_mode_rate_uom_[4] = this->sample_rate_;

    this->issue_command_(this->cmd_mode_rate_uom_, sizeof(this->cmd_mode_rate_uom_));
    this->update_mode_rate_uom_ = false;
    return;
  }

  if (this->update_anti_vib_comp_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_ANTI_VIB_COMP: ");
    this->cmd_anti_vib_comp_[3] = this->vibration_correction_;

    this->issue_command_(this->cmd_anti_vib_comp_, sizeof(this->cmd_anti_vib_comp_));
    this->update_anti_vib_comp_ = false;
    return;
  }

  if (this->update_relay_duration_speed_) {
    ESP_LOGD(TAG, "LD2415H_CMD_SET_RELAY_DURATION_SPEED: ");
    this->cmd_relay_duration_speed_[3] = this->relay_trigger_duration_;
    this->cmd_relay_duration_speed_[4] = this->relay_trigger_speed_;

    this->issue_command_(this->cmd_relay_duration_speed_, sizeof(this->cmd_relay_duration_speed_));
    this->update_relay_duration_speed_ = false;
    return;
  }

  if (this->update_config_) {
    ESP_LOGD(TAG, "LD2415H_CMD_GET_CONFIG: ");

    this->issue_command_(this->cmd_config_, sizeof(this->cmd_config_));
    this->update_config_ = false;
    return;
  }
}

void LD2415HComponent::set_min_speed_threshold(uint8_t speed) {
  this->min_speed_threshold_ = speed;
  this->update_speed_angle_sense_ = true;
}

void LD2415HComponent::set_compensation_angle(uint8_t angle) {
  this->compensation_angle_ = angle;
  this->update_speed_angle_sense_ = true;
}

void LD2415HComponent::set_sensitivity(uint8_t sensitivity) {
  this->sensitivity_ = sensitivity;
  this->update_speed_angle_sense_ = true;
}

void LD2415HComponent::set_vibration_correction(uint8_t correction) {
  this->vibration_correction_ = correction;
  this->update_anti_vib_comp_ = true;
}

void LD2415HComponent::set_relay_trigger_duration(uint8_t duration) {
  this->relay_trigger_duration_ = duration;
  this->update_relay_duration_speed_ = true;
}

void LD2415HComponent::set_relay_trigger_speed(uint8_t speed) {
  this->relay_trigger_speed_ = speed;
  this->update_relay_duration_speed_ = true;
}

void LD2415HComponent::set_timeout_duration(uint32_t duration) {
  this->timeout_duration_ = duration;
}
#endif

#ifdef USE_SELECT
void LD2415HComponent::set_tracking_mode(const std::string &state) {
  uint8_t mode = TRACKING_MODE_STR_TO_INT.at(state);
  this->set_tracking_mode(mode);
  this->tracking_mode_selector_->publish_state(state);
}

void LD2415HComponent::set_tracking_mode(TrackingMode mode) {
  this->tracking_mode_ = mode;
  this->update_mode_rate_uom_ = true;
}

void LD2415HComponent::set_tracking_mode(uint8_t mode) { this->set_tracking_mode(i_to_tracking_mode_(mode)); }

void LD2415HComponent::set_sample_rate(const std::string &state) {
  uint8_t rate = SAMPLE_RATE_STR_TO_INT.at(state);
  this->set_sample_rate(rate);
  this->sample_rate_selector_->publish_state(state);
}

void LD2415HComponent::set_sample_rate(uint8_t rate) {
  ESP_LOGD(TAG, "set_sample_rate: %i", rate);
  this->sample_rate_ = rate;
  this->update_mode_rate_uom_ = true;
}
#endif

void LD2415HComponent::issue_command_(const uint8_t cmd[], uint8_t size) {
  for (uint8_t i = 0; i < size; i++)
    ESP_LOGD(TAG, "  0x%02x", cmd[i]);

  // Don't assume the response buffer is empty, clear it before issuing a command.
  clear_remaining_buffer_(0);
  this->write_array(cmd, size);
}

bool LD2415HComponent::fill_buffer_(char c) {
  switch (c) {
    case 0x00:
    case 0xFF:
    case '\r':
      // Ignore these characters (simplified approach like cptskippy)
      break;

    case '\n':
      // End of response
      if (this->response_buffer_index_ == 0)
        break;

      clear_remaining_buffer_(this->response_buffer_index_);
      ESP_LOGV(TAG, "Response Received: %s", this->response_buffer_);
      return true;

    default:
      // Append to response
      if (this->response_buffer_index_ < sizeof(this->response_buffer_) - 1) {
        this->response_buffer_[this->response_buffer_index_] = c;
        this->response_buffer_index_++;
      } else {
        ESP_LOGW(TAG, "Buffer full, clearing");
        this->clear_remaining_buffer_(0);
      }
      break;
  }

  return false;
}

void LD2415HComponent::clear_remaining_buffer_(uint8_t pos) {
  while (pos < sizeof(this->response_buffer_)) {
    this->response_buffer_[pos] = 0x00;
    pos++;
  }

  this->response_buffer_index_ = 0;
}

void LD2415HComponent::parse_buffer_() {
  char c = this->response_buffer_[0];
  
  ESP_LOGV(TAG, "Parsing buffer: first char '%c' (0x%02X)", c, (uint8_t)c);

  switch (c) {
    case 'N':
      // Firmware Version
      ESP_LOGI(TAG, "Parsing firmware response");
      this->parse_firmware_();
      break;
    case 'X':
      // Config Response
      ESP_LOGI(TAG, "Parsing config response");
      this->parse_config_();
      break;
    case 'V':
      // Speed
      ESP_LOGV(TAG, "Parsing speed response");
      this->parse_speed_();
      break;

    default:
      ESP_LOGW(TAG, "Unknown ASCII response type '%c' (0x%02X): %s", c, (uint8_t)c, this->response_buffer_);
      break;
  }
}

void LD2415HComponent::parse_config_() {
  // Example: "X1:01 X2:00 X3:05 X4:01 X5:00 X6:00 X7:05 X8:03 X9:01 X0:01"

  const char *delim = ": ";
  uint8_t token_len = 2;
  char *key;
  char *val;

  char *token = strtok(this->response_buffer_, delim);

  while (token != nullptr) {
    if (std::strlen(token) != token_len) {
      ESP_LOGE(TAG, "Configuration key length invalid.");
      break;
    }
    key = token;

    token = strtok(nullptr, delim);
    if (std::strlen(token) != token_len) {
      ESP_LOGE(TAG, "Configuration value length invalid.");
      break;
    }
    val = token;

    this->parse_config_param_(key, val);

    token = strtok(nullptr, delim);
  }

  ESP_LOGD(TAG, "Configuration received:");
  ESP_LOGCONFIG(TAG, "LD2415H:");
  ESP_LOGCONFIG(TAG, "  Firmware: %s", this->firmware_);
  ESP_LOGCONFIG(TAG, "  Minimum Speed Threshold: %u KPH", this->min_speed_threshold_);
  ESP_LOGCONFIG(TAG, "  Compensation Angle: %u", this->compensation_angle_);
  ESP_LOGCONFIG(TAG, "  Sensitivity: %u", this->sensitivity_);
  ESP_LOGCONFIG(TAG, "  Tracking Mode: %s", tracking_mode_to_s_(this->tracking_mode_));
  ESP_LOGCONFIG(TAG, "  Sampling Rate: %u", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Unit of Measure: %s", unit_of_measure_to_s_(this->unit_of_measure_));
  ESP_LOGCONFIG(TAG, "  Vibration Correction: %u", this->vibration_correction_);
  ESP_LOGCONFIG(TAG, "  Relay Trigger Duration: %u", this->relay_trigger_duration_);
  ESP_LOGCONFIG(TAG, "  Relay Trigger Speed: %u KPH", this->relay_trigger_speed_);
  ESP_LOGCONFIG(TAG, "  Negotiation Mode: %s", negotiation_mode_to_s_(this->negotiation_mode_));
}

void LD2415HComponent::parse_firmware_() {
  // Example: "No.:20230801E v5.0"

  const char *fw = strchr(this->response_buffer_, ':');

  if (fw != nullptr) {
    // Move p to the character after ':'
    ++fw;

    // Copy string into firmware
    std::strncpy(this->firmware_, fw, sizeof(this->firmware_));
  } else {
    ESP_LOGE(TAG, "Firmware value invalid.");
  }
}

void LD2415HComponent::parse_speed_() {
  // Parse multiple concatenated measurements like "V+004.6V+040.3", "V+0V+038.1", or "V+000V+001.7"
  // Work directly with char buffer to minimize memory allocation
  
  char *buffer = this->response_buffer_;
  size_t len = strlen(buffer);
  
  // Find all 'V' characters and process each measurement
  for (size_t pos = 0; pos < len; pos++) {
    if (buffer[pos] != 'V') continue;
    
    // Find the next 'V' to determine the length of this measurement
    size_t next_v_pos = len; // Default to end of buffer
    for (size_t i = pos + 1; i < len; i++) {
      if (buffer[i] == 'V') {
        next_v_pos = i;
        break;
      }
    }
    
    size_t measurement_length = next_v_pos - pos;
    
    // Need at least 3 characters for a minimal measurement (V±X)
    if (measurement_length < 3) {
      continue;
    }
    
    // Extract this measurement into a temporary buffer
    char measurement[32]; // Local buffer to avoid heap allocation
    if (measurement_length >= sizeof(measurement)) {
      measurement_length = sizeof(measurement) - 1;
    }
    strncpy(measurement, &buffer[pos], measurement_length);
    measurement[measurement_length] = '\0';
    
    // Validate and parse the measurement
    if (measurement_length >= 3 && 
        (measurement[1] == '+' || measurement[1] == '-')) {
      
      // Extract the numeric part after V± directly
      char speed_str[16];
      strncpy(speed_str, &measurement[2], sizeof(speed_str) - 1);
      speed_str[sizeof(speed_str) - 1] = '\0';
      
      // Handle cases where decimal point might be missing (like "V+000" instead of "V+000.0")
      size_t speed_len = strlen(speed_str);
      if (strchr(speed_str, '.') == nullptr && speed_len >= 3) {
        // Insert decimal point before last digit if it looks like integer format
        if (speed_len < sizeof(speed_str) - 1) {
          memmove(&speed_str[speed_len - 1 + 1], &speed_str[speed_len - 1], 2);
          speed_str[speed_len - 1] = '.';
        }
      }
      
      // Simple validation: check if string contains only digits and decimal point
      bool valid = true;
      for (size_t i = 0; speed_str[i] != '\0'; i++) {
        if (!std::isdigit(speed_str[i]) && speed_str[i] != '.') {
          valid = false;
          break;
        }
      }
      
      if (valid && speed_str[0] != '\0') {
        bool approaching = (measurement[1] == '+');
        double speed = std::atof(speed_str);
        
        // Only process valid speed values (ignore very small or truncated values)
        if (speed >= 0.1) { // Minimum meaningful speed
          // Convert to absolute speed for all processing and reporting
          speed = std::abs(speed);
          
          // Apply robust filtering (outlier detection + moving average + Kalman)
          speed = this->apply_robust_filter_(speed);
          
          // Process this measurement
          this->process_single_measurement_(speed, approaching);
        }
      } else {
        // Skip invalid measurements (use LOGV to reduce spam)
        ESP_LOGV(TAG, "Failed to parse measurement: invalid format");
      }
    }
  }
}

bool LD2415HComponent::parse_hex_speed_packet_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "Parsing hex speed packet (%zu bytes)", data.size());
  
  if (data.size() < 7 || data[0] != 0x56) {
    ESP_LOGW(TAG, "Invalid hex packet: size %zu or doesn't start with 'V' (0x56)", data.size());
    return false;
  }
  
  // Convert hex bytes to ASCII string for easier parsing
  std::string ascii_data;
  for (size_t i = 0; i < data.size(); i++) {
    if (data[i] >= 0x20 && data[i] <= 0x7E) { // Printable ASCII
      ascii_data += (char)data[i];
    } else if (data[i] == 0x0D || data[i] == 0x0A) {
      // Carriage return or line feed - end of packet
      break;
    } else {
      ESP_LOGV(TAG, "Non-printable byte at position %zu: 0x%02X", i, data[i]);
    }
  }
  
  ESP_LOGI(TAG, "ASCII interpretation: '%s'", ascii_data.c_str());
  
  if (ascii_data.length() < 6) { // At least "V+0.0" or "V-0.0"
    ESP_LOGW(TAG, "ASCII data too short: '%s'", ascii_data.c_str());
    return false;
  }
  
  // Check for valid direction character
  char direction = ascii_data[1];
  if (direction != '+' && direction != '-') {
    ESP_LOGW(TAG, "Invalid direction character: '%c' (expected + or -)", direction);
    return false;
  }
  
  bool is_approaching = (direction == '+');
  
  // Extract speed portion (everything after the direction character)
  std::string speed_str = ascii_data.substr(2);
  
  // Parse the speed value
  float speed = 0.0f;
  if (sscanf(speed_str.c_str(), "%f", &speed) != 1) {
    ESP_LOGW(TAG, "Failed to parse speed from: '%s'", speed_str.c_str());
    return false;
  }
  
  ESP_LOGI(TAG, "Hex speed parsed successfully:");
  ESP_LOGI(TAG, "  ASCII packet: '%s'", ascii_data.c_str());
  ESP_LOGI(TAG, "  Direction: %s (%c)", is_approaching ? "approaching" : "departing", direction);
  ESP_LOGI(TAG, "  Speed: %.3f km/h", speed);
  
  // Update sensor values
  this->approaching_ = is_approaching;
  this->velocity_ = is_approaching ? speed : -speed;
  this->speed_ = std::abs(speed);
  
  // Apply Kalman filtering for noise reduction
  this->speed_ = this->apply_kalman_filter_(this->speed_);
  
  // Publish sensor data
  if (this->speed_sensor_ != nullptr) {
    this->publish_sensor_state_(this->speed_sensor_, this->speed_, "Speed Sensor");
  }

  if (this->approaching_) {
    if (this->approaching_speed_sensor_ != nullptr) {
      this->publish_sensor_state_(this->approaching_speed_sensor_, this->speed_, "Approaching Speed Sensor");
      this->last_approaching_update_time_ = millis();
    }
  } else {
    if (this->departing_speed_sensor_ != nullptr) {
      this->publish_sensor_state_(this->departing_speed_sensor_, this->speed_, "Departing Speed Sensor");
      this->last_departing_update_time_ = millis();
    }
  }

  if (this->velocity_sensor_ != nullptr) {
    this->publish_sensor_state_(this->velocity_sensor_, this->velocity_, "Velocity Sensor");
  }

  // Process vehicle tracking
  this->process_vehicle_detection_(this->speed_, this->approaching_);
  
  return true;
}

void LD2415HComponent::parse_config_param_(char *key, char *value) {
  if (std::strlen(key) != 2 || std::strlen(value) != 2 || key[0] != 'X') {
    ESP_LOGE(TAG, "Invalid Parameter %s:%s", key, value);
    return;
  }

  uint8_t v = std::stoi(value, nullptr, 16);

  switch (key[1]) {
    case '1':
      this->min_speed_threshold_ = v;
      break;
    case '2':
      this->compensation_angle_ = std::stoi(value, nullptr, 16);
      break;
    case '3':
      this->sensitivity_ = std::stoi(value, nullptr, 16);
      break;
    case '4':
      this->tracking_mode_ = this->i_to_tracking_mode_(v);
      break;
    case '5':
      this->sample_rate_ = v;
      break;
    case '6':
      this->unit_of_measure_ = this->i_to_unit_of_measure_(v);
      break;
    case '7':
      this->vibration_correction_ = v;
      break;
    case '8':
      this->relay_trigger_duration_ = v;
      break;
    case '9':
      this->relay_trigger_speed_ = v;
      break;
    case '0':
      this->negotiation_mode_ = this->i_to_negotiation_mode_(v);
      break;
    default:
      ESP_LOGD(TAG, "Unknown Parameter %s:%s", key, value);
      break;
  }
}

TrackingMode LD2415HComponent::i_to_tracking_mode_(uint8_t value) {
  TrackingMode u = TrackingMode(value);
  switch (u) {
    case TrackingMode::APPROACHING_AND_RETREATING:
      return TrackingMode::APPROACHING_AND_RETREATING;
    case TrackingMode::APPROACHING:
      return TrackingMode::APPROACHING;
    case TrackingMode::RETREATING:
      return TrackingMode::RETREATING;
    default:
      ESP_LOGE(TAG, "Invalid TrackingMode:%u", value);
      return TrackingMode::APPROACHING_AND_RETREATING;
  }
}

UnitOfMeasure LD2415HComponent::i_to_unit_of_measure_(uint8_t value) {
  UnitOfMeasure u = UnitOfMeasure(value);
  switch (u) {
    case UnitOfMeasure::MPS:
      return UnitOfMeasure::MPS;
    case UnitOfMeasure::MPH:
      return UnitOfMeasure::MPH;
    case UnitOfMeasure::KPH:
      return UnitOfMeasure::KPH;
    default:
      ESP_LOGE(TAG, "Invalid UnitOfMeasure:%u", value);
      return UnitOfMeasure::KPH;
  }
}

NegotiationMode LD2415HComponent::i_to_negotiation_mode_(uint8_t value) {
  NegotiationMode u = NegotiationMode(value);

  switch (u) {
    case NegotiationMode::CUSTOM_AGREEMENT:
      return NegotiationMode::CUSTOM_AGREEMENT;
    case NegotiationMode::STANDARD_PROTOCOL:
      return NegotiationMode::STANDARD_PROTOCOL;
    default:
      ESP_LOGE(TAG, "Invalid NegotiationMode:%u", value);
      return NegotiationMode::CUSTOM_AGREEMENT;
  }
}

const char *LD2415HComponent::tracking_mode_to_s_(TrackingMode value) {
  switch (value) {
    case TrackingMode::APPROACHING_AND_RETREATING:
      return "APPROACHING_AND_RETREATING";
    case TrackingMode::APPROACHING:
      return "APPROACHING";
    case TrackingMode::RETREATING:
    default:
      return "RETREATING";
  }
}

const char *LD2415HComponent::unit_of_measure_to_s_(UnitOfMeasure value) {
  switch (value) {
    case UnitOfMeasure::MPS:
      return "MPS";
    case UnitOfMeasure::MPH:
      return "MPH";
    case UnitOfMeasure::KPH:
    default:
      return "KPH";
  }
}

const char *LD2415HComponent::negotiation_mode_to_s_(NegotiationMode value) {
  switch (value) {
    case NegotiationMode::CUSTOM_AGREEMENT:
      return "CUSTOM_AGREEMENT";
    case NegotiationMode::STANDARD_PROTOCOL:
    default:
      return "STANDARD_PROTOCOL";
  }
}

const char *LD2415HComponent::i_to_s_(const std::map<std::string, uint8_t> &map, uint8_t i) {
  for (const auto &pair : map) {
    if (pair.second == i) {
      return pair.first.c_str();
    }
  }
  return "Unknown";
}

// Vehicle tracking implementation
void LD2415HComponent::process_vehicle_detection_(double speed, bool is_approaching) {
  // Only process if speed is above minimum threshold
  if (speed < MIN_DETECTION_SPEED) {
    return;
  }
  
  Vehicle* vehicle = this->find_or_create_vehicle_(speed, is_approaching);
  if (vehicle != nullptr) {
    this->update_vehicle_(vehicle, speed, millis());
  }
}

Vehicle* LD2415HComponent::find_or_create_vehicle_(double speed, bool is_approaching) {
  std::vector<Vehicle>& vehicles = is_approaching ? approaching_vehicles_ : departing_vehicles_;
  uint32_t current_time = millis();
  
  // Update direction tracking
  if (is_approaching) {
    this->last_approaching_time_ = current_time;
  } else {
    this->last_departing_time_ = current_time;
  }
  
  // Check for potential overtake situation
  bool overtake_situation = this->is_potential_overtake_(is_approaching);
  
  // Look for existing active vehicle using conservative grouping
  Vehicle* best_match = nullptr;
  double best_probability = 0.0;
  uint32_t best_time_gap = UINT32_MAX;
  
  for (auto& vehicle : vehicles) {
    if (!vehicle.is_active) continue;
    
    uint32_t time_gap = current_time - vehicle.last_detection_time;
    
    // Skip vehicles that haven't been seen for too long
    if (time_gap > VEHICLE_TIMEOUT) {
      // Mark current vehicle as finished and publish final data
      this->publish_vehicle_data_(vehicle, true);
      vehicle.is_active = false;
      continue;
    }
    
    // Use conservative grouping logic
    if (this->is_same_vehicle_conservative_(vehicle, speed, time_gap)) {
      // Calculate probability for this match
      double probability = this->calculate_vehicle_grouping_probability_(vehicle, speed, time_gap);
      
      ESP_LOGV(TAG, "Vehicle #%u match probability: %.2f (speed: %.1f→%.1f, time_gap: %u ms)", 
               vehicle.id, probability, vehicle.last_speed, speed, time_gap);
      
      // Select best match (highest probability, or if tied, most recent)
      if (probability > best_probability || 
          (probability == best_probability && time_gap < best_time_gap)) {
        best_match = &vehicle;
        best_probability = probability;
        best_time_gap = time_gap;
      }
    }
  }
  
  // Use best match if probability is high enough
  const double MIN_MATCH_PROBABILITY = overtake_situation ? 0.7 : 0.5;  // Higher threshold during overtakes
  
  if (best_match && best_probability >= MIN_MATCH_PROBABILITY) {
    // Rate limit vehicle update logs to reduce spam
    uint32_t current_time = millis();
    if ((current_time - this->last_vehicle_detection_log_time_) > MIN_LOG_UPDATE_INTERVAL) {
      ESP_LOGD(TAG, "Updated existing %s vehicle #%u: %.1f km/h -> %.1f km/h (probability: %.2f, time gap: %u ms)", 
               is_approaching ? "approaching" : "departing", best_match->id, 
               best_match->last_speed, speed, best_probability, best_time_gap);
      this->last_vehicle_detection_log_time_ = current_time;
    }
    
    return best_match;
  }
  
  // No good match found - create new vehicle only if speed is significant enough
  if (speed < MIN_STABLE_SPEED) {
    ESP_LOGV(TAG, "Speed too low for new vehicle creation: %.1f km/h < %.1f km/h", speed, MIN_STABLE_SPEED);
    return nullptr;  // Don't create vehicles for very low speeds
  }
  
  // During overtakes, be extra conservative about creating new vehicles
  if (overtake_situation && speed < MIN_STABLE_SPEED * 2) {
    ESP_LOGD(TAG, "Overtake situation: not creating new vehicle for low speed %.1f km/h", speed);
    return nullptr;
  }
  
  // No matching vehicle found, create a new one
  Vehicle new_vehicle;
  new_vehicle.id = next_vehicle_id_++;
  new_vehicle.is_approaching = is_approaching;
  new_vehicle.is_active = true;
  new_vehicle.first_detection_time = current_time;
  new_vehicle.last_speed = speed;
  new_vehicle.max_speed = speed;
  new_vehicle.last_detection_time = current_time;
  
  vehicles.push_back(new_vehicle);
  
  // Update vehicle counters
  if (is_approaching) {
    approaching_vehicle_count_++;
    if (this->approaching_vehicle_count_sensor_ != nullptr) {
      this->approaching_vehicle_count_sensor_->publish_state(approaching_vehicle_count_);
    }
  } else {
    departing_vehicle_count_++;
    if (this->departing_vehicle_count_sensor_ != nullptr) {
      this->departing_vehicle_count_sensor_->publish_state(departing_vehicle_count_);
    }
  }
  
  ESP_LOGI(TAG, "New %s vehicle #%u detected: %.1f km/h%s", 
           is_approaching ? "approaching" : "departing", new_vehicle.id, speed,
           overtake_situation ? " (during potential overtake)" : "");
  
  return &vehicles.back();
}

void LD2415HComponent::update_vehicle_(Vehicle* vehicle, double speed, uint32_t current_time) {
  vehicle->last_speed = speed;
  vehicle->last_detection_time = current_time;
  
  // Update max speed if this is higher
  if (speed > vehicle->max_speed) {
    vehicle->max_speed = speed;
  }
  
  // Publish current vehicle data
  this->publish_vehicle_data_(*vehicle, false);
}

void LD2415HComponent::cleanup_inactive_vehicles_() {
  uint32_t current_time = millis();
  
  // Clean up approaching vehicles
  for (auto it = approaching_vehicles_.begin(); it != approaching_vehicles_.end();) {
    if (it->is_active && (current_time - it->last_detection_time > VEHICLE_TIMEOUT)) {
      this->publish_vehicle_data_(*it, true);
      it->is_active = false;
    }
    
    // Remove inactive vehicles older than 10 seconds to prevent memory buildup
    if (!it->is_active && (current_time - it->last_detection_time > 10000)) {
      it = approaching_vehicles_.erase(it);
    } else {
      ++it;
    }
  }
  
  // Clean up departing vehicles
  for (auto it = departing_vehicles_.begin(); it != departing_vehicles_.end();) {
    if (it->is_active && (current_time - it->last_detection_time > VEHICLE_TIMEOUT)) {
      this->publish_vehicle_data_(*it, true);
      it->is_active = false;
    }
    
    // Remove inactive vehicles older than 10 seconds to prevent memory buildup
    if (!it->is_active && (current_time - it->last_detection_time > 10000)) {
      it = departing_vehicles_.erase(it);
    } else {
      ++it;
    }
  }
}

void LD2415HComponent::publish_vehicle_data_(const Vehicle& vehicle, bool is_final) {
  if (vehicle.is_approaching) {
    if (this->current_approaching_vehicle_speed_sensor_ != nullptr) {
      if (is_final) {
        // Publish final max speed to Home Assistant
        this->publish_sensor_state_(this->current_approaching_vehicle_speed_sensor_, vehicle.max_speed, "Approaching Vehicle Max Speed");
      } else {
        // Publish current speed
        this->publish_sensor_state_(this->current_approaching_vehicle_speed_sensor_, vehicle.last_speed, "Approaching Vehicle Current Speed");
      }
    }
  } else {
    if (this->current_departing_vehicle_speed_sensor_ != nullptr) {
      if (is_final) {
        // Publish final max speed to Home Assistant
        this->publish_sensor_state_(this->current_departing_vehicle_speed_sensor_, vehicle.max_speed, "Departing Vehicle Max Speed");
      } else {
        // Publish current speed
        this->publish_sensor_state_(this->current_departing_vehicle_speed_sensor_, vehicle.last_speed, "Departing Vehicle Current Speed");
      }
    }
  }
}

// Conservative vehicle grouping implementation
bool LD2415HComponent::is_same_vehicle_conservative_(const Vehicle& vehicle, double new_speed, uint32_t time_gap) {
  // Extremely conservative approach - prefer stability over accuracy
  
  // 1. Time gap check - if too much time has passed, it's likely a new vehicle
  if (time_gap > CONSERVATIVE_TIME_GAP) {
    // Rate limit these debug messages to reduce log spam
    uint32_t current_time = millis();
    if ((current_time - this->last_conservative_log_time_) > MIN_LOG_UPDATE_INTERVAL) {
      ESP_LOGD(TAG, "Time gap too large: %u ms > %u ms - treating as new vehicle", 
               time_gap, CONSERVATIVE_TIME_GAP);
      this->last_conservative_log_time_ = current_time;
    }
    return false;
  }
  
  // 2. Speed change check - if speed changed too much, likely a different vehicle
  double speed_diff = std::abs(vehicle.last_speed - new_speed);
  if (speed_diff > CONSERVATIVE_SPEED_THRESHOLD) {
    // Rate limit these debug messages to reduce log spam
    uint32_t current_time = millis();
    if ((current_time - this->last_conservative_log_time_) > MIN_LOG_UPDATE_INTERVAL) {
      ESP_LOGD(TAG, "Speed change too large: %.1f km/h vs %.1f km/h (diff: %.1f) - treating as new vehicle", 
               vehicle.last_speed, new_speed, speed_diff);
      this->last_conservative_log_time_ = current_time;
    }
    return false;
  }
  
  // 3. Minimum detection duration - vehicle must have been detected for minimum time to be "real"
  uint32_t detection_duration = time_gap + (vehicle.last_detection_time - vehicle.first_detection_time);
  if (detection_duration < MIN_DETECTION_DURATION) {
    ESP_LOGV(TAG, "Detection too brief: %u ms < %u ms - might be noise", 
             detection_duration, MIN_DETECTION_DURATION);
    // Don't reject immediately, but be more strict on other criteria
  }
  
  // 4. Speed stability check - both speeds should be above minimum threshold
  if (vehicle.last_speed < MIN_STABLE_SPEED || new_speed < MIN_STABLE_SPEED) {
    ESP_LOGV(TAG, "Speed too low for stable detection: last=%.1f, new=%.1f (min=%.1f)", 
             vehicle.last_speed, new_speed, MIN_STABLE_SPEED);
    return false;
  }
  
  // Rate limit the "same vehicle" debug messages to reduce log spam
  uint32_t current_time = millis();
  if ((current_time - this->last_conservative_log_time_) > MIN_LOG_UPDATE_INTERVAL) {
    ESP_LOGD(TAG, "Conservative grouping: same vehicle (speed: %.1f→%.1f, time_gap: %u ms)", 
             vehicle.last_speed, new_speed, time_gap);
    this->last_conservative_log_time_ = current_time;
  }
  return true;
}

bool LD2415HComponent::is_potential_overtake_(bool is_approaching) {
  uint32_t current_time = millis();
  
  // Check if we've seen traffic in both directions recently (potential overtake)
  bool recent_approaching = (current_time - this->last_approaching_time_) < OVERTAKE_PROTECTION_TIME;
  bool recent_departing = (current_time - this->last_departing_time_) < OVERTAKE_PROTECTION_TIME;
  
  if (recent_approaching && recent_departing) {
    if (!this->potential_overtake_in_progress_) {
      ESP_LOGW(TAG, "Potential overtake detected - being extra conservative with vehicle counting");
      this->potential_overtake_in_progress_ = true;
      this->overtake_start_time_ = current_time;
    }
    return true;
  }
  
  // Clear overtake flag if enough time has passed
  if (this->potential_overtake_in_progress_ && 
      !recent_approaching && !recent_departing) {
    uint32_t overtake_duration = current_time - this->overtake_start_time_;
    ESP_LOGI(TAG, "Overtake situation cleared after %u ms", overtake_duration);
    this->potential_overtake_in_progress_ = false;
    this->overtake_start_time_ = 0;
  }
  
  return false;
}

double LD2415HComponent::calculate_vehicle_grouping_probability_(const Vehicle& vehicle, double new_speed, uint32_t time_gap) {
  // Calculate probability (0.0 to 1.0) that this is the same vehicle
  double probability = 1.0;
  uint32_t current_time = millis();
  
  // Time penalty - exponential decay
  double time_factor = static_cast<double>(time_gap) / CONSERVATIVE_TIME_GAP;
  probability *= std::exp(-time_factor * 2.0);  // Exponential decay
  
  // Speed change penalty
  double speed_diff = std::abs(vehicle.last_speed - new_speed);
  double speed_factor = speed_diff / CONSERVATIVE_SPEED_THRESHOLD;
  probability *= std::exp(-speed_factor * 2.0);  // Exponential decay
  
  // Bonus for stable, high-speed detections
  double min_speed = std::min(vehicle.last_speed, new_speed);
  if (min_speed > MIN_STABLE_SPEED * 2) {
    probability *= 1.2;  // 20% bonus for stable detections
  }
  
  // Penalty during potential overtakes (enhanced)
  if (this->potential_overtake_in_progress_) {
    uint32_t overtake_duration = current_time - this->overtake_start_time_;
    
    // Apply stronger penalty for longer overtakes
    double overtake_penalty = 0.4;  // Base 60% penalty during overtakes
    if (overtake_duration > 3000) {  // If overtake lasts more than 3 seconds
      overtake_penalty = 0.2;  // Apply even stronger penalty (80% reduction)
    }
    
    probability *= overtake_penalty;
    ESP_LOGD(TAG, "Overtake penalty applied: probability reduced to %.2f (duration: %u ms)", 
             probability, overtake_duration);
  }
  
  return std::min(probability, 1.0);
}

// Simplified speed parsing based on dermodmaster's approach
bool LD2415HComponent::parse_speed_(const std::string& line) {
  if (line.length() < 6) return false;
  
  // Simple speed data format check - look for speed pattern
  if (line.find("AA FF") == 0 || line.find("AAFF") == 0) {
    try {
      // Extract speed value - simplified approach
      size_t speed_pos = line.find("03 02", 4);  // Look for speed data identifier
      if (speed_pos != std::string::npos && speed_pos + 10 < line.length()) {
        std::string speed_hex = line.substr(speed_pos + 6, 4);
        // Remove spaces if any
        speed_hex.erase(std::remove(speed_hex.begin(), speed_hex.end(), ' '), speed_hex.end());
        
        if (speed_hex.length() >= 4) {
          uint16_t speed_raw = std::stoi(speed_hex, nullptr, 16);
          float speed_ms = static_cast<float>(speed_raw) / 100.0f;  // Convert to m/s
          float speed_kmh = speed_ms * 3.6f;  // Convert to km/h
          
          if (speed_kmh > 0.5f && speed_kmh < 250.0f) {  // Reasonable speed range
            this->handle_speed_detection_(speed_kmh);
            return true;
          }
        }
      }
    } catch (const std::exception& e) {
      ESP_LOGV(TAG, "Speed parsing error: %s", e.what());
    }
  }
  
  return false;
}

void LD2415HComponent::handle_speed_detection_(float speed) {
  uint32_t current_time = millis();
  
  // Simple vehicle tracking with timeout
  if (this->current_vehicle_timeout_ > 0 && 
      (current_time - this->last_detection_time_) > this->current_vehicle_timeout_) {
    // Vehicle timed out - finish current vehicle if any
    if (this->current_vehicle_max_speed_ > 0) {
      this->finish_current_vehicle_();
    }
    this->reset_current_vehicle_();
  }
  
  // Update current vehicle tracking
  if (this->current_vehicle_max_speed_ == 0) {
    // Start new vehicle
    this->current_vehicle_start_time_ = current_time;
    ESP_LOGD(TAG, "New vehicle detected with speed: %.1f km/h", speed);
  }
  
  // Update max speed and counts
  if (speed > this->current_vehicle_max_speed_) {
    this->current_vehicle_max_speed_ = speed;
  }
  
  this->last_detection_time_ = current_time;
  
  // Update sensors
  if (this->speed_sensor_ != nullptr) {
    this->speed_sensor_->publish_state(speed);
  }
  
  if (this->velocity_sensor_ != nullptr) {
    this->velocity_sensor_->publish_state(speed / 3.6f);  // m/s
  }
}

void LD2415HComponent::finish_current_vehicle_() {
  if (this->current_vehicle_max_speed_ > 0) {
    ESP_LOGI(TAG, "Vehicle finished - Max speed: %.1f km/h", this->current_vehicle_max_speed_);
    
    // Update vehicle count
    this->total_vehicle_count_++;
    
    // Update sensors with final values
    if (this->approaching_last_max_speed_sensor_ != nullptr) {
      this->approaching_last_max_speed_sensor_->publish_state(this->current_vehicle_max_speed_);
    }
    
    if (this->approaching_vehicle_count_sensor_ != nullptr) {
      this->approaching_vehicle_count_sensor_->publish_state(this->total_vehicle_count_);
    }
  }
}

void LD2415HComponent::reset_current_vehicle_() {
  this->current_vehicle_max_speed_ = 0;
  this->current_vehicle_start_time_ = 0;
  this->last_detection_time_ = 0;
}

// Memory protection implementation
void LD2415HComponent::enable_memory_protection_() {
  if (!this->memory_protection_mode_) {
    ESP_LOGW(TAG, "Enabling memory protection mode due to allocation failures");
    this->memory_protection_mode_ = true;
  }
}

void LD2415HComponent::check_memory_status_() {
  // Check for low memory conditions and enable protection if needed
  // This is a simplified approach - in a real implementation you might check heap size
  uint32_t current_time = millis();
  static uint32_t last_memory_check = 0;
  
  if ((current_time - last_memory_check) > 10000) {  // Check every 10 seconds
    last_memory_check = current_time;
    
    // If we've been in protection mode for a while, try to exit it
    if (this->memory_protection_mode_) {
      ESP_LOGI(TAG, "Attempting to exit memory protection mode");
      this->memory_protection_mode_ = false;
    }
  }
}

}  // namespace ld2415h
}  // namespace esphome
