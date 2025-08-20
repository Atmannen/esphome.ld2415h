#include "ld2415h.h"
#include "esphome/core/log.h"
#include <cmath>

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
  
  ESP_LOGI(TAG, "LD2415H Component initialized and ready for data");
  ESP_LOGI(TAG, "Waiting for sensor packets...");
  ESP_LOGI(TAG, "Protocol mode: %s", this->is_binary_protocol_ ? "BINARY" : "ASCII");
  ESP_LOGI(TAG, "Using hex-to-ASCII parser to handle binary protocol data");
  
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
  // Clean up inactive vehicles periodically
  this->cleanup_inactive_vehicles_();
  
  // Publish timeout values
  uint32_t current_time = millis();
  if (this->approaching_last_max_speed_sensor_ != nullptr &&
      current_time - this->last_approaching_update_time_ > this->timeout_duration_ &&
      this->last_max_approaching_speed_ > 0) {
    this->publish_sensor_state_(this->approaching_last_max_speed_sensor_, last_max_approaching_speed_, "Approaching Last Max Speed Sensor (Timeout)");
    this->last_max_approaching_speed_ = 0;
  }

  if (this->departing_last_max_speed_sensor_ != nullptr &&
      current_time - this->last_departing_update_time_ > this->timeout_duration_ &&
      this->last_max_departing_speed_ > 0) {
    this->publish_sensor_state_(this->departing_last_max_speed_sensor_, last_max_departing_speed_, "Departing Last Max Speed Sensor (Timeout)");
    this->last_max_departing_speed_ = 0;
  }

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->fill_buffer_(byte)) {
      this->parse_buffer_();
    }
  }
}

bool LD2415HComponent::publish_sensor_state_(sensor::Sensor *sensor, float value, const char *sensor_name) {
  if (sensor == nullptr || std::isnan(value) || std::isinf(value)) {
    return false;
  }
  
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

// Process a single speed measurement
void LD2415HComponent::process_single_measurement_(double speed, bool approaching) {
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
      // Ignore these characters
      ESP_LOGV(TAG, "Ignoring character: 0x%02X", (uint8_t)c);
      break;

    case '\n':
      // End of response
      if (this->response_buffer_index_ == 0) {
        ESP_LOGV(TAG, "Received newline but buffer is empty, ignoring");
        break;
      }

      clear_remaining_buffer_(this->response_buffer_index_);
      ESP_LOGI(TAG, "Complete ASCII response received (%d chars): '%s'", this->response_buffer_index_, this->response_buffer_);
      return true;

    default:
      // Append to response
      if (this->response_buffer_index_ < sizeof(this->response_buffer_) - 1) {
        this->response_buffer_[this->response_buffer_index_] = c;
        this->response_buffer_index_++;
        ESP_LOGV(TAG, "Added char '%c' (0x%02X) to buffer at pos %d", c, (uint8_t)c, this->response_buffer_index_ - 1);
      } else {
        ESP_LOGW(TAG, "ASCII response buffer full, dropping character '%c' (0x%02X)", c, (uint8_t)c);
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
  
  ESP_LOGI(TAG, "Parsing ASCII buffer: '%s' (first char: '%c' 0x%02X)", this->response_buffer_, c, (uint8_t)c);
  ESP_LOGI(TAG, "Protocol flag is set to: %s (but currently always parsing as ASCII)", this->is_binary_protocol_ ? "BINARY" : "ASCII");

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
      ESP_LOGI(TAG, "Parsing speed response");
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
  
  std::string buffer_str = std::string(this->response_buffer_);
  size_t pos = 0;
  
  // Find all 'V' characters and process each measurement
  while ((pos = buffer_str.find('V', pos)) != std::string::npos) {
    // Find the next 'V' to determine the length of this measurement
    size_t next_v = buffer_str.find('V', pos + 1);
    size_t measurement_length = (next_v != std::string::npos) ? (next_v - pos) : (buffer_str.length() - pos);
    
    // Need at least 3 characters for a minimal measurement (V±X)
    if (measurement_length < 3) {
      pos++;
      continue;
    }
    
    // Extract this measurement
    std::string measurement = buffer_str.substr(pos, measurement_length);
    
    // Validate and parse the measurement
    if (measurement.length() >= 3 && 
        (measurement[1] == '+' || measurement[1] == '-')) {
      
      // Extract the numeric part after V±
      std::string speed_str = measurement.substr(2);
      
      // Handle cases where decimal point might be missing (like "V+000" instead of "V+000.0")
      if (speed_str.find('.') == std::string::npos && speed_str.length() >= 3) {
        // Insert decimal point before last digit if it looks like integer format
        speed_str.insert(speed_str.length() - 1, ".");
      }
      
      // Simple validation: check if string contains only digits, decimal point, and optional sign
      bool valid = true;
      for (char c : speed_str) {
        if (!std::isdigit(c) && c != '.') {
          valid = false;
          break;
        }
      }
      
      if (valid && !speed_str.empty()) {
        bool approaching = (measurement[1] == '+');
        double speed = std::atof(speed_str.c_str());
        
        // Only process valid speed values (ignore very small or truncated values)
        if (speed >= 0.1) { // Minimum meaningful speed
          // Convert to absolute speed for all processing and reporting
          speed = std::abs(speed);
          
          // Apply Kalman filtering for noise reduction
          speed = this->apply_kalman_filter_(speed);
          
          // Process this measurement
          this->process_single_measurement_(speed, approaching);
        }
      } else {
        // Skip invalid measurements
        ESP_LOGV(TAG, "Failed to parse measurement '%s': invalid format", measurement.c_str());
      }
    }
    
    // Move to next potential 'V'
    pos++;
  }
}

void LD2415HComponent::parse_binary_speed_(const std::vector<uint8_t> &data) {
  ESP_LOGI(TAG, "Parsing binary speed packet (%zu bytes)", data.size());
  
  if (data.size() != 9) {
    ESP_LOGW(TAG, "Invalid binary packet size: %zu (expected 9)", data.size());
    return;
  }
  
  // Check if it starts with 'V' (0x56) and ends with 0x0D 0x0A
  if (data[0] != 0x56 || data[7] != 0x0D || data[8] != 0x0A) {
    ESP_LOGW(TAG, "Invalid binary speed packet format:");
    ESP_LOGW(TAG, "  Expected: 0x56 ... 0x0D 0x0A");
    ESP_LOGW(TAG, "  Got:      0x%02X ... 0x%02X 0x%02X", data[0], data[7], data[8]);
    return;
  }
  
  // Extract sign (+ = 0x2B, - = 0x2D)
  bool is_approaching = (data[1] == 0x2B);
  ESP_LOGI(TAG, "Direction byte: 0x%02X (%c) -> %s", data[1], (char)data[1], is_approaching ? "approaching" : "departing");
  
  // Extract speed digits (ASCII numbers)
  if (data[2] < 0x30 || data[2] > 0x39 ||  // hundreds
      data[3] < 0x30 || data[3] > 0x39 ||  // tens
      data[4] < 0x30 || data[4] > 0x39 ||  // units
      data[5] != 0x2E ||                   // decimal point
      data[6] < 0x30 || data[6] > 0x39) {  // decimal
    ESP_LOGW(TAG, "Invalid binary speed format:");
    ESP_LOGW(TAG, "  Bytes 2-6: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", data[2], data[3], data[4], data[5], data[6]);
    ESP_LOGW(TAG, "  Expected:  0x30-0x39 0x30-0x39 0x30-0x39 0x2E 0x30-0x39");
    return;
  }
  
  // Convert to speed value
  float speed = (data[2] - 0x30) * 100.0f +  // hundreds
                (data[3] - 0x30) * 10.0f +   // tens
                (data[4] - 0x30) * 1.0f +    // units
                (data[6] - 0x30) * 0.1f;     // decimal
  
  this->approaching_ = is_approaching;
  this->velocity_ = is_approaching ? speed : -speed;
  this->speed_ = std::abs(speed);
  
  // Apply Kalman filtering for noise reduction
  this->speed_ = this->apply_kalman_filter_(this->speed_);
  
  ESP_LOGI(TAG, "Binary speed parsed successfully:");
  ESP_LOGI(TAG, "  Speed: %.3f km/h", speed);
  ESP_LOGI(TAG, "  Direction: %s", is_approaching ? "approaching" : "departing");
  ESP_LOGI(TAG, "  Velocity: %.3f km/h", this->velocity_);

  // Publish sensor data
  if (this->speed_sensor_ != nullptr) {
    ESP_LOGI(TAG, "Publishing to speed sensor: %.3f km/h", this->speed_);
    this->publish_sensor_state_(this->speed_sensor_, this->speed_, "Speed Sensor (Binary)");
  }

  if (this->approaching_) {
    if (this->approaching_speed_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to approaching speed sensor: %.3f km/h", this->speed_);
      this->publish_sensor_state_(this->approaching_speed_sensor_, this->speed_, "Approaching Speed Sensor (Binary)");
      this->last_approaching_update_time_ = millis();
    }
  } else {
    if (this->departing_speed_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to departing speed sensor: %.3f km/h", this->speed_);
      this->publish_sensor_state_(this->departing_speed_sensor_, this->speed_, "Departing Speed Sensor (Binary)");
      this->last_departing_update_time_ = millis();
    }
  }

  if (this->velocity_sensor_ != nullptr) {
    ESP_LOGI(TAG, "Publishing to velocity sensor: %.3f", this->velocity_);
    this->publish_sensor_state_(this->velocity_sensor_, this->velocity_, "Velocity Sensor (Binary)");
  }

  // Process vehicle tracking
  ESP_LOGI(TAG, "Processing vehicle detection with speed=%.3f, approaching=%s", this->speed_, this->approaching_ ? "yes" : "no");
  this->process_vehicle_detection_(this->speed_, this->approaching_);
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
  
  // Look for existing active vehicle in the same direction
  for (auto& vehicle : vehicles) {
    if (!vehicle.is_active) continue;
    
    // Check if this could be the same vehicle (speed difference within threshold)
    double speed_diff = std::abs(vehicle.last_speed - speed);
    uint32_t time_diff = current_time - vehicle.last_detection_time;
    
    // If speed jump is too large or too much time has passed, this is likely a new vehicle
    if (speed_diff > SPEED_JUMP_THRESHOLD || time_diff > VEHICLE_TIMEOUT) {
      // Mark current vehicle as finished and publish final data
      this->publish_vehicle_data_(vehicle, true);
      vehicle.is_active = false;
      continue;
    }
    
    // This appears to be the same vehicle
    return &vehicle;
  }
  
  // No matching vehicle found, create a new one
  Vehicle new_vehicle;
  new_vehicle.id = next_vehicle_id_++;
  new_vehicle.is_approaching = is_approaching;
  new_vehicle.is_active = true;
  new_vehicle.first_detection_time = current_time;
  
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
  
  ESP_LOGD(TAG, "New vehicle detected: ID %u, direction: %s", 
           new_vehicle.id, is_approaching ? "approaching" : "departing");
  
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

}  // namespace ld2415h
}  // namespace esphome
