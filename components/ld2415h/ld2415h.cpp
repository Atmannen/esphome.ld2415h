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
  // Process the stream from the sensor UART
  const uint32_t now = millis();
  
  // Heartbeat logging every 30 seconds to show component is alive
  if (now - this->last_heartbeat_time_ > 30000) {
    ESP_LOGI(TAG, "LD2415H heartbeat - component is alive and listening for data");
    if (this->last_data_received_time_ > 0) {
      ESP_LOGI(TAG, "  Last data received: %u ms ago", now - this->last_data_received_time_);
    } else {
      ESP_LOGI(TAG, "  No data received since startup");
    }
    this->last_heartbeat_time_ = now;
  }
  
  if (available()) {
    // Update last data received time
    this->last_data_received_time_ = now;
    
    std::vector<uint8_t> raw_data;
    
    // Read all available data
    while (available()) {
      uint8_t byte;
      if (read_byte(&byte)) {
        raw_data.push_back(byte);
        binary_buffer_.push_back(byte);
      }
    }
    
    // Log raw data received for debugging
    if (!raw_data.empty()) {
      ESP_LOGV(TAG, "Raw data received (%zu bytes): ", raw_data.size());
      std::string hex_dump;
      std::string ascii_dump;
      for (size_t i = 0; i < raw_data.size(); i++) {
        char hex_str[4];
        snprintf(hex_str, sizeof(hex_str), "%02X ", raw_data[i]);
        hex_dump += hex_str;
        
        // ASCII representation
        char c = (raw_data[i] >= 32 && raw_data[i] < 127) ? (char)raw_data[i] : '.';
        ascii_dump += c;
        
        if ((i + 1) % 16 == 0 || i == raw_data.size() - 1) {
          ESP_LOGV(TAG, "  %s | %s", hex_dump.c_str(), ascii_dump.c_str());
          hex_dump.clear();
          ascii_dump.clear();
        }
      }
    }
    
    // Try to detect protocol format
    std::string ascii_data(raw_data.begin(), raw_data.end());
    
    // Check for ASCII format (starts with 'V')
    if (!ascii_data.empty() && ascii_data[0] == 'V') {
      is_binary_protocol_ = false;
      ESP_LOGI(TAG, "Detected ASCII protocol packet: '%s'", ascii_data.c_str());
      
      // Process ASCII data using existing method
      for (char c : ascii_data) {
        if (this->fill_buffer_(c)) {
          ESP_LOGI(TAG, "ASCII buffer full, parsing...");
          this->parse_buffer_();
        }
      }
      binary_buffer_.clear();
    } else {
      // Check for binary protocol (9 bytes ending with 0x0D 0x0A)
      is_binary_protocol_ = true;
      ESP_LOGD(TAG, "Processing binary protocol data (buffer size: %zu)", binary_buffer_.size());
      
      if (binary_buffer_.size() >= 9) {
        // Look for complete 9-byte packets ending with 0x0D 0x0A
        bool packet_found = false;
        for (size_t i = 0; i <= binary_buffer_.size() - 9; i++) {
          if (binary_buffer_[i + 7] == 0x0D && binary_buffer_[i + 8] == 0x0A) {
            std::vector<uint8_t> packet(binary_buffer_.begin() + i, binary_buffer_.begin() + i + 9);
            
            // Log binary packet found
            ESP_LOGI(TAG, "Found binary packet at offset %zu:", i);
            std::string packet_hex;
            for (uint8_t b : packet) {
              char hex_str[4];
              snprintf(hex_str, sizeof(hex_str), "%02X ", b);
              packet_hex += hex_str;
            }
            ESP_LOGI(TAG, "  Binary packet: %s", packet_hex.c_str());
            
            parse_binary_speed_(packet);
            binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + i + 9);
            packet_found = true;
            break;
          }
        }
        
        if (!packet_found) {
          ESP_LOGW(TAG, "No valid binary packet found in buffer (size: %zu)", binary_buffer_.size());
          // Log current buffer contents for debugging
          std::string buffer_hex;
          for (size_t i = 0; i < std::min(binary_buffer_.size(), size_t(20)); i++) {
            char hex_str[4];
            snprintf(hex_str, sizeof(hex_str), "%02X ", binary_buffer_[i]);
            buffer_hex += hex_str;
          }
          ESP_LOGW(TAG, "  Buffer contents: %s%s", buffer_hex.c_str(), binary_buffer_.size() > 20 ? "..." : "");
        }
        
        // Keep buffer size manageable
        if (binary_buffer_.size() > 50) {
          ESP_LOGW(TAG, "Binary buffer growing too large (%zu bytes), trimming...", binary_buffer_.size());
          binary_buffer_.erase(binary_buffer_.begin(), binary_buffer_.begin() + 25);
        }
      } else {
        ESP_LOGV(TAG, "Binary buffer too small (%zu bytes), waiting for more data", binary_buffer_.size());
      }
    }
  }

  // Timeout handling for last max speed
  // Approaching speed
  if (this->approaching_last_max_speed_sensor_ != nullptr && last_max_approaching_speed_ > 0 && now - this->last_approaching_update_time_ > this->timeout_duration_) {
    ESP_LOGI(TAG, "Timeout reached for approaching max speed: publishing %.3f km/h after %u ms", 
             last_max_approaching_speed_, now - this->last_approaching_update_time_);
    this->publish_sensor_state_(this->approaching_last_max_speed_sensor_, last_max_approaching_speed_, "Approaching Last Max Speed Sensor (Timeout)");
    this->last_approaching_update_time_ = now;
    this->last_max_approaching_speed_ = 0;
  }

  // Departing speed
  if (this->departing_last_max_speed_sensor_ != nullptr && last_max_departing_speed_ > 0 && now - this->last_departing_update_time_ > this->timeout_duration_) {
    ESP_LOGI(TAG, "Timeout reached for departing max speed: publishing %.3f km/h after %u ms", 
             last_max_departing_speed_, now - this->last_departing_update_time_);
    this->publish_sensor_state_(this->departing_last_max_speed_sensor_, last_max_departing_speed_, "Departing Last Max Speed Sensor (Timeout)");
    this->last_departing_update_time_ = now;
    this->last_max_departing_speed_ = 0;
  }
  
  // Clean up inactive vehicles
  this->cleanup_inactive_vehicles_();
}

bool LD2415HComponent::publish_sensor_state_(sensor::Sensor *sensor, float value, const char *sensor_name) {
  if (sensor == nullptr) {
    ESP_LOGW(TAG, "Cannot publish to %s: sensor is NULL", sensor_name);
    return false;
  }
  
  // Check if value is valid
  if (std::isnan(value)) {
    ESP_LOGW(TAG, "Cannot publish to %s: value is NaN", sensor_name);
    return false;
  }
  
  if (std::isinf(value)) {
    ESP_LOGW(TAG, "Cannot publish to %s: value is infinite", sensor_name);
    return false;
  }
  
  // Get current state for comparison
  float current_state = sensor->get_state();
  bool has_state = sensor->has_state();
  
  ESP_LOGI(TAG, "Publishing to %s:", sensor_name);
  ESP_LOGI(TAG, "  Sensor pointer: %p", (void*)sensor);
  ESP_LOGI(TAG, "  Current state: %s%s", has_state ? std::to_string(current_state).c_str() : "no state", has_state ? "" : " (first time)");
  ESP_LOGI(TAG, "  New value: %.3f", value);
  ESP_LOGI(TAG, "  Entity ID: %s", sensor->get_object_id().c_str());
  ESP_LOGI(TAG, "  Entity Name: %s", sensor->get_name().c_str());
  
  // Check if sensor is ready/connected
  if (!sensor->is_internal() && sensor->get_object_id().empty()) {
    ESP_LOGW(TAG, "Sensor %s has empty object_id - may not be properly configured", sensor_name);
  }
  
  // Publish the state
  sensor->publish_state(value);
  ESP_LOGI(TAG, "✓ Successfully published %.3f to %s", value, sensor_name);
  
  // Verify the state was set
  if (sensor->has_state() && std::abs(sensor->get_state() - value) < 0.001) {
    ESP_LOGI(TAG, "✓ Sensor state confirmed: %.3f", sensor->get_state());
  } else {
    ESP_LOGW(TAG, "⚠ Sensor state mismatch after publish: expected %.3f, got %.3f", value, sensor->get_state());
  }
  
  return true;
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
  // Example: "V+001.9"
  ESP_LOGI(TAG, "Parsing ASCII speed from: '%s'", this->response_buffer_);

  const char *p = strchr(this->response_buffer_, 'V');

  if (p != nullptr) {
    ++p;
    this->approaching_ = (*p == '+');
    this->velocity_ = strtod(p, nullptr);
    ++p;
    this->speed_ = strtod(p, nullptr);
    
    ESP_LOGI(TAG, "ASCII speed parsed successfully:");
    ESP_LOGI(TAG, "  Direction char: '%c'", *(p-1));
    ESP_LOGI(TAG, "  Approaching: %s", this->approaching_ ? "yes" : "no");
    ESP_LOGI(TAG, "  Velocity: %.3f", this->velocity_);
    ESP_LOGI(TAG, "  Speed: %.3f km/h", this->speed_);

    ESP_LOGV(TAG, "Speed updated: %f KPH", this->speed_);

/*    for (auto &listener : this->listeners_) {
      listener->on_speed(this->speed_);
      listener->on_velocity(this->velocity_);
      listener->on_approach(this->approaching_);
    }
*/
    if (this->speed_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to speed sensor: %.3f km/h", this->speed_);
      this->publish_sensor_state_(this->speed_sensor_, this->speed_, "Speed Sensor");
    } else {
      ESP_LOGW(TAG, "Speed sensor is null, cannot publish");
    }

    if (this->approaching_) {
      if (this->approaching_speed_sensor_ != nullptr) {
        ESP_LOGI(TAG, "Publishing to approaching speed sensor: %.3f km/h", this->speed_);
        this->publish_sensor_state_(this->approaching_speed_sensor_, this->speed_, "Approaching Speed Sensor");
        this->last_approaching_update_time_ = millis();
      
        // Update last max speed
        if (this->speed_ > this->last_max_approaching_speed_) {
          ESP_LOGI(TAG, "New max approaching speed: %.3f km/h (was %.3f)", this->speed_, this->last_max_approaching_speed_);
          this->last_max_approaching_speed_ = this->speed_;
        }
      } else {
        ESP_LOGW(TAG, "Approaching speed sensor is null, cannot publish");
      }
    } else {
      // Handle departing speed
      if (this->departing_speed_sensor_ != nullptr) {
        ESP_LOGI(TAG, "Publishing to departing speed sensor: %.3f km/h", this->speed_);
        this->publish_sensor_state_(this->departing_speed_sensor_, this->speed_, "Departing Speed Sensor");
        this->last_departing_update_time_ = millis();
      
        // Update last max speed
        if (this->speed_ > this->last_max_departing_speed_) {
          ESP_LOGI(TAG, "New max departing speed: %.3f km/h (was %.3f)", this->speed_, this->last_max_departing_speed_);
          this->last_max_departing_speed_ = this->speed_;
        }
      } else {
        ESP_LOGW(TAG, "Departing speed sensor is null, cannot publish");
      }
    }
  
    if (this->velocity_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to velocity sensor: %.3f", this->velocity_);
      this->publish_sensor_state_(this->velocity_sensor_, this->velocity_, "Velocity Sensor");
    } else {
      ESP_LOGW(TAG, "Velocity sensor is null, cannot publish");
    }
    
    // Process vehicle tracking
    ESP_LOGI(TAG, "Processing vehicle detection with speed=%.3f, approaching=%s", this->speed_, this->approaching_ ? "yes" : "no");
    this->process_vehicle_detection_(this->speed_, this->approaching_);

  } else {
    ESP_LOGE(TAG, "Failed to find 'V' in ASCII response: '%s'", this->response_buffer_);
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
  
  ESP_LOGI(TAG, "Binary speed calculation:");
  ESP_LOGI(TAG, "  Hundreds: %c (0x%02X) = %d", (char)data[2], data[2], data[2] - 0x30);
  ESP_LOGI(TAG, "  Tens:     %c (0x%02X) = %d", (char)data[3], data[3], data[3] - 0x30);
  ESP_LOGI(TAG, "  Units:    %c (0x%02X) = %d", (char)data[4], data[4], data[4] - 0x30);
  ESP_LOGI(TAG, "  Decimal:  %c (0x%02X) = %d", (char)data[6], data[6], data[6] - 0x30);
  ESP_LOGI(TAG, "  Result:   %.3f km/h", speed);
  
  this->approaching_ = is_approaching;
  this->velocity_ = is_approaching ? speed : -speed;
  this->speed_ = speed;
  
  ESP_LOGI(TAG, "Binary speed parsed successfully:");
  ESP_LOGI(TAG, "  Speed: %.3f km/h", speed);
  ESP_LOGI(TAG, "  Approaching: %s", is_approaching ? "yes" : "no");
  ESP_LOGI(TAG, "  Velocity: %.3f", this->velocity_);
  
  // Update sensors with the parsed speed (same logic as ASCII parsing)
  if (this->speed_sensor_ != nullptr) {
    ESP_LOGI(TAG, "Publishing to speed sensor: %.3f km/h", this->speed_);
    this->publish_sensor_state_(this->speed_sensor_, this->speed_, "Speed Sensor (Binary)");
  } else {
    ESP_LOGW(TAG, "Speed sensor is null, cannot publish");
  }

  if (this->approaching_) {
    if (this->approaching_speed_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to approaching speed sensor: %.3f km/h", this->speed_);
      this->publish_sensor_state_(this->approaching_speed_sensor_, this->speed_, "Approaching Speed Sensor (Binary)");
      this->last_approaching_update_time_ = millis();
    
      // Update last max speed
      if (this->speed_ > this->last_max_approaching_speed_) {
        ESP_LOGI(TAG, "New max approaching speed: %.3f km/h (was %.3f)", this->speed_, this->last_max_approaching_speed_);
        this->last_max_approaching_speed_ = this->speed_;
      }
    } else {
      ESP_LOGW(TAG, "Approaching speed sensor is null, cannot publish");
    }
  } else {
    // Handle departing speed
    if (this->departing_speed_sensor_ != nullptr) {
      ESP_LOGI(TAG, "Publishing to departing speed sensor: %.3f km/h", this->speed_);
      this->publish_sensor_state_(this->departing_speed_sensor_, this->speed_, "Departing Speed Sensor (Binary)");
      this->last_departing_update_time_ = millis();
    
      // Update last max speed
      if (this->speed_ > this->last_max_departing_speed_) {
        ESP_LOGI(TAG, "New max departing speed: %.3f km/h (was %.3f)", this->speed_, this->last_max_departing_speed_);
        this->last_max_departing_speed_ = this->speed_;
      }
    } else {
      ESP_LOGW(TAG, "Departing speed sensor is null, cannot publish");
    }
  }

  if (this->velocity_sensor_ != nullptr) {
    ESP_LOGI(TAG, "Publishing to velocity sensor: %.3f", this->velocity_);
    this->publish_sensor_state_(this->velocity_sensor_, this->velocity_, "Velocity Sensor (Binary)");
  } else {
    ESP_LOGW(TAG, "Velocity sensor is null, cannot publish");
  }
  
  // Process vehicle tracking
  ESP_LOGI(TAG, "Processing vehicle detection with speed=%.3f, approaching=%s", this->speed_, this->approaching_ ? "yes" : "no");
  this->process_vehicle_detection_(this->speed_, this->approaching_);
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
  ESP_LOGI(TAG, "Processing vehicle detection: speed=%.3f km/h, approaching=%s, threshold=%.1f", 
           speed, is_approaching ? "yes" : "no", MIN_DETECTION_SPEED);
  
  // Only process if speed is above minimum threshold
  if (speed < MIN_DETECTION_SPEED) {
    ESP_LOGD(TAG, "Speed %.3f below minimum threshold %.1f, ignoring", speed, MIN_DETECTION_SPEED);
    return;
  }
  
  Vehicle* vehicle = this->find_or_create_vehicle_(speed, is_approaching);
  if (vehicle != nullptr) {
    ESP_LOGI(TAG, "Found/created vehicle ID %u, updating with speed %.3f", vehicle->id, speed);
    this->update_vehicle_(vehicle, speed, millis());
  } else {
    ESP_LOGW(TAG, "Failed to find or create vehicle for speed %.3f", speed);
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
      ESP_LOGD(TAG, "Vehicle %u finished with max speed %.1f km/h", vehicle.id, vehicle.max_speed);
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
    ESP_LOGV(TAG, "Vehicle %u new max speed: %.1f km/h", vehicle->id, speed);
  }
  
  // Publish current vehicle data
  this->publish_vehicle_data_(*vehicle, false);
}

void LD2415HComponent::cleanup_inactive_vehicles_() {
  uint32_t current_time = millis();
  
  // Clean up approaching vehicles
  for (auto it = approaching_vehicles_.begin(); it != approaching_vehicles_.end();) {
    if (it->is_active && (current_time - it->last_detection_time > VEHICLE_TIMEOUT)) {
      ESP_LOGD(TAG, "Vehicle %u timed out with max speed %.1f km/h", it->id, it->max_speed);
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
      ESP_LOGD(TAG, "Vehicle %u timed out with max speed %.1f km/h", it->id, it->max_speed);
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
        // Publish final max speed
        ESP_LOGI(TAG, "Publishing final approaching vehicle data: ID=%u, max_speed=%.3f", vehicle.id, vehicle.max_speed);
        this->publish_sensor_state_(this->current_approaching_vehicle_speed_sensor_, vehicle.max_speed, "Current Approaching Vehicle Speed (Final)");
      } else {
        // Publish current speed
        ESP_LOGI(TAG, "Publishing current approaching vehicle data: ID=%u, current_speed=%.3f", vehicle.id, vehicle.last_speed);
        this->publish_sensor_state_(this->current_approaching_vehicle_speed_sensor_, vehicle.last_speed, "Current Approaching Vehicle Speed");
      }
    } else {
      ESP_LOGW(TAG, "Current approaching vehicle speed sensor is null, cannot publish vehicle data");
    }
  } else {
    if (this->current_departing_vehicle_speed_sensor_ != nullptr) {
      if (is_final) {
        // Publish final max speed
        ESP_LOGI(TAG, "Publishing final departing vehicle data: ID=%u, max_speed=%.3f", vehicle.id, vehicle.max_speed);
        this->publish_sensor_state_(this->current_departing_vehicle_speed_sensor_, vehicle.max_speed, "Current Departing Vehicle Speed (Final)");
      } else {
        // Publish current speed
        ESP_LOGI(TAG, "Publishing current departing vehicle data: ID=%u, current_speed=%.3f", vehicle.id, vehicle.last_speed);
        this->publish_sensor_state_(this->current_departing_vehicle_speed_sensor_, vehicle.last_speed, "Current Departing Vehicle Speed");
      }
    } else {
      ESP_LOGW(TAG, "Current departing vehicle speed sensor is null, cannot publish vehicle data");
    }
  }
}

}  // namespace ld2415h
}  // namespace esphome
