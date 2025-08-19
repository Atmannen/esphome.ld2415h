#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#include <map>
#include <vector>

namespace esphome {
namespace ld2415h {

enum NegotiationMode : uint8_t { CUSTOM_AGREEMENT = 0x01, STANDARD_PROTOCOL = 0x02 };

enum SampleRateStructure : uint8_t { SAMPLE_RATE_22FPS = 0x00, SAMPLE_RATE_11FPS = 0x01, SAMPLE_RATE_6FPS = 0x02 };

static const std::map<std::string, uint8_t> SAMPLE_RATE_STR_TO_INT{
    {"~22 fps", SAMPLE_RATE_22FPS}, {"~11 fps", SAMPLE_RATE_11FPS}, {"~6 fps", SAMPLE_RATE_6FPS}};

enum TrackingMode : uint8_t { APPROACHING_AND_RETREATING = 0x00, APPROACHING = 0x01, RETREATING = 0x02 };

static const std::map<std::string, uint8_t> TRACKING_MODE_STR_TO_INT{
    {"Approaching and Restreating", APPROACHING_AND_RETREATING},
    {"Approaching", APPROACHING},
    {"Restreating", RETREATING}};

enum UnitOfMeasure : uint8_t { KPH = 0x00, MPH = 0x01, MPS = 0x02 };

// Vehicle tracking structure
struct Vehicle {
  uint32_t id;
  double max_speed;
  double last_speed;
  uint32_t first_detection_time;
  uint32_t last_detection_time;
  bool is_approaching;
  bool is_active;
  
  Vehicle() : id(0), max_speed(0), last_speed(0), first_detection_time(0), 
              last_detection_time(0), is_approaching(false), is_active(false) {}
};

class LD2415HListener {
 public:
  virtual void on_speed(double speed){};
  virtual void on_velocity(double velocity){};
  virtual void on_approach(bool approaching){};
};

class LD2415HComponent : public Component, public uart::UARTDevice {
 public:
  // Constructor declaration
  LD2415HComponent();
  void setup() override;
  void dump_config() override;
  void loop() override;

#ifdef USE_NUMBER
  void update();
  void set_min_speed_threshold_number(number::Number *number) { this->min_speed_threshold_number_ = number; };
  void set_compensation_angle_number(number::Number *number) { this->compensation_angle_number_ = number; };
  void set_sensitivity_number(number::Number *number) { this->sensitivity_number_ = number; };
  void set_vibration_correction_number(number::Number *number) { this->vibration_correction_number_ = number; };
  void set_relay_trigger_duration_number(number::Number *number) { this->relay_trigger_duration_number_ = number; };
  void set_relay_trigger_speed_number(number::Number *number) { this->relay_trigger_speed_number_ = number; };
  void set_timeout_duration_number(number::Number *n) { this->timeout_duration_number_ = n; }
#endif
#ifdef USE_SELECT
  void set_sample_rate_select(select::Select *selector) { this->sample_rate_selector_ = selector; };
  void set_tracking_mode_select(select::Select *selector) { this->tracking_mode_selector_ = selector; };
#endif
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  //void register_listener(LD2415HListener *listener) { this->listeners_.push_back(listener); }
  void set_speed_sensor(sensor::Sensor *sensor) { this->speed_sensor_ = sensor; };
  void set_approaching_speed_sensor(sensor::Sensor *sensor) { this->approaching_speed_sensor_ = sensor; };
  void set_departing_speed_sensor(sensor::Sensor *sensor) { this->departing_speed_sensor_ = sensor; };
  void set_approaching_last_max_speed_sensor(sensor::Sensor *sensor) {
    this->approaching_last_max_speed_sensor_ = sensor;
  };
  void set_departing_last_max_speed_sensor(sensor::Sensor *sensor) { this->departing_last_max_speed_sensor_ = sensor; };
  void set_velocity_sensor(sensor::Sensor *sensor) { this->velocity_sensor_ = sensor; };
  
  // Vehicle tracking sensors
  void set_approaching_vehicle_count_sensor(sensor::Sensor *sensor) { this->approaching_vehicle_count_sensor_ = sensor; };
  void set_departing_vehicle_count_sensor(sensor::Sensor *sensor) { this->departing_vehicle_count_sensor_ = sensor; };
  void set_current_approaching_vehicle_speed_sensor(sensor::Sensor *sensor) { this->current_approaching_vehicle_speed_sensor_ = sensor; };
  void set_current_departing_vehicle_speed_sensor(sensor::Sensor *sensor) { this->current_departing_vehicle_speed_sensor_ = sensor; };

  void set_min_speed_threshold(uint8_t speed);
  void set_compensation_angle(uint8_t angle);
  void set_sensitivity(uint8_t sensitivity);
  void set_tracking_mode(const std::string &state);
  void set_tracking_mode(TrackingMode mode);
  void set_tracking_mode(uint8_t mode);
  void set_sample_rate(const std::string &state);
  void set_sample_rate(uint8_t rate);
  void set_vibration_correction(uint8_t correction);
  void set_relay_trigger_duration(uint8_t duration);
  void set_relay_trigger_speed(uint8_t speed);
  void set_timeout_duration(uint32_t duration);
  void set_binary_protocol(bool binary_mode) { this->is_binary_protocol_ = binary_mode; }

#ifdef USE_NUMBER
  number::Number *min_speed_threshold_number_{nullptr};
  number::Number *compensation_angle_number_{nullptr};
  number::Number *sensitivity_number_{nullptr};
  number::Number *vibration_correction_number_{nullptr};
  number::Number *relay_trigger_duration_number_{nullptr};
  number::Number *relay_trigger_speed_number_{nullptr};
  number::Number *timeout_duration_number_{nullptr};
#endif
#ifdef USE_SELECT
  select::Select *sample_rate_selector_{nullptr};
  select::Select *tracking_mode_selector_{nullptr};
#endif

 protected:
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *approaching_speed_sensor_{nullptr};
  sensor::Sensor *departing_speed_sensor_{nullptr};
  sensor::Sensor *approaching_last_max_speed_sensor_{nullptr};
  sensor::Sensor *departing_last_max_speed_sensor_{nullptr};
  sensor::Sensor *velocity_sensor_{nullptr};
  
  // Vehicle tracking sensors
  sensor::Sensor *approaching_vehicle_count_sensor_{nullptr};
  sensor::Sensor *departing_vehicle_count_sensor_{nullptr};
  sensor::Sensor *current_approaching_vehicle_speed_sensor_{nullptr};
  sensor::Sensor *current_departing_vehicle_speed_sensor_{nullptr};

  // Configuration
  uint8_t min_speed_threshold_ = 0;
  uint8_t compensation_angle_ = 0;
  uint8_t sensitivity_ = 0;
  TrackingMode tracking_mode_ = TrackingMode::APPROACHING_AND_RETREATING;
  uint8_t sample_rate_ = 0;
  UnitOfMeasure unit_of_measure_ = UnitOfMeasure::KPH;
  uint8_t vibration_correction_ = 0;
  uint8_t relay_trigger_duration_ = 0;
  uint8_t relay_trigger_speed_ = 0;
  uint32_t timeout_duration_ = 400;  // Timeout duration in milliseconds
  NegotiationMode negotiation_mode_ = NegotiationMode::CUSTOM_AGREEMENT;

  // State
  uint8_t cmd_speed_angle_sense_[8];
  uint8_t cmd_mode_rate_uom_[8];
  uint8_t cmd_anti_vib_comp_[8];
  uint8_t cmd_relay_duration_speed_[8];
  uint8_t cmd_config_[13];

  bool update_speed_angle_sense_ = false;
  bool update_mode_rate_uom_ = false;
  bool update_anti_vib_comp_ = false;
  bool update_relay_duration_speed_ = false;
  bool update_config_ = false;

  char firmware_[20] = "";
  double speed_ = 0;
  double velocity_ = 0;
  bool approaching_ = false;
  double last_max_approaching_speed_ = 0;
  double last_max_departing_speed_ = 0;
  uint32_t last_approaching_update_time_ = 0;  // Time of the last update in milliseconds for approaching speed
  uint32_t last_departing_update_time_ = 0;   // Time of the last update in milliseconds for departing speed
  char response_buffer_[64];
  uint8_t response_buffer_index_ = 0;
  
  // Binary protocol support
  bool is_binary_protocol_{false};
  std::vector<uint8_t> binary_buffer_;
  
  // Heartbeat for debugging
  uint32_t last_heartbeat_time_{0};
  uint32_t last_data_received_time_{0};
  
  // Vehicle tracking
  std::vector<Vehicle> approaching_vehicles_;
  std::vector<Vehicle> departing_vehicles_;
  uint32_t next_vehicle_id_{1};
  uint32_t approaching_vehicle_count_{0};
  uint32_t departing_vehicle_count_{0};
  
  // Vehicle detection thresholds
  static constexpr double SPEED_JUMP_THRESHOLD = 20.0;  // km/h difference to detect new vehicle
  static constexpr uint32_t VEHICLE_TIMEOUT = 3000;     // ms without detection before vehicle is considered gone
  static constexpr double MIN_DETECTION_SPEED = 5.0;    // km/h minimum speed to consider as vehicle

  // Processing
  void issue_command_(const uint8_t cmd[], uint8_t size);
  bool fill_buffer_(char c);
  void clear_remaining_buffer_(uint8_t pos);
  void parse_buffer_();
  void parse_config_();
  void parse_firmware_();
  void parse_speed_();
  void parse_binary_speed_(const std::vector<uint8_t> &data);
  bool parse_hex_speed_packet_(const std::vector<uint8_t> &data);
  void parse_config_param_(char *key, char *value);
  
  // Sensor publishing with detailed logging
  bool publish_sensor_state_(sensor::Sensor *sensor, float value, const char *sensor_name);
  
  // Vehicle tracking methods
  void process_vehicle_detection_(double speed, bool is_approaching);
  Vehicle* find_or_create_vehicle_(double speed, bool is_approaching);
  void update_vehicle_(Vehicle* vehicle, double speed, uint32_t current_time);
  void cleanup_inactive_vehicles_();
  void publish_vehicle_data_(const Vehicle& vehicle, bool is_final);

  // Helpers
  TrackingMode i_to_tracking_mode_(uint8_t value);
  UnitOfMeasure i_to_unit_of_measure_(uint8_t value);
  NegotiationMode i_to_negotiation_mode_(uint8_t value);
  const char *tracking_mode_to_s_(TrackingMode value);
  const char *unit_of_measure_to_s_(UnitOfMeasure value);
  const char *negotiation_mode_to_s_(NegotiationMode value);
  const char *i_to_s_(const std::map<std::string, uint8_t> &map, uint8_t i);

  //std::vector<LD2415HListener *> listeners_{};
};

}  // namespace ld2415h
}  // namespace esphome
