#pragma once

#include "esphome/core/component.h"
#include "esphome/core/esphal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/pulse_counter/pulse_counter_sensor.h"

namespace esphome {
namespace hlw8012 {

enum HLW8012InitialMode { HLW8012_INITIAL_MODE_CURRENT = 0, HLW8012_INITIAL_MODE_VOLTAGE };

class HLW8012Component : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_initial_mode(HLW8012InitialMode initial_mode) {
    current_mode_ = initial_mode == HLW8012_INITIAL_MODE_CURRENT;
  }
  //void set_sensor_model(HLW8012SensorModels sensor_model) { sensor_model_ = sensor_model; }
  void set_change_mode_every(uint32_t change_mode_every) { change_mode_every_ = change_mode_every; }
  void set_voltage_constant(uint16_t voltage_constant) { voltage_constant_ = voltage_constant; }
  void set_current_constant(uint16_t current_constant) { current_constant_ = current_constant; }
  void set_power_constant  (uint16_t power_constant)   { power_constant_ = power_constant; }
  void set_sel_pin(GPIOPin *sel_pin) { sel_pin_ = sel_pin; }
  void set_cf_pin(GPIOPin *cf_pin) { cf_pin_ = cf_pin; }
  void set_cf1_pin(GPIOPin *cf1_pin) { cf1_pin_ = cf1_pin; }
  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_current_sensor(sensor::Sensor *current_sensor) { current_sensor_ = current_sensor; }
  void set_power_sensor(sensor::Sensor *power_sensor) { power_sensor_ = power_sensor; }
  void set_energy_sensor(sensor::Sensor *energy_sensor) { energy_sensor_ = energy_sensor; }

 protected:
  uint32_t nth_value_{0};
  bool current_mode_{false};
  uint32_t change_mode_at_{0};
  uint32_t change_mode_every_{8};

  uint16_t voltage_constant_{1000};
  uint16_t current_constant_{1000};
  uint16_t power_constant_  {1000};

  uint64_t cf_total_pulses_{0};
  GPIOPin *sel_pin_;
  GPIOPin *cf_pin_;
  pulse_counter::PulseCounterStorage cf_store_;
  GPIOPin *cf1_pin_;
  pulse_counter::PulseCounterStorage cf1_store_;
  
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_  {nullptr};
  sensor::Sensor *energy_sensor_ {nullptr};

  float voltage_multiplier_{0.0f};
  float current_multiplier_{0.0f};
  float power_multiplier_  {0.0f};
};

}  // namespace hlw8012
}  // namespace esphome
