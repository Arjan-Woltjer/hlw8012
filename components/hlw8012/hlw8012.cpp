#include "hlw8012.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hlw8012 {

static const char *const TAG = "hlw8012";

void HLW8012Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up HLW8012...");
  this->sel_pin_->setup();
  this->sel_pin_->digital_write(this->current_mode_);
  this->cf_store_.pulse_counter_setup(this->cf_pin_);
  this->cf1_store_.pulse_counter_setup(this->cf1_pin_);

  // Initialize multipliers
  this->voltage_multiplier_ = float(this->voltage_constant_) / 10000;
  this->current_multiplier_ = float(this->current_constant_) / 100000;
  this->power_multiplier_   = float(this->power_constant_)   / 1000;
}

void HLW8012Component::dump_config() {
  ESP_LOGCONFIG(TAG, "HLW8012:");
  LOG_PIN("  SEL Pin: ", this->sel_pin_)
  LOG_PIN("  CF  Pin: ", this->cf_pin_)
  LOG_PIN("  CF1 Pin: ", this->cf1_pin_)
  ESP_LOGCONFIG(TAG, "  Change measurement mode every %u", this->change_mode_every_);
  ESP_LOGCONFIG(TAG, "  Voltage constant: %u V /10000  Hz", this->voltage_constant_);
  ESP_LOGCONFIG(TAG, "  Current constant: %u mA/100000 Hz", this->current_constant_);
  ESP_LOGCONFIG(TAG, "  Power   constant: %u W /1000   Hz", this->power_constant_);
  LOG_UPDATE_INTERVAL(this)
  LOG_SENSOR("  ", "Voltage", this->voltage_sensor_)
  LOG_SENSOR("  ", "Current", this->current_sensor_)
  LOG_SENSOR("  ", "Power",   this->power_sensor_)
  LOG_SENSOR("  ", "Energy",  this->energy_sensor_)
}
float HLW8012Component::get_setup_priority() const { return setup_priority::DATA; }
void HLW8012Component::update() {
  // HLW8012 has 50% duty cycle
  pulse_counter::pulse_counter_t raw_cf = this->cf_store_.read_raw_value();
  pulse_counter::pulse_counter_t raw_cf1 = this->cf1_store_.read_raw_value();
  float cf_hz = raw_cf / (this->get_update_interval() / 1000.0f);
  if (raw_cf <= 1) {
    // don't count single pulse as power
    cf_hz = 0.0f;
  }
  float cf1_hz = raw_cf1 / (this->get_update_interval() / 1000.0f);
  if (raw_cf1 <= 1) {
    // don't count single pulse as anything
    cf1_hz = 0.0f;
  }

  if (this->nth_value_++ < 2) {
    return;
  }

  float power = cf_hz * this->power_multiplier_;

  if (this->change_mode_at_ != 0) {
    // Only read cf1 after one cycle. Apparently it's quite unstable after being changed.
    if (this->current_mode_) {
      float current = cf1_hz * this->current_multiplier_;
      ESP_LOGD(TAG, "Got power=%.1fW, current=%.1fA", power, current);
      if (this->current_sensor_ != nullptr) {
        this->current_sensor_->publish_state(current);
      }
    } else {
      float voltage = cf1_hz * this->voltage_multiplier_;
      ESP_LOGD(TAG, "Got power=%.1fW, voltage=%.1fV", power, voltage);
      if (this->voltage_sensor_ != nullptr) {
        this->voltage_sensor_->publish_state(voltage);
      }
    }
  }

  if (this->power_sensor_ != nullptr) {
    this->power_sensor_->publish_state(power);
  }

  if (this->energy_sensor_ != nullptr) {
    cf_total_pulses_ += raw_cf;
    float energy = cf_total_pulses_ * this->power_multiplier_ / 3600;
    this->energy_sensor_->publish_state(energy);
  }

  if (this->change_mode_at_++ == this->change_mode_every_) {
    this->current_mode_ = !this->current_mode_;
    ESP_LOGV(TAG, "Changing mode to %s mode", this->current_mode_ ? "CURRENT" : "VOLTAGE");
    this->change_mode_at_ = 0;
    this->sel_pin_->digital_write(this->current_mode_);
  }
}

}  // namespace hlw8012
}  // namespace esphome
