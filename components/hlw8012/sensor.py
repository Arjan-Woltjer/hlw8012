import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_CHANGE_MODE_EVERY,
    CONF_INITIAL_MODE,
    CONF_SEL_PIN,
    CONF_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_ENERGY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    STATE_CLASS_TOTAL_INCREASING,
    STATE_CLASS_MEASUREMENT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_WATT_HOURS,
)

AUTO_LOAD = ["pulse_counter"]

hlw8012_ns = cg.esphome_ns.namespace("hlw8012")
HLW8012Component = hlw8012_ns.class_("HLW8012Component", cg.PollingComponent)
HLW8012InitialMode = hlw8012_ns.enum("HLW8012InitialMode")

CONF_CF1_PIN = "cf1_pin"
CONF_CF_PIN = "cf_pin"
CONF_VOLTAGE_CONSTANT = "voltage_constant"
CONF_CURRENT_CONSTANT = "current_constant"
CONF_POWER_CONSTANT = "power_constant"
     
INITIAL_MODES = {
    CONF_CURRENT: HLW8012InitialMode.HLW8012_INITIAL_MODE_CURRENT,
    CONF_VOLTAGE: HLW8012InitialMode.HLW8012_INITIAL_MODE_VOLTAGE,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(HLW8012Component),
        cv.Required(CONF_SEL_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_CF_PIN): cv.All(
            pins.internal_gpio_input_pullup_pin_schema, pins.validate_has_interrupt
        ),
        cv.Required(CONF_CF1_PIN): cv.All(
            pins.internal_gpio_input_pullup_pin_schema, pins.validate_has_interrupt
        ),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT_HOURS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_VOLTAGE_CONSTANT, default=1200): cv.positive_not_null_int,
        cv.Optional(CONF_CURRENT_CONSTANT, default=1000): cv.positive_not_null_int,
        cv.Optional(CONF_POWER_CONSTANT, default=1000): cv.positive_not_null_int,
        cv.Optional(CONF_CHANGE_MODE_EVERY, default=8): cv.positive_not_null_int,
        cv.Optional(CONF_INITIAL_MODE, default=CONF_VOLTAGE): cv.one_of(
            *INITIAL_MODES, lower=True
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    sel = await cg.gpio_pin_expression(config[CONF_SEL_PIN])
    cg.add(var.set_sel_pin(sel))
    cf = await cg.gpio_pin_expression(config[CONF_CF_PIN])
    cg.add(var.set_cf_pin(cf))
    cf1 = await cg.gpio_pin_expression(config[CONF_CF1_PIN])
    cg.add(var.set_cf1_pin(cf1))

    if CONF_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VOLTAGE])
        cg.add(var.set_voltage_sensor(sens))
    if CONF_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_CURRENT])
        cg.add(var.set_current_sensor(sens))
    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))
    if CONF_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_ENERGY])
        cg.add(var.set_energy_sensor(sens))
    cg.add(var.set_voltage_constant(config[CONF_VOLTAGE_CONSTANT]))
    cg.add(var.set_current_constant(config[CONF_CURRENT_CONSTANT]))
    cg.add(var.set_power_constant(config[CONF_POWER_CONSTANT]))
    cg.add(var.set_change_mode_every(config[CONF_CHANGE_MODE_EVERY]))
    cg.add(var.set_initial_mode(INITIAL_MODES[config[CONF_INITIAL_MODE]]))
