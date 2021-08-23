# Copyright (C) 2021 Andreas Boehler
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT

# ESPHome doesn't know the Stream abstraction yet, so hardcode to use a UART for now.

DEPENDENCIES = ["uart"]

MULTI_CONF = True

hmlgw_ns = cg.esphome_ns.namespace("hmlgw")
HmlgwComponent = hmlgw_ns.class_("HmlgwComponent", cg.Component)

CONF_KEEPALIVE_PORT = 'keepalive_port'
CONF_RESET_PIN = 'reset_pin'
CONF_HM_SERIAL = 'homematic_address'

CONFIG_SCHEMA = (
	cv.Schema(
		{
			cv.GenerateID(): cv.declare_id(HmlgwComponent),
			cv.Required(CONF_HM_SERIAL): cv.string, 
			cv.Optional(CONF_PORT): cv.port,
			cv.Optional(CONF_KEEPALIVE_PORT): cv.port,
			cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
		}
	)
		.extend(cv.COMPONENT_SCHEMA)
		.extend(uart.UART_DEVICE_SCHEMA)
)

async def to_code(config):
	var = cg.new_Pvariable(config[CONF_ID])
	await cg.register_component(var, config)
	await uart.register_uart_device(var, config)
	cg.add(var.set_hm_serial(config[CONF_HM_SERIAL]))

	if CONF_PORT in config:
		cg.add(var.set_port(config[CONF_PORT]))

	if CONF_KEEPALIVE_PORT in config:
		cg.add(var.set_keepalive_port(config[CONF_KEEPALIVE_PORT]))
	if CONF_RESET_PIN in config:
		reset = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
		cg.add(var.set_reset_pin(reset))


