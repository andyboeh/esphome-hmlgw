# Copyright (C) 2021-2022 Andreas Boehler
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID, CONF_PORT

# ESPHome doesn't know the Stream abstraction yet, so hardcode to use a UART for now.

DEPENDENCIES = ["uart", "network"]
AUTO_LOAD = ["async_tcp"]

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

	if CONF_PORT in config:
		cg.add(var.set_port(config[CONF_PORT]))

	if CONF_KEEPALIVE_PORT in config:
		cg.add(var.set_keepalive_port(config[CONF_KEEPALIVE_PORT]))
	if CONF_RESET_PIN in config:
		reset = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
		cg.add(var.set_reset_pin(reset))


