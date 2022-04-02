/* Copyright (C) 2020-2021 Andreas Boehler
 * Copyright (C) 2015 Oliver Kastl
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/version.h"

// Provide VERSION_CODE for ESPHome versions lacking it, as existence checking doesn't work for function-like macros
#ifndef VERSION_CODE
#define VERSION_CODE(major, minor, patch) ((major) << 16 | (minor) << 8 | (patch))
#endif

#include <memory>
#include <string>
#include <vector>
#include <Stream.h>

#ifdef ARDUINO_ARCH_ESP8266
#include <ESPAsyncTCP.h>
#else
// AsyncTCP.h includes parts of freertos, which require FreeRTOS.h header to be included first
#include <freertos/FreeRTOS.h>
#include <AsyncTCP.h>
#endif

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2021, 10, 0)
using SSStream = esphome::uart::UARTComponent;
#else
using SSStream = Stream;
#endif

namespace esphome {
namespace hmlgw {

class HmlgwComponent : public Component {
public:
    HmlgwComponent() = default;
    explicit HmlgwComponent(SSStream *stream) : stream_{stream} {}
    void set_uart_parent(esphome::uart::UARTComponent *parent) { this->stream_ = parent; }

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

    void set_port(uint16_t port) { this->port_ = port; }
    void set_keepalive_port(uint16_t port) { this->keepalive_port_ = port; }
    
    void set_reset_pin(GPIOPin *pin_reset) { pin_reset_ = pin_reset; }

protected:
	void reset();
    void cleanup();
    void read();
    void write();
    void handle_keepalive();
    int read_bidcos_frame(char *buffer, int bufsize);
    int read_bidcos_frame_retries(char *buffer, int bufsize, int retries);
    void detect_radio_module();
    void send_bidcos_frame(uint8_t counter, uint8_t destination, uint8_t command, unsigned char *data, uint8_t data_len);

    struct Client {
        Client(AsyncClient *client, std::vector<uint8_t> &recv_buf, HmlgwComponent *parent);
        ~Client();

        AsyncClient *tcp_client{nullptr};
        std::string identifier{};
        bool disconnected{false};
    };

    bool synced_{false};
    bool module_ready_{false};
    SSStream *stream_{nullptr};
    std::string hm_serial_;
    AsyncServer server_{0};
    AsyncServer keepalive_server_{0};
    uint16_t port_{2000};
    uint16_t keepalive_port_{2001};
    std::vector<uint8_t> recv_buf_{};
    std::vector<uint8_t> keepalive_recv_buf_{};
    std::vector<std::unique_ptr<Client>> clients_{};
    std::vector<std::unique_ptr<Client>> keepalive_clients_{};
    unsigned char keepalive_count_{0};
    unsigned char message_count_{0};
    GPIOPin *pin_reset_{nullptr};
};

} // namespace hmlgw
} // namespace esphome

