/* Copyright (C) 2020-2021 Andreas Boehler
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
#include "esphome/core/esphal.h"
#include "esphome/components/uart/uart.h"

#include <memory>
#include <string>
#include <vector>
#include <Stream.h>

#ifdef ARDUINO_ARCH_ESP8266
#include <ESPAsyncTCP.h>
#else
#include <AsyncTCP.h>
#endif

namespace esphome {
namespace hmlgw {

class HmlgwComponent : public Component {
public:
    HmlgwComponent() = default;
    explicit HmlgwComponent(Stream *stream) : stream_{stream} {}
    void set_uart_parent(esphome::uart::UARTComponent *parent) { this->stream_ = parent; }

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

    void set_port(uint16_t port) { this->port_ = port; }
    void set_keepalive_port(uint16_t port) { this->keepalive_port_ = port; }

    void set_hm_serial(const std::string &hm_serial) { this->hm_serial_ = hm_serial; }
    
    void set_reset_pin(GPIOPin *pin_reset) { pin_reset_ = pin_reset; }

protected:
	void reset();
    void cleanup();
    void read();
    void write();
    void handle_keepalive();
    int read_bidcos_frame(char *buffer, int bufsize);

    struct Client {
        Client(AsyncClient *client, std::vector<uint8_t> &recv_buf, HmlgwComponent *parent);
        ~Client();

        AsyncClient *tcp_client{nullptr};
        std::string identifier{};
        bool disconnected{false};
    };

    bool synced_{false};
    Stream *stream_{nullptr};
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

