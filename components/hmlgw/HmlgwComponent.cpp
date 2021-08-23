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

#include "HmlgwComponent.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"

#define VERSION "0.0.1"

namespace esphome {
namespace hmlgw {

static const char *TAG = "hmlgw";
static const char *g_productString = "01,Revilo-HM-LGW," VERSION ",%s\r\n";

using namespace esphome;

void HmlgwComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up HMLGW server...");
    this->recv_buf_.reserve(128);
    this->keepalive_recv_buf_.reserve(128);

    this->server_ = AsyncServer(this->port_);
    this->keepalive_server_ = AsyncServer(this->keepalive_port_);
    this->server_.begin();
    this->keepalive_server_.begin();
    this->server_.onClient([this](void *h, AsyncClient *tcpClient) {
        if(tcpClient == nullptr)
            return;

        this->clients_.push_back(std::unique_ptr<Client>(new Client(tcpClient, this->recv_buf_, this)));
    }, this);

    this->keepalive_server_.onClient([this](void *h, AsyncClient *tcpClient) {
        if(tcpClient == nullptr)
            return;

        this->keepalive_clients_.push_back(std::unique_ptr<Client>(new Client(tcpClient, this->keepalive_recv_buf_, this)));
    }, this);
    
    if(this->pin_reset_) {
        this->pin_reset_->setup();
    }
}

void HmlgwComponent::reset() {
    if(this->pin_reset_) {
        this->pin_reset_->digital_write(false);
        delay(10); // roughly 10ms
        this->pin_reset_->digital_write(true);
    }
}

void HmlgwComponent::loop() {
    this->cleanup();
    this->read();
    this->write();
    this->handle_keepalive();
}

void HmlgwComponent::cleanup() {
    auto discriminator = [](std::unique_ptr<Client> &client) { return !client->disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    for (auto it = last_client; it != this->clients_.end(); it++)
        ESP_LOGD(TAG, "Client %s disconnected", (*it)->identifier.c_str());

    this->clients_.erase(last_client, this->clients_.end());
    
    last_client = std::partition(this->keepalive_clients_.begin(), this->keepalive_clients_.end(), discriminator);
    for (auto it = last_client; it != this->keepalive_clients_.end(); it++)
        ESP_LOGD(TAG, "Keepalive Client %s disconnected", (*it)->identifier.c_str());

    this->keepalive_clients_.erase(last_client, this->keepalive_clients_.end());
}

int HmlgwComponent::read_bidcos_frame(char *buffer, int bufsize) {
    int result = 0;
    int msgLen = 0;
    int escaped = 0;
    int count = bufsize;
    unsigned char *buf = (unsigned char *)buffer;
    char escapeValue = 0x00;
    bool haveLength = false;
    
    while (count)
    {
        int r = this->stream_->readBytes(buf, 1);
        if( r <= 0 )
        {
            result = r;
            ESP_LOGE(TAG, "readBidcosFrame" );
            break;
        }
        if( *buf == 0xFD ) // sync byte
        {
            result = 0;
            msgLen = 0;
            escaped = 0;
            count = bufsize;
            buf = (unsigned char *)buffer;
            escapeValue = 0x00;
            haveLength = false;
            // fprintf( stderr,  "readBidcosFrame reset\n" );
        }
        else if( result == 0 )
        {
            ESP_LOGE(TAG, "readBidcosFrame sync error %2.2x\n", *buf );
            // No sync at beginning? Not good...
            break;
        }
        
        result += r;
        
        if( *buf == 0xFC && result > 1 ) // escape byte
        {
            escaped++;
            escapeValue = 0x80;
            ESP_LOGD(TAG, "ESCAPE msgLen set %d result %d\n", msgLen, result );
        }
        else
        {
            escapeValue = 0x00;
        }

        if( false == haveLength )
        {
            if( result == 2 + escaped )
            {
                msgLen = *buf; // MSB
                msgLen |= escapeValue;
                msgLen = msgLen<<8;
            }
            else if( result == 3 + escaped )
            {
                msgLen |= *buf; // LSB
                msgLen |= escapeValue;
                haveLength = true;
                ESP_LOGD(TAG, "readBidcosFrame msgLen set %d result %d\n", msgLen, result );
            }
        }
        else if( result >= msgLen + escaped + 5 )
        {
            ESP_LOGD(TAG, "readBidcosFrame done, msgLen %d result %d\n", msgLen, result );
            break;
        }
        
        count -= r;
        buf+=r;
    }

    return result;


}

void HmlgwComponent::read() {
    int len;
    char buf[4096];

    if(this->stream_->available() > 0) {
        int res = this->read_bidcos_frame(buf, sizeof(buf));
        if(res > 0 && this->synced_) {
            for(auto const& client : this->clients_)
                client->tcp_client->write(buf, res);
        }
    }
}

void HmlgwComponent::write() {
    size_t len;
    int index;
    int number;
    int pos = 0;

    len = this->recv_buf_.size();
    if(len == 0)
        return;

    if(this->synced_) {
        while (len > 0) {
            this->stream_->write(this->recv_buf_.data(), len);
            this->recv_buf_.erase(this->recv_buf_.begin(), this->recv_buf_.begin() + len);
            len = this->recv_buf_.size();
        }
    } else {
        for(int i=0; i<len; i++) {
            if(this->recv_buf_.at(i) == '\n') {
                pos = i;
                break;
            }
        }
        
        if(pos == 0)
            return;
        
        if(sscanf((const char*)&this->recv_buf_.data()[1], ">%x,%d", &index, &number) == 2) {
            if(index == (int)this->message_count_ && number == 0) {
                this->synced_ = true;
                // FIXME: Flush serial buffer
            }
        }
    }
    
}

void HmlgwComponent::handle_keepalive() {
    size_t len;
    int pos = 0;
    char buf[128];
    
    len = this->keepalive_recv_buf_.size();
    if(len == 0)
        return;
    for(int i=0; i<len; i++) {
        if(this->keepalive_recv_buf_.at(i) == '\n') {
            pos = i;
            break;
        }
    }
    if(pos == 0)
        return;
    
    if(this->keepalive_recv_buf_.at(0) == 'L' || this->keepalive_recv_buf_.at(0) == 'K') {
        int counter;
        if(1 == sscanf((const char*)&this->keepalive_recv_buf_.data()[1], "%x", &counter)) {
            if(this->keepalive_recv_buf_.at(0) == 'L') {
                this->keepalive_count_ = counter;
            }
            else {
                this->keepalive_count_++;
                if(counter != this->keepalive_count_) {
                    for(auto &client : this->keepalive_clients_)
                        client->tcp_client->close(true);
                }
            }
            sprintf(buf, ">%c%2.2x\r\n", this->keepalive_recv_buf_.at(0), counter);
            for (auto const& client : this->keepalive_clients_)
                client->tcp_client->write(buf, strlen(buf));
        }
     }
     this->keepalive_recv_buf_.erase(this->keepalive_recv_buf_.begin(), this->keepalive_recv_buf_.begin() + pos + 1);
}

void HmlgwComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", network_get_address().c_str(), this->port_);
}

void HmlgwComponent::on_shutdown() {
    for (auto &client : this->clients_)
        client->tcp_client->close(true);
}

HmlgwComponent::Client::Client(AsyncClient *client, std::vector<uint8_t> &recv_buf, HmlgwComponent *parent) :
        tcp_client{client}, identifier{client->remoteIP().toString().c_str()}, disconnected{false} {
    ESP_LOGD(TAG, "New client connected from %s", this->identifier.c_str());

    char buf[128];

    this->tcp_client->onError(     [this](void *h, AsyncClient *client, int8_t error)  { this->disconnected = true; });
    this->tcp_client->onDisconnect([this](void *h, AsyncClient *client)                { this->disconnected = true; });
    this->tcp_client->onTimeout(   [this](void *h, AsyncClient *client, uint32_t time) { this->disconnected = true; });

    this->tcp_client->onData([&](void *h, AsyncClient *client, void *data, size_t len) {
        if (len == 0 || data == nullptr)
            return;

        auto buf = static_cast<uint8_t *>(data);
        recv_buf.insert(recv_buf.end(), buf, buf + len);
    }, nullptr);


    sprintf(buf, "H%2.2x,", ++parent->message_count_);
    sprintf(&buf[strlen(buf)], g_productString, parent->hm_serial_.c_str());
    this->tcp_client->write(buf, strlen(buf));
    if(client->localPort() == parent->keepalive_port_) {  
        sprintf(buf, "S%2.2x,SysCom-1.0\r\n", ++parent->message_count_);
        this->tcp_client->write(buf, strlen(buf));
    }
    if(client->localPort() == parent->port_) {
        sprintf(buf, "S%2.2x,BidCoS-over-LAN-1.0\r\n", ++parent->message_count_);
        this->tcp_client->write(buf, strlen(buf));
    }
}

HmlgwComponent::Client::~Client() {
    delete this->tcp_client;
}

} // hmlgw
} // esphome

