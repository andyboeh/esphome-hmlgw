/* Copyright (C) 2020-2022 Andreas Boehler
 * Copyright (C) 2021 Alexander Reinert
 *
 * Based on hmframe.h from the HB-RF-ETH project, originally licensed
 * under the Apache License, Version 2.0
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

#include <stdint.h>


static const uint8_t HM_DST_HMSYSTEM = 0x00;
static const uint8_t HM_DST_TRX = 0x01;
static const uint8_t HM_DST_HMIP = 0x02;
static const uint8_t HM_DST_LLMAC = 0x03;
static const uint8_t HM_DST_COMMON = 0xfe;

static const uint8_t HM_CMD_HMSYSTEM_IDENTIFY = 0x00;
static const uint8_t HM_CMD_HMSYSTEM_GET_VERSION = 0x02;
static const uint8_t HM_CMD_HMSYSTEM_CHANGE_APP = 0x03;
static const uint8_t HM_CMD_HMSYSTEM_ACK = 0x04;
static const uint8_t HM_CMD_HMSYSTEM_GET_SERIAL = 0x0b;

static const uint8_t HM_CMD_TRX_GET_VERSION = 0x02;
static const uint8_t HM_CMD_TRX_ACK = 0x04;
static const uint8_t HM_CMD_TRX_GET_MCU_TYPE = 0x09;
static const uint8_t HM_CMD_TRX_GET_DEFAULT_RF_ADDR = 0x10;

static const uint8_t HM_CMD_HMIP_GET_DEFAULT_RF_ADDR = 0x01;
static const uint8_t HM_CMD_HMIP_ACK = 0x06;

static const uint8_t HM_CMD_LLMAC_ACK = 0x01;
static const uint8_t HM_CMD_LLMAC_GET_SERIAL = 0x07;
static const uint8_t HM_CMD_LLMAC_GET_DEFAULT_RF_ADDR = 0x08;

static const uint8_t HM_CMD_COMMON_IDENTIFY = 0x01;
static const uint8_t HM_CMD_COMMON_START_BL = 0x02;
static const uint8_t HM_CMD_COMMON_START_APP = 0x03;
static const uint8_t HM_CMD_COMMON_GET_SGTIN = 0x04;
static const uint8_t HM_CMD_COMMON_ACK = 0x05;

class HMFrame
{
public:
    static bool TryParse(unsigned char *buffer, uint16_t len, HMFrame *frame);
    static uint16_t crc(unsigned char *buffer, uint16_t len);

    HMFrame();
    uint8_t counter;
    uint8_t destination;
    uint8_t command;
    unsigned char *data;
    uint16_t data_len;
    void dump();

    uint16_t encode(unsigned char *buffer, uint16_t len, bool escaped);
};
