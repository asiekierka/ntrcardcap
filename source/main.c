/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <nds.h>

#include "tusb.h"
#include "ui.h"

static void wait_for_key(uint16_t mask) {
    while (1) {
        swiWaitForVBlank();
        scanKeys();

        if (keysDown() & mask)
            break;
    }
}

static void exit_to_loader(void) {
    printf("\n" UI_COLOR_INFO "Press any button to exit...\n");
    wait_for_key(0xFFFF);
    exit(0);
}

// NTRCARD code

bool card_delay = false;
bool running = true;
uint8_t rx_buffer[49152];
uint8_t tx_buffer[49152];
uint32_t rx_bufpos = 0;

// 0x10 [3:bufsize] [4:flags] [command] [data...]
#define CMD_NTR_START  0x10
// 0x11 [3:bufsize] [data...]
#define CMD_NTR_FINISH 0x11
// 0x12 [value]
#define CMD_NTR_SPI    0x12
// 0x13 [1:empty] [1:seed0h] [1:seed1h] [4:seed0l] [4:seed0h]
#define CMD_NTR_SET_SEED 0x13
// 0x14 [none] [2:flags]
#define CMD_NTR_SPICNT 0x14

/* static const uint32_t ntr_buffer_sizes[] = {
  	0, 256 << 1, 256 << 2, 256 << 3, 256 << 4, 256 << 5, 256 << 6, 4
}; */

static inline uint32_t get_u16(uint8_t *buf) {
    return buf[0] | (buf[1] << 8);
}

static inline uint32_t get_u24(uint8_t *buf) {
    return buf[0] | (buf[1] << 8) | (buf[2] << 16);
}

static inline uint32_t get_u32(uint8_t *buf) {
    return buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
}

static inline void set_u16(uint8_t *buf, uint32_t value) {
    buf[0] = value;
    buf[1] = value >> 8;
}

static inline void set_u24(uint8_t *buf, uint32_t value) {
    buf[0] = value;
    buf[1] = value >> 8;
    buf[2] = value >> 16;
}

static inline void set_u32(uint8_t *buf, uint32_t value) {
    buf[0] = value;
    buf[1] = value >> 8;
    buf[2] = value >> 16;
    buf[3] = value >> 24;
}

static void slot1_card_reset(void) {
    REG_ROMCTRL = 0;
    REG_AUXSPICNTH = 0;

    swiDelay(167550);

    REG_CARD_1B0 = 0;
    REG_CARD_1B4 = 0;
    REG_CARD_1B8 = 0;
    REG_CARD_1BA = 0;

    REG_AUXSPICNTH = CARD_CR1_ENABLE | CARD_CR1_IRQ;
    REG_ROMCTRL = CARD_nRESET | CARD_SEC_SEED;

    while (REG_ROMCTRL & CARD_BUSY);
}

static void cdc_write_large(void *buf, uint32_t len) {
    for (int i = 0; i < len; i += CFG_TUD_CDC_EP_BUFSIZE) {
        uint32_t bytes_to_send = (len - i) > CFG_TUD_CDC_EP_BUFSIZE ? CFG_TUD_CDC_EP_BUFSIZE : (len - i);
        tud_cdc_write(tx_buffer + i, bytes_to_send);
    }
    tud_cdc_write_flush();
}

static inline void ui_toggle_blink_rom_activity(uint32_t flags) {
    if (flags & CARD_WR)
        ui_toggle_blink_write_activity();
    else
        ui_toggle_blink_activity();
}

ITCM_CODE
static void cdc_task(void) {
    // populate rx buffer
    uint32_t rx_available = tud_cdc_available();
    if (rx_available > 0) {
        if ((rx_bufpos + rx_available) > sizeof(rx_buffer)) {
            printf(UI_COLOR_ERROR "Buffer overrun\n");
            exit_to_loader();
        }

        uint32_t bytes_read = tud_cdc_read(rx_buffer + rx_bufpos, rx_available);
        rx_bufpos += bytes_read;
    }

    // try to process commands from RX buffer
    while (rx_bufpos) {
        uint32_t data_read = 0;
        switch (rx_buffer[0]) {
        default:
            printf(UI_COLOR_ERROR "Unknown command %02X\n", rx_buffer[0]);
            exit_to_loader();
            break;

        case CMD_NTR_START:
            // Read data from buffer
            if (rx_bufpos < 16) break;
            uint32_t data_length = get_u24(rx_buffer + 1);
            if (rx_bufpos < (16 + data_length)) break;
            uint32_t flags = get_u32(rx_buffer + 4);

            data_read = 16 + data_length;

            ui_toggle_blink_rom_activity(flags);

            // Handle edge cases
            if (rx_buffer[8] == 0x9F && flags == 0xAD180000) {
                printf("Card reset detected\n");
                slot1_card_reset();
            }

            if (card_delay && rx_buffer[8] == 0xB7 && rx_buffer[13] == 0 && rx_buffer[14] == 0 && rx_buffer[15] == 0) {
                printf("Disabling delay\n");
                card_delay = false;
            }

            // Start card command
            // REG_AUXSPICNTH = CARD_CR1_ENABLE | CARD_CR1_IRQ;
            for (int i = 0; i < 8; i++)
                REG_CARD_COMMAND[i] = rx_buffer[8 + i];
            REG_ROMCTRL = flags;

            // Write data to card
            if (flags & CARD_WR) {
                uint32_t *data = (uint32_t*) (rx_buffer + 16);
                do {
                    if (REG_ROMCTRL & CARD_DATA_READY) {
                        REG_CARD_DATA_RD = *(data++);
                    }
                } while (REG_ROMCTRL & CARD_BUSY);
            }

            if (card_delay) {
                swiDelay(8400 * 25);
            } else if (rx_buffer[8] == 0x3C || rx_buffer[8] == 0x3D) {
                printf("Enabling delay\n");
                card_delay = true;
            }

            // Read data from card
            tx_buffer[0] = CMD_NTR_FINISH;
            int tx_data_bufpos = 0;
            if (!(flags & CARD_WR)) {
                uint32_t *data = (uint32_t*) (tx_buffer + 4);
                do {
                    if (REG_ROMCTRL & CARD_DATA_READY) {
                        uint32_t value = REG_CARD_DATA_RD;
                        if (tx_data_bufpos < 32768) {
                            *(data++) = value;
                            tx_data_bufpos += 4;
                        }
                    }
                } while (REG_ROMCTRL & CARD_BUSY);
            }
            set_u24(tx_buffer + 1, tx_data_bufpos);

            // Send response to host
            uint32_t tx_buflen = tx_data_bufpos + 4;
            cdc_write_large(tx_buffer, tx_buflen);

            ui_toggle_blink_rom_activity(flags);

            break;

        case CMD_NTR_SPICNT:
            // Read data from buffer
            if (rx_bufpos < 4) break;
            data_read = 4;

            uint16_t spiCnt = *((uint16_t*) (rx_buffer + 2));;
            REG_AUXSPICNT = spiCnt;
            break;

        case CMD_NTR_SPI:
            // Read data from buffer
            if (rx_bufpos < 2) break;
            data_read = 2;

            ui_toggle_blink_spi_activity();

            // Perform SPI exchange
            REG_AUXSPIDATA = rx_buffer[1];
            eepromWaitBusy();
            rx_buffer[1] = REG_AUXSPIDATA;

            // Write data to host
            tud_cdc_write(rx_buffer, 2);
            tud_cdc_write_flush();

            ui_toggle_blink_spi_activity();
            break;

        case CMD_NTR_SET_SEED:
            if (rx_bufpos < 16) break;
            printf("Synchronizing 40001Bx\n");
            REG_CARD_1B8 = rx_buffer[2];
            REG_CARD_1BA = rx_buffer[3];
            REG_CARD_1B0 = *((uint32_t*) (rx_buffer + 4));
            REG_CARD_1B4 = *((uint32_t*) (rx_buffer + 8));
            REG_ROMCTRL = *((uint32_t*) (rx_buffer + 12));
            data_read = 16;
            break;
        }

        // If no command read, return
        if (!data_read)
            break;

        if (rx_bufpos > data_read)
            memmove(rx_buffer, rx_buffer + data_read, rx_bufpos - data_read);
        rx_bufpos -= data_read;
    }
}

// UI/bringup code

int main(void) {
    defaultExceptionHandler();
    powerOff(POWER_3D_CORE | POWER_MATRIX);

    ui_init();

    if (isDSiMode()) {
        printf(UI_COLOR_ERROR "This program is not compatible with DSi/3DS consoles.\n");
        exit_to_loader();
    }

    sysSetBusOwners(BUS_OWNER_ARM9, BUS_OWNER_ARM9);

    REG_ROMCTRL = 0;
    REG_AUXSPICNTH = 0;

    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO
    };
    bool usb_init_status = !isDSiMode() && tusb_init(BOARD_TUD_RHPORT, &dev_init);

    ui_show_chip_id();

    if (!usb_init_status) {
        printf(UI_COLOR_ERROR "Could not initialize USB!\n");
        exit_to_loader();
    }

    bool last_key_lid = false;

    while (running) {
        tud_task();
        cdc_task();

        scanKeys();
        if (keysDown() & KEY_START) {
            running = false;
            break;
        }

        bool curr_key_lid = keysHeld() & KEY_LID;
        if (last_key_lid != curr_key_lid) {
            if (curr_key_lid)
                powerOff(POWER_ALL_2D);
            else
                powerOn(POWER_ALL_2D);
            swiWaitForVBlank();
            last_key_lid = curr_key_lid;
        }
    }

    tud_deinit(0);
    powerOn(POWER_ALL);
}

// Invoked when device is mounted
void tud_mount_cb(void) {
    fifoSendValue32(FIFO_PM, PM_REQ_SLEEP_DISABLE);
    printf(UI_COLOR_INFO "Device connected.\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    printf(UI_COLOR_INFO "Device disconnected.\n");
    fifoSendValue32(FIFO_PM, PM_REQ_SLEEP_ENABLE);
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void) remote_wakeup_en;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
}

// Invoked when cdc when line state changed e.g connected/disconnected
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void) itf;
    (void) rts;

    ui_set_connection_state(dtr);
}
