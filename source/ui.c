// SPDX-License-Identifier: MIT
//
// SPDX-FileContributor: Adrian "asie" Siekierka, 2024

#include <stdio.h>
#include <nds.h>
#include "ui.h"

static PrintConsole bottomConsole, topConsole;

extern uint32_t dcd_read_chip_id(void);

void ui_toggle_blink_activity(void) {
    topConsole.fontBgMap[(23 * 32) + 30] ^= 0xA000;
}

void ui_toggle_blink_write_activity(void) {
    topConsole.fontBgMap[(23 * 32) + 29] ^= 0x9000;
}

void ui_toggle_blink_spi_activity(void) {
    topConsole.fontBgMap[(23 * 32) + 28] ^= 0xC000;
}

void ui_set_connection_state(bool value) {
    if (value)
        topConsole.fontBgMap[(23 * 32) + 27] |= 0xB000;
    else
        topConsole.fontBgMap[(23 * 32) + 27] &= ~0xF000;
}

void ui_init(void) {
    videoSetMode(MODE_0_2D);
    videoSetModeSub(MODE_0_2D);

    vramSetPrimaryBanks(VRAM_A_LCD, VRAM_B_LCD, VRAM_C_SUB_BG, VRAM_D_MAIN_BG_0x06000000);
    setBrightness(3, 0);

    consoleInit(&bottomConsole,
        0, BgType_Text4bpp, BgSize_T_256x256, 22, 3, false, true);
    consoleInit(&topConsole,
        0, BgType_Text4bpp, BgSize_T_256x256, 22, 3, true, true);

    consoleSelect(&topConsole);

    puts("\x1b[2J" "ntrcardcap\n" __DATE__ "\n" __TIME__ "\n");

    consoleSelect(&bottomConsole);
    puts("\x1b[2J" "\x1b[23;0H");

    for (int i = 0; i < 8; i++) {
        topConsole.fontBgMap[(23 * 32) + 24 + i] = ((uint8_t) '*') + topConsole.fontCharOffset - topConsole.font.asciiOffset;
    }
}

void ui_show_chip_id(void) {
    uint32_t chip_id = dcd_read_chip_id();

    consoleSelect(&topConsole);
    printf("\x1b[21;0H" "\x1b[37;1mUSB controller %04X, rev %02X", (int) ((chip_id >> 8) & 0xFFFF), (int) (chip_id & 0xFF));
    consoleSelect(&bottomConsole);
}

void ui_select_top(void) {
    consoleSelect(&topConsole);
}

void ui_select_bottom(void) {
    consoleSelect(&bottomConsole);
}
