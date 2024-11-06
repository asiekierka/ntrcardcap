// SPDX-License-Identifier: MIT
//
// SPDX-FileContributor: Adrian "asie" Siekierka, 2024

#ifndef _UI_H_
#define _UI_H_

#define UI_COLOR_ERROR "\x1b[31;1m"
#define UI_COLOR_INFO "\x1b[37;1m"
#define UI_COLOR_SUCCESS "\x1b[32;1m"
#define UI_COLOR_WARNING "\x1b[33;1m"

void ui_toggle_blink_activity(void);
void ui_toggle_blink_write_activity(void);
void ui_toggle_blink_spi_activity(void);
void ui_set_connection_state(bool value);
void ui_init(void);
void ui_show_chip_id(void);
void ui_select_top(void);
void ui_select_bottom(void);

#endif /* _UI_H_ */
