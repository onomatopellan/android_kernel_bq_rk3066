/*
 * include/linux/power/ricoh61x_battery_init.h
 *
 * Battery initial parameter for RICOH RN5T618/619 power management chip.
 *
 * Copyright (C) 2012-2013 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */
#ifndef __LINUX_POWER_RICOH61X_BATTERY_INIT_H
#define __LINUX_POWER_RICOH61X_BATTERY_INIT_H


uint8_t battery_init_para[][32] = {
        0x0B, 0x3F, 0x0B, 0xCB, 0x0B, 0xEE, 0x0C, 0x08, 0x0C, 0x1E, 0x0C, 0x38, 0x0C, 0x5B, 0x0C, 0x94,
        0x0C, 0xC8, 0x0D, 0x08, 0x0D, 0x55, 0x0E, 0x14, 0x00, 0x39, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56
//        0x0C, 0xC8, 0x0D, 0x08, 0x0D, 0x55, 0x0E, 0x14, 0x00, 0x3E, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56 //150ohme
//        0x0C, 0xC8, 0x0D, 0x08, 0x0D, 0x55, 0x0E, 0x14, 0x00, 0x32, 0x0F, 0xC8, 0x05, 0x2C, 0x22, 0x56 //120ohme
		//0x08, 0xa3, 0x0a, 0xf9, 0x0b, 0x4b, 0x0b, 0x74,
		//0x0b, 0x94, 0x0b, 0xb5, 0x0b, 0xe6, 0x0c, 0x30,
		//0x0c, 0x92, 0x0c, 0xf4, 0x0d, 0x6f, 0x08, 0xca,
		//0x00, 0x36, 0x0f, 0xc8, 0x05, 0x2c, 0x22, 0x56
};

#endif

/*
<Other Parameter>
nominal_capacity=3800
cut-off_v=3400
thermistor_b=3435
board_impe=0
bat_impe=0.1363
load_c=768
available_c=3604
battery_v=3515
MakingMode=Normal
ChargeV=4.20V
LoadMode=Resistor
 */

