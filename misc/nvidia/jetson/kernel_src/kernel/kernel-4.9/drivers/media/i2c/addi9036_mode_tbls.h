/*
 * addi9036_mode_tbls.h - ADI ToF ADDI9036 sensor mode tables
 *
 * Copyright (c) 2020 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ADDI9036_I2C_TABLES__
#define __ADDI9036_I2C_TABLES__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

static const struct reg_sequence addi9036_power_up[] = {
	{ 0xc4c0, 0x001c },
	{ 0xc4c3, 0x001c },
	{ 0xc4d7, 0x0000 },
	{ 0xc4d5, 0x0002 },
	{ 0xc4da, 0x0001 },
	{ 0xc4f0, 0x0000 },
	{ 0xc427, 0x0003 },
	{ 0xc427, 0x0001 },
	{ 0xc427, 0x0000 },
	{ 0xc426, 0x0030 },
	{ 0xc426, 0x0010 },
	{ 0xc426, 0x0000 },
	{ 0xc423, 0x0080 },
	{ 0xc431, 0x0080 },
	{ 0x4001, 0x0007 },
	{ 0x7c22, 0x0004 }
};

static const struct reg_sequence addi9036_power_down[] = {
	{ 0xc022, 0x0001 },
	{ 0x4001, 0x0006 },
	{ 0x7c22, 0x0004 },
	{ 0xc431, 0x0082 },
	{ 0xc423, 0x0000 },
	{ 0xc426, 0x0020 },
	{ 0xc427, 0x0002 },
	{ 0xc4c0, 0x003c },
	{ 0xc4c3, 0x003c },
	{ 0xc4d5, 0x0003 },
	{ 0xc4da, 0x0000 },
	{ 0xc4d7, 0x0001 },
	{ 0xc4f0, 0x0001 }
};

enum {
	ADDI9036_MODE_640X480_CROP_30FPS,
	ADDI9036_MODE_668X750_CROP_30FPS,
	ADDI9036_MODE_POWER_UP,
	ADDI9036_MODE_POWER_DOWN,
};

static const int addi9036_30fps[] = {
	30,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt addi9036_frmfmt[] = {
	{{640, 480}, addi9036_30fps, 1, 0,
			ADDI9036_MODE_640X480_CROP_30FPS},
	{{668, 750}, addi9036_30fps, 1, 0,
			ADDI9036_MODE_668X750_CROP_30FPS},
	/* Add modes with no device tree support after below */
};
#endif /* __ADDI9036_I2C_TABLES__ */
