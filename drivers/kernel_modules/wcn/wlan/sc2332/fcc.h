/*
 * Copyright (C) 2021 Spreadtrum Communications Inc.
 *
 * Authors	:
 * Baolei.yuan <Baolei.yuan@unisoc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SPRDWL_FCC_H__
#define __SPRDWL_FCC_H__

struct sprdwl_power_backoff {
	u8 sub_type;
	s8 value;
	u8 mode;
	u8 channel;
} __packed;

struct fcc_power_bo {
	char country[3];
	u8 num;
	struct sprdwl_power_backoff power_backoff[4];
} __packed;

#define MAC_FCC_COUNTRY_NUM	6
struct fcc_power_bo g_fcc_power_table[MAC_FCC_COUNTRY_NUM] = {
	{
		.country = "UY",
		.num = 4,
		.power_backoff = {
			/* subtype, value, mode, channel */
			{1, 8, 1, 11},
			{1, 7, 1, 1},
			{1, 6, 2, 11},
			{1, 5, 2, 1},
		},
	},
	{
		.country = "MX",
		.num = 4,
		.power_backoff = {
			/* subtype, value, mode, channel */
			{1, 9, 1, 11},
			{1, 8, 1, 1},
			{1, 7, 2, 11},
			{1, 6, 2, 1},
		},
	},
};

#endif

