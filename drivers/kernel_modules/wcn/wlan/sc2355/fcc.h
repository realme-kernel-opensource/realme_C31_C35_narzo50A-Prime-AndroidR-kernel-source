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

#define MAX_PHY_MODE	7
#define MAX_POWER_BACKOFF_RULE	100
#define MAX_FCC_COUNTRY_NUM	70

struct sprdwl_priv;

struct sprd_power_backoff {
	u8 subtype;
	u8 channel;
	u8 bw;
	u8 power_rule[MAX_PHY_MODE][2];
} __packed;

struct fcc_power_bo {
	char country[3];
	u8 num;
	struct sprd_power_backoff power_backoff[MAX_POWER_BACKOFF_RULE];
} __packed;

struct fresh_bo_info {
	u8 pw_channel;
	u8 pw_bw;
};

struct sprdwl_fcc_priv {
	struct mutex lock;/* protects the FCC data */
	struct fcc_power_bo *cur_power_bo;
	bool flag;
	u8 channel;
	u8 bw;
};

void sprdwl_fcc_fresh_bo_lock(struct sprdwl_priv *priv, void *data, u16 len);
void sprdwl_fcc_match_country(struct sprdwl_priv *priv, const char *alpha2);
void sprdwl_fcc_reset_bo(void);
void sprdwl_fcc_init(void);
void sprdwl_fcc_deinit(void);
#endif

