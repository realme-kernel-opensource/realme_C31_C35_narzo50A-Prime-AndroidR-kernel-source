#include "pam_wifi_driver.h"
#include <linux/dma-direction.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

dma_addr_t dl_type1_phy_addr;
dma_addr_t dl_type2_phy_addr;
dma_addr_t dl_type3_phy_addr;
dma_addr_t dl_type4_phy_addr;
dma_addr_t dl_free_phy_addr;
dma_addr_t miss_tx_phy_addr;
dma_addr_t miss_rx_phy_addr;
dma_addr_t dl_4in1_phy_addr;
dma_addr_t ul_tx_phy_addr;
dma_addr_t ul_rx_phy_addr;

void *dl_type1_virt_addr;
void * dl_type2_virt_addr;
void * dl_type3_virt_addr;
void * dl_type4_virt_addr;
void * dl_free_virt_addr;
void * miss_tx_virt_addr;
void * miss_rx_virt_addr;
void * dl_4in1_virt_addr;
void * ul_tx_virt_addr;
void * ul_rx_virt_addr;

u32 * pam_wifi_msdu_header_info;
dma_addr_t term_pam_wifi_msdu_header_buf;
struct sprdwl_pamwifi_priv *pamwifi_priv;

pamwifi_interrupt_table_t pamwifi_int_table[] = {
	{
		.id = pamwifi_ul_free,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_UL_FREE
	},
	{
		.id = pamwifi_ul_fill,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_UL_FILL
	},
	{
		.id = pamwifi_dl_free,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FREE
	},
	{
		.id = pamwifi_dl_fill_4in1,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_4IN1
	},
	{
		.id = pamwifi_dl_fill_type1,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE1
	},
	{
		.id = pamwifi_dl_fill_type2,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE2
	},
	{
		.id = pamwifi_dl_fill_type3,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE3
	},
	{
		.id = pamwifi_dl_fill_type4,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE4
	},
	{
		.id = pamwifi_dl_miss_rx,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_INDEX_MISS_RX
	},
	{
		.id = pamwifi_dl_miss_tx,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_INDEX_MISS_TX
	},
	{
		.id = pamwifi_dl_4in1_type1_overflow,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE1_OVERFLOW
	},
	{
		.id = pamwifi_dl_4in1_type2_overflow,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE2_OVERFLOW
	},
	{
		.id = pamwifi_dl_4in1_type3_overflow,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE3_OVERFLOW
	},
	{
		.id = pamwifi_dl_4in1_type4_overflow,
		.int_owner = BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE4_OVERFLOW
	},
};

unsigned char tx_ipv4_udp[244] = {
0x40, 0x45, 0xda, 0xaa, 0xbb, 0xcc, 0x40, 0x45,
0xda, 0xcc, 0xbb, 0xaa, 0x08, 0x00, 0x45, 0x00,
0x00, 0xe4, 0xc8, 0xc0, 0x40, 0x00, 0x40, 0x11,
0x8d, 0xb2, 0xc0, 0xa8, 0x31, 0x01, 0xc0, 0xa8,
0x31, 0x44, 0x67, 0x62, 0x3c, 0xbe, 0x00, 0xd0,
0xa0, 0x26, 0x80, 0x21, 0x81, 0x4b, 0x03, 0x68,
0x2b, 0x37, 0xde, 0xad, 0xbe, 0xef, 0x47, 0x10,
0x11, 0x35, 0xb1, 0x00, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00,
0x00, 0x00, 0xAA, 0xBB};

unsigned char tx_ipv6_udp[244] = {
0x40, 0x45, 0xda, 0xaa, 0xbb, 0xcc, 0x40, 0x45,
0xda, 0xcc, 0xbb, 0xaa, 0x08, 0x00, 0x60, 0x00,
0x00, 0x00, 0x00, 0x12, 0x11, 0x40, 0xfc, 0xb1,
0xab, 0xab, 0xcd, 0xcd, 0xef, 0xe0, 0x70, 0x0b,
0xf6, 0x64, 0x27, 0x04, 0xb4, 0xd3, 0xfc, 0xb1,
0xca, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x02, 0x54, 0xc1, 0x01,
0x30, 0x39, 0x00, 0x12, 0x9c, 0x36, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*TODO*/
void pamwifi_config_ipa(void)
{
/*follow ipa definition*/
#define term_wifi_ul 1
#define term_wifi_dl 6
	u32 temp, depth;

	sipa_get_ep_info(SIPA_EP_WIFI, &pamwifi_priv->sipa_info);

	depth = pamwifi_priv->sipa_info.dl_fifo.fifo_depth;
	pamwifi_priv->sipa_params.recv_param.tx_enter_flowctrl_watermark = depth - depth / 4;
	pamwifi_priv->sipa_params.recv_param.tx_leave_flowctrl_watermark = depth / 2;
	pamwifi_priv->sipa_params.recv_param.flow_ctrl_cfg = 1;
	pamwifi_priv->sipa_params.send_param.flow_ctrl_irq_mode = 2;
	pamwifi_priv->sipa_params.send_param.tx_intr_threshold = 64;
	pamwifi_priv->sipa_params.send_param.tx_intr_delay_us = 200;
	pamwifi_priv->sipa_params.recv_param.tx_intr_threshold = 64;
	pamwifi_priv->sipa_params.recv_param.tx_intr_delay_us = 200;
	pamwifi_priv->sipa_params.id = SIPA_EP_WIFI;

	/*Setting IPA PAM WIFI fifo base addr*/
	writel_relaxed((u32)((pamwifi_priv->sipa_info.ul_fifo.rx_fifo_base_addr) & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_UL_FILLED_FIFO_BASE_ADDRL);
	writel_relaxed((u32)((pamwifi_priv->sipa_info.ul_fifo.tx_fifo_base_addr) & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_UL_FREE_FIFO_BASE_ADDRL);
	writel_relaxed((u32)((pamwifi_priv->sipa_info.dl_fifo.tx_fifo_base_addr) & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_DL_FILLED_FIFO_BASE_ADDRL);
	writel_relaxed((u32)((pamwifi_priv->sipa_info.dl_fifo.rx_fifo_base_addr) & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_DL_FREE_FIFO_BASE_ADDRL);

	temp = (u32)(((pamwifi_priv->sipa_info.ul_fifo.rx_fifo_base_addr) >> 32) & 0xFFl);
	temp |= (u32)(((pamwifi_priv->sipa_info.ul_fifo.tx_fifo_base_addr) >> 32) & 0xFFl) << 8;
	temp |= (u32)(((pamwifi_priv->sipa_info.dl_fifo.tx_fifo_base_addr) >> 32) & 0xFFl) << 16;
	temp |= (u32)(((pamwifi_priv->sipa_info.dl_fifo.rx_fifo_base_addr) >> 32) & 0xFFl) << 24;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_AP_UL_FREE_FIFO_BASE_ADDRH);

	/*Setting IPA PAM WIFI fifo sts*/
	writel_relaxed((u32)(pamwifi_priv->sipa_info.ul_fifo.fifo_sts_addr & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_UL_FILLED_STS_BASE_ADDRL);
	writel_relaxed((u32)(pamwifi_priv->sipa_info.ul_fifo.fifo_sts_addr & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_UL_FREE_STS_BASE_ADDRL);
	writel_relaxed((u32)(pamwifi_priv->sipa_info.dl_fifo.fifo_sts_addr & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_DL_FILLED_STS_BASE_ADDRL);
	writel_relaxed((u32)(pamwifi_priv->sipa_info.dl_fifo.fifo_sts_addr & 0xFFFFFFFFl), (void *)REG_PAM_WIFI_AP_DL_FREE_STS_BASE_ADDRL);

	temp = (u32)((pamwifi_priv->sipa_info.ul_fifo.fifo_sts_addr>> 32) & 0xFFl);
	temp |= (u32)((pamwifi_priv->sipa_info.ul_fifo.fifo_sts_addr >> 32) & 0xFFl) << 8;
	temp |= (u32)((pamwifi_priv->sipa_info.dl_fifo.fifo_sts_addr >> 32) & 0xFFl) << 16;
	temp |= (u32)((pamwifi_priv->sipa_info.dl_fifo.fifo_sts_addr >> 32) & 0xFFl) << 24;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_AP_DL_FILLED_STS_BASE_ADDRH);
	wl_info("ipa dl sts addr:%lu, ul sts addr:%lu\n", pamwifi_priv->sipa_info.dl_fifo.fifo_sts_addr,
		    pamwifi_priv->sipa_info.ul_fifo.fifo_sts_addr);
}

/*****************************************************************************
 * Description: set tx fifo total depth.
 * Input:
 *   fifo_base: Need to set tx fifo empty status of the FIFO, the base
 *              address of the FIFO.
 * return:
 *   0: set tx fifo total depth successfully.
 *   -1: set tx fifo total_depth failed.
 * Note:
*****************************************************************************/
static inline u32 pam_wifi_set_tx_fifo_total_depth(u64 fifo_base, u32 depth)
{
	u32 tmp;

	if (depth > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_DEPTH));
		tmp &= 0x0000FFFFl;
		tmp |= (depth << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_DEPTH));
	}
	return 0;
}

/*****************************************************************************
 * Description: update tx fifo read pointer.
 * Input:
 *   fifo_base: Need to update tx fifo read pointer of the FIFO, the base
 *              address of the FIFO.
 * return:
 *   0: update tx fifo read pointer successfully.
 *   -1: update tx fifo read pointer failed.
 * Note:
*****************************************************************************/
static inline u32 pam_wifi_update_tx_fifo_rptr(u64 fifo_base, u32 rptr)
{
#if 0
	u32 tmp = 0;

	if (tmp > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_RD));
		tmp &= 0x0000FFFFl;
		tmp |= (rptr << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_RD));
	}
#endif
	return 0;
}

u32 pam_wifi_update_tx_fifo_wptr(u64 fifo_base, u32 wptr)
{
#if 0
	u32 tmp = 0;

	if (tmp > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_WR));
		tmp &= 0x0000FFFFl;
		tmp |= BIT_(1);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_WR));
		tmp |= (wptr << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_WR));
		tmp &= ~ BIT_(1);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_TX_FIFO_WR));
	}
#endif
	return 0;
}

static inline u32 pam_wifi_set_rx_fifo_total_depth(u64 fifo_base, u32 depth)
{
	u32 tmp;

	if (depth > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_DEPTH));
		tmp &= 0x0000FFFFl;
		tmp |= (depth << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_DEPTH));
	}
	return 0;
}

/*****************************************************************************
 * Description: update rx fifo read pointer.
 * Input:
 *   fifo_base: Need to update rx fifo read pointer of the FIFO, the base
 *              address of the FIFO.
 * return:
 *   TRUE: update rx fifo read pointer successfully,
 *   FALSE: update rx fifo read pointer failed.
 * Note:
*****************************************************************************/
static inline u32 pam_wifi_update_rx_fifo_rptr(u64 fifo_base, u32 rptr)
{
#if 0
	u32 tmp = 0;

	if (rptr > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_RD));
		tmp &= 0x0000FFFFl;
		tmp |= (rptr << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_RD));
	}
#endif
	return 0;
}

/*****************************************************************************
 * Description: update rx fifo write pointer.
 * Input:
 *   fifo_base: Need to update rx fifo write pointer of the FIFO, the base
 *              address of the FIFO.
 * return:
 *   TRUE: update rx fifo write pointer successfully,
 *   FALSE: update rx fifo write pointer failed.
 * Note:
*****************************************************************************/
static inline u32 pam_wifi_update_rx_fifo_wptr(u64 fifo_base, u32 wptr)
{
#if 0
	u32 tmp = 0;

	if (wptr > 0xFFFFl)
		return -1;
	else {
		tmp = readl_relaxed((void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_WR));
		tmp &= 0x0000FFFFl;
		tmp |= (wptr << 16);
		writel_relaxed(tmp, (void *)(fifo_base + PAMWIFI_COMMON_RX_FIFO_WR));
	}
#endif
	return 0;
}

void pamwifi_config_ipi(void)
{
	u32 temp = 0, ipi_mode = 1, ipi_ul1_base_addr_l = 0,	ipi_ul2_base_addr_l = 0,
		ipi_dl1_base_addr_l = 0, ipi_dl2_base_addr_l = 0;
	u8  ipi_ul1_base_addr_h = 0, ipi_ul2_base_addr_h = 0, ipi_dl1_base_addr_h = 0,
		ipi_dl2_base_addr_h = 0;

	if (pamwifi_priv->cp_cap.ipi_mode == 0) {
		ipi_mode = 1;
		ipi_ul1_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[0];
		ipi_ul1_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[0];
	} else if (pamwifi_priv->cp_cap.ipi_mode == 1) {
		ipi_mode = 2;
		ipi_dl1_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[0];
		ipi_dl1_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[0];
		ipi_ul1_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[1];
		ipi_ul1_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[1];
	} else if (pamwifi_priv->cp_cap.ipi_mode == 2) {
		ipi_mode = 4;
		ipi_dl1_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[0];
		ipi_dl1_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[0];
		ipi_ul1_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[1];
		ipi_ul1_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[1];
		ipi_dl2_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[2];
		ipi_dl2_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[2];
		ipi_ul2_base_addr_l = pamwifi_priv->cp_cap.ipi_reg_addr_l[3];
		ipi_ul2_base_addr_h = pamwifi_priv->cp_cap.ipi_reg_addr_h[3];
	}

	temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & ~0xE0l;
	temp |= ipi_mode << 5;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);

	writel_relaxed(ipi_ul1_base_addr_l, (void *)REG_PAM_WIFI_IPI_UL1_BASE_ADDRL);
	writel_relaxed(ipi_ul2_base_addr_l, (void *)REG_PAM_WIFI_IPI_UL2_BASE_ADDRL);
	writel_relaxed(ipi_dl1_base_addr_l, (void *)REG_PAM_WIFI_IPI_DL1_BASE_ADDRL);
	writel_relaxed(ipi_dl2_base_addr_l, (void *)REG_PAM_WIFI_IPI_DL2_BASE_ADDRL);
	temp = (u32)ipi_ul1_base_addr_h << 24;
	temp |= (u32)ipi_ul2_base_addr_h << 16;
	temp |= (u32)ipi_dl1_base_addr_h << 8;
	temp |= (u32)ipi_dl2_base_addr_h;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_IPI_BASE_ADDRH);

	/*REG_PAM_WIFI_IPI_UL1_BASE_WDATA*/
	writel_relaxed(2, (void *)REG_PAM_WIFI_IPI_UL1_BASE_WDATA);
	writel_relaxed(2, (void *)REG_PAM_WIFI_IPI_UL2_BASE_WDATA);
	writel_relaxed(2, (void *)REG_PAM_WIFI_IPI_DL1_BASE_WDATA);
	writel_relaxed(2, (void *)REG_PAM_WIFI_IPI_DL2_BASE_WDATA);
}

void pamwifi_sel_ac_ax_mode(u32 ac_ax_mode)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & ~0x4l;
	temp |= ac_ax_mode << 2;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);
}

void pamwifi_4in1_type4_overflow_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 13;
	temp |= dir << 29;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_4in1_type3_overflow_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 12;
	temp |= dir << 28;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_4in1_type2_overflow_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 11;
	temp |= dir << 27;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_4in1_type1_overflow_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 10;
	temp |= dir << 26;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_miss_tx_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 9;
	temp |= dir << 25;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_miss_rx_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 8;
	temp |= dir << 24;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_fill_type4_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 7;
	temp |= dir << 23;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_fill_type3_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 6;
	temp |= dir << 22;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_fill_type2_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 5;
	temp |= dir << 21;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_fill_type1_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 4;
	temp |= dir << 20;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_dl_fill_4in1_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 3;
	temp |= dir << 19;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}



void pamwifi_dl_free_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 2;
	temp |= dir << 18;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_ul_fill_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable << 1;
	temp |= dir << 17;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

void pamwifi_ul_free_en_dir(u32 enable, u32 dir)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_INT_EN);
	temp |= enable;
	temp |= dir << 16;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_EN);
}

/*****************************************************************************
 * ingroup  HWDRVR_COM
 * This function sets register specific field(1 or multiple bits) to all one
*****************************************************************************/
void set_reg_bits_all_one(u64 u4_addr, u32 mask)
{
	u32 u4_temp;
	/* Read the value into a local var */
	u4_temp = readl_relaxed((void *)u4_addr);

	/* set the value to one at bit positions */
	u4_temp |= mask;

	/* Set the correct value */
	writel_relaxed(u4_temp, (void *)u4_addr);
}

/*****************************************************************************
* ingroup  HWDRVR_COM
* This function wait register specific field till it changes to desired value, or timeout.
* Return FALSE if timeout, return TRUE if register changes to desired value before timeout.
*****************************************************************************/
bool wait_reg(u64 u4_addr, u32 u4_mask, u32 u4_lsb, u32 u4_value, u32 u4_microsec_timeout)
{
	u32 u4_temp, u4_timeout_cnt , u4_temp_cnt =0;
	u32 u4_worst_timeout_cnt = 100000;/*worst timeout count =10^5 , 10us per one, total 1s*/

	u4_timeout_cnt = u4_worst_timeout_cnt;
	u4_temp = ((u4_value << u4_lsb) & u4_mask);

	while (u4_temp != (readl_relaxed((void *)u4_addr) & u4_mask))
	{
		if(u4_temp_cnt >= u4_timeout_cnt)
			{
				if(u4_temp_cnt >= u4_worst_timeout_cnt)
					{
						//SCI_ASSERT(0);
						//PAM_WIFI_LOG("wait_reg return FALSE!!\n");
            				}
				return false;
			}
		//add_calib_delay(1);/*10us*/
		u4_temp_cnt ++;
	}
	return true;
}

/*****************************************************************************
 * \ingroup  HWDRVR_COM
 * This function clears register specific field to all zero
*****************************************************************************/
void clear_reg_bits(u64 u4_addr, u32 mask)
{
	u32 u4_temp;

	/* Read the value into a local var */
	u4_temp = readl_relaxed((void *)u4_addr);

	/* clear the value at bit positions */
	u4_temp &= ~mask;

	/* Set the correct value */
	writel_relaxed(u4_temp, (void *)u4_addr);
}

bool pamwifi_table_lock()
{
	set_reg_bits_all_one(REG_PAM_WIFI_CFG_START, BIT_PAM_WIFI_CFG_START_SOFT_TABLE_UPDATE_REQ);
	if (false == wait_reg(REG_PAM_WIFI_CFG_START, BIT_PAM_WIFI_CFG_START_PAM_WIFI_TABLE_RD_STOP,
						BIT_PAM_WIFI_CFG_START_PAM_WIFI_TABLE_RD_STOP_EN_MSB, 1, PAM_WIFI_HW_LOCK_TO))
		{
			clear_reg_bits(REG_PAM_WIFI_CFG_START, BIT_PAM_WIFI_CFG_START_SOFT_TABLE_UPDATE_REQ);
			wl_info("pam_wifi_table_update access lock fail !!\n");
			return false;
		}
	return true;
}

bool pamwifi_table_unlock()
{
	clear_reg_bits(REG_PAM_WIFI_CFG_START, BIT_PAM_WIFI_CFG_START_SOFT_TABLE_UPDATE_REQ);
	return true;
}

void pamwifi_update_router_table_test(void)
{
	u32 temp;
	u8 da[6] = {0x40, 0x45, 0xda, 0xaa, 0xbb, 0xcc};

	if(pamwifi_table_lock() == FALSE)
        {
            wl_err("%s pam_wifi_table_lock FALSE\n", __func__);
        }
    else
        {
            wl_info("%s pam_wifi_table_lock TRUE\n", __func__);
        }

	/*consider uc w2w pkt, sa maybe not be marlin3 self mac addr, so only set da*/
	temp = 0;
	writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x00 + 16 * 0 ));

	temp = (u32)( (da[0] << 16) | (da[1] << 24));
	writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x04 + 16 * 0 ));

	temp = (u32)(da[2] | (da[3] << 8) | (da[4] << 16) |(da[5] << 24));
	writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x08 + 16 * 0));

	temp = (6 & 0x3FF) | ((0 & 0x7) << 10) | ((0 & 0xFF) << 24);
	writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x0C + 16 * 0 ));


	pamwifi_table_unlock();
}

static void pamwifi_set_enable(bool enable)
{
	int read_on_reg;

	if(enable) {
		read_on_reg = __raw_readl(pamwifi_priv->pamwifi_glb_base);
		read_on_reg |= 0x40l;
		__raw_writel(read_on_reg, pamwifi_priv->pamwifi_glb_base);
	} else {
		read_on_reg = __raw_readl(pamwifi_priv->pamwifi_glb_base);
		read_on_reg &= ~(0x40l);
		__raw_writel(read_on_reg, pamwifi_priv->pamwifi_glb_base);
	}
}

void pamwifi_update_router_table(struct sprdwl_priv *priv, struct sprdwl_sta_lut_ind *sta_lut,
									   u8 vif_mode, u32 index, int flag)
{
	struct sprdwl_vif *vif;
	struct sprdwl_intf *intf;
	int i;
	u8 router_lut, ram_index, select_ram;
	u32 temp;

	vif = mode_to_vif(priv, vif_mode);
	intf = (struct sprdwl_intf *)vif->priv->hw_priv;
	router_lut = sta_lut->sta_lut_index - 6;
	ram_index = router_lut / 2;
	select_ram = router_lut % 2;

	/*check ctx_id*/
	if (vif->ctx_id != sta_lut->ctx_id) {
		wl_err("ctx_id do not match!\n");
		return;
	}

	pamwifi_set_enable(true);

	if(pamwifi_table_lock() == FALSE)
	{
		wl_err("%s pam_wifi_table_lock FALSE\n", __func__);
	}
	else
	{
		wl_info("%s pam_wifi_table_lock TRUE\n", __func__);
	}

	if (flag == 0) {
		if (!select_ram) {
			/*temp = (u32)(sta_lut->ra[0] |(sta_lut->ra[1] << 8) |(sta_lut->ra[2] << 16) |
					(sta_lut->ra[3] << 24));*/
			temp = 0x0l;
			writel_relaxed(temp, (void *)(PSEL_RAM1 + 16 * ram_index));
			/*backup*/
			pamwifi_priv->router_table_backup[router_lut][0] = temp;

			/*consider uc w2w pkt, sa maybe not be marlin3 self mac addr, so only set da*/
			temp = (u32)((sta_lut->ra[0] << 16) |(sta_lut->ra[1]<< 24));
			writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x04 + 16 * ram_index ));
			pamwifi_priv->router_table_backup[router_lut][1] = temp;

			temp = (u32)(sta_lut->ra[2] | (sta_lut->ra[3] << 8) | (sta_lut->ra[4] << 16) |
				(sta_lut->ra[5] << 24));
			writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x08 + 16 * ram_index ));
			pamwifi_priv->router_table_backup[router_lut][2] = temp;

			temp = (sta_lut->sta_lut_index & 0x3FF) | ((vif->ctx_id & 0x7) << 10) | ((index & 0xFF) << 24);
			writel_relaxed(temp, (void *)(PSEL_RAM1 + 0x0C + 16 * ram_index ));
			pamwifi_priv->router_table_backup[router_lut][3] = temp;
		} else {
			/*temp = (u32)(sta_lut->ra[0] |(sta_lut->ra[1] << 8) |(sta_lut->ra[2] << 16) |
					(sta_lut->ra[3] << 24));*/
			temp = 0x0l;
			writel_relaxed(temp, (void *)(PSEL_RAM2 + 16 * ram_index));
			/*backup*/
			pamwifi_priv->router_table_backup[router_lut][0] = temp;

			temp = (u32)((sta_lut->ra[0] << 16) |(sta_lut->ra[1]<< 24));
			writel_relaxed(temp, (void *)(PSEL_RAM2 + 0x04 + 16 * ram_index ));
			/*backup*/
			pamwifi_priv->router_table_backup[router_lut][1] = temp;

			temp = (u32)(sta_lut->ra[2] | (sta_lut->ra[3] << 8) | (sta_lut->ra[4] << 16) |
				(sta_lut->ra[5] << 24));
			writel_relaxed(temp, (void *)(PSEL_RAM2 + 0x08 + 16 * ram_index ));
			/*backup*/
			pamwifi_priv->router_table_backup[router_lut][2] = temp;

			temp = (sta_lut->sta_lut_index & 0x3FF) | ((vif->ctx_id & 0x7) << 10) | ((index & 0xFF) << 24);
			writel_relaxed(temp, (void *)(PSEL_RAM2 + 0x0C + 16 * ram_index ));
			/*backup*/
			pamwifi_priv->router_table_backup[router_lut][3] = temp;
		}
	}else if (flag == 1) {
		if (!select_ram) {
			writel_relaxed(0x00000000, (void *)(PSEL_RAM1 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM1 + 0x04 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM1 + 0x08 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM1 + 0x0C + 16 * ram_index));
		} else {
			writel_relaxed(0x00000000, (void *)(PSEL_RAM2 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM2 + 0x04 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM2 + 0x08 + 16 * ram_index));
			writel_relaxed(0x00000000, (void *)(PSEL_RAM2 + 0x0C + 16 * ram_index));
		}
		for(i = 0; i < 4; i++)
			pamwifi_priv->router_table_backup[router_lut][i] = 0;
	}
	pamwifi_table_unlock();
}

void pamwifi_ac_msdu_init(u8 msdu_index)
{
#define MSDU_HEADER_LEN 16
#define SPRDWL_TYPE_DATA 2
	struct tx_msdu_dscr  *tmp_msdu_header;
	unsigned char *msdu_dscr;
	int i = 0, j = 0;

	if (!pam_wifi_msdu_header_info) {
		wl_err("pamwifi msdu dma_alloc_coherent err\n");
		return;
	}

	pamwifi_sel_ac_ax_mode(1);
	msdu_dscr = (unsigned char *)pam_wifi_msdu_header_info;
	memset((u8 *)msdu_dscr + MSDU_HEADER_LEN * msdu_index, 0x00, MSDU_HEADER_LEN);
	tmp_msdu_header = (struct tx_msdu_dscr *)(msdu_dscr + 5);

	((struct tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->common.type = SPRDWL_TYPE_DATA;
	((struct tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->offset = DSCR_LEN;
	((struct tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->tx_ctrl.pcie_mh_readcomp = 1;
}

void pamwifi_ax_msdu_init(u8 msdu_index)
{
#define MSDU_HEADER_LEN 16
#define SPRDWL_TYPE_DATA 2
	struct ax_tx_msdu_dscr  *tmp_msdu_header;
	unsigned char *msdu_dscr;
	int i = 0, j = 0;

	if (!pam_wifi_msdu_header_info) {
		wl_err("pamwifi msdu dma_alloc_coherent err\n");
		return;
	}

	pamwifi_sel_ac_ax_mode(0);
	msdu_dscr = (unsigned char *)pam_wifi_msdu_header_info;
	memset((u8 *)msdu_dscr + MSDU_HEADER_LEN * msdu_index, 0x00, MSDU_HEADER_LEN);
	tmp_msdu_header = (struct ax_tx_msdu_dscr *)(msdu_dscr + 5);

	((struct ax_tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->common.type = SPRDWL_TYPE_DATA;
	((struct ax_tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->offset = DSCR_LEN;
	((struct ax_tx_msdu_dscr  *)((char *)tmp_msdu_header + (i * 4 + j) * MSDU_HEADER_LEN))->tx_ctrl.pcie_mh_readcomp = 1;

}

int pamwifi_config_type1_4_common_fifo(struct platform_device *pdev, u32 pamwifi_tx_intr_threshold,
											      u32 pamwifi_tx_intr_threshold_en, u32 pamwifi_tx_intr_threshold_delay,
											      u32 pamwifi_tx_intr_threshold_delay_en, u32 pamwifi_fifo_depth)
{
	u32 temp_threshold_value = 0;

	/*PSEL_DL_TYPE1*/
	if (!dl_type1_virt_addr)
		dl_type1_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth*sizeof(dma_addr_t), &dl_type1_phy_addr, GFP_KERNEL);
	if (!dl_type1_virt_addr) {
		wl_err("dl_type1_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(dl_type1_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_TYPE1 + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(dl_type1_phy_addr >> 32),
				(void *)(PSEL_DL_TYPE1 + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_TYPE1, pamwifi_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE1 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE1 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE1 + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE1 + PAMWIFI_INT_GEN_CTL_EN));


	/*PSEL_DL_TYPE2*/
	if (!dl_type2_virt_addr)
		dl_type2_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth*sizeof(dma_addr_t), &dl_type2_phy_addr, GFP_KERNEL);
	if (!dl_type2_virt_addr) {
		wl_err("dl_type2_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(dl_type2_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_TYPE2 + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(dl_type2_phy_addr >> 32),
				(void *)(PSEL_DL_TYPE2 + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_TYPE2, pamwifi_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE2 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE2 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE2 + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE2 + PAMWIFI_INT_GEN_CTL_EN));

	/*PSEL_DL_TYPE3*/
	if (!dl_type3_virt_addr)
		dl_type3_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth * sizeof(dma_addr_t),
											 &dl_type3_phy_addr, GFP_KERNEL);
	if (!dl_type3_virt_addr) {
		wl_err("dl_type3_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(dl_type3_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_TYPE3 + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(dl_type3_phy_addr >> 32),
				(void *)(PSEL_DL_TYPE3 + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_TYPE3, pamwifi_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE3 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE3 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE3 + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE3 + PAMWIFI_INT_GEN_CTL_EN));

	/*PSEL_DL_TYPE4*/
	if (!dl_type4_virt_addr)
		dl_type4_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth*sizeof(dma_addr_t),
											 &dl_type4_phy_addr, GFP_KERNEL);
	if (!dl_type4_virt_addr) {
		wl_err("dl_type4_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(dl_type4_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_TYPE4 + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(dl_type4_phy_addr >> 32),
				(void *)(PSEL_DL_TYPE4 + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_TYPE4, pamwifi_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE4 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE4 + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_TYPE4 + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_TYPE4+PAMWIFI_INT_GEN_CTL_EN));

	/*set intr dir*/
	pamwifi_dl_fill_type1_en_dir(1, 0);
	pamwifi_dl_fill_type2_en_dir(1, 0);
	pamwifi_dl_fill_type3_en_dir(1, 0);
	pamwifi_dl_fill_type4_en_dir(1, 0);

	return 0;
}

int sprdwl_pamwifi_fill_rx_buf(struct sprdwl_intf *intf)
{
	int i;
	u32 temp;

	skb_queue_head_init(&pamwifi_priv->buffer_list);
	for(i = 0; i < 500; i++) {
		unsigned long pcie_addr = 0;
		struct sk_buff *skb = NULL;

		skb = dev_alloc_skb(SPRDWL_MAX_DATA_RXLEN);
		if (skb) {
			/*TODO: if parts of skb alloc succ?*/
			skb_queue_tail(&pamwifi_priv->buffer_list, skb);
			SAVE_ADDR(skb->data, skb, sizeof(skb));
			pcie_addr = mm_virt_to_phys(&intf->pdev->dev,
					    skb->data, skb->len,
					    DMA_FROM_DEVICE);
			*((unsigned long *)ul_tx_virt_addr + i) = pcie_addr;
			if(i < 20)
				wl_info("ul tx pcie_addr1:%lu\n", pcie_addr);
		} else {
			wl_err("%s, alloc %d skb fail\n", __func__, i);
		}
	}

	/*send rx buf to cp2*/
	temp = readl_relaxed((void *)REG_PAM_WIFI_CFG_START);
	if ((temp & 0x1l) == 1) {
		temp &= ~0x1;
		/*disable pamwifi*/
		writel_relaxed(temp, (void *)REG_PAM_WIFI_CFG_START);
	}
	/*to be modified*/
	pam_wifi_update_tx_fifo_wptr(PSEL_UL, i);
	/*start pamwifi*/
	temp |= 0x1l;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_CFG_START);

	return 0;
}

/*init dl free/ul filled/ul free common fifo*/
int pamwifi_config_others_common_fifo(struct platform_device *pdev, u32 pamwifi_tx_intr_threshold,
											   u32 pamwifi_tx_intr_threshold_en, u32 pamwifi_tx_intr_threshold_delay,
											   u32 pamwifi_tx_intr_threshold_delay_en, u32 pamwifi_fifo_depth)
{
	u32 temp_threshold_value = 0, pamwifi_tx_wrptr, pamwifi_tx_rdptr;

	/*dl free*/
	if (!dl_free_virt_addr)
		dl_free_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth*sizeof(dma_addr_t), &dl_free_phy_addr, GFP_KERNEL);
	if (!dl_free_virt_addr) {
		wl_err("dl_free_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(dl_free_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_ADDRL));
	writel_relaxed((u32)(dl_free_phy_addr >> 32),
				(void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_ADDRH));
	pam_wifi_set_rx_fifo_total_depth(PSEL_DL_FREE, pamwifi_fifo_depth);

	/*miss tx*/
	if (!miss_tx_virt_addr)
		miss_tx_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth * sizeof(dma_addr_t),
										      &miss_tx_phy_addr, GFP_KERNEL);
	if (!miss_tx_virt_addr) {
		wl_err("miss_tx_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(miss_tx_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(miss_tx_phy_addr >> 32),
				(void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_MISS, pamwifi_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_MISS+PAMWIFI_INT_GEN_CTL_EN));

	/*miss rx*/
	if (!miss_rx_virt_addr)
		miss_rx_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth * sizeof(dma_addr_t),
										      &miss_rx_phy_addr, GFP_KERNEL);
	if (!miss_rx_virt_addr) {
		wl_err("miss_rx_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(miss_rx_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_ADDRL));
	writel_relaxed((u32)(miss_rx_phy_addr >> 32),
				(void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_ADDRH));
	pam_wifi_set_rx_fifo_total_depth(PSEL_DL_MISS, pamwifi_fifo_depth);

	/*ul filled*/
	if (!ul_rx_virt_addr)
		ul_rx_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth * sizeof(dma_addr_t),
										  &ul_rx_phy_addr, GFP_KERNEL);
	if (!ul_rx_virt_addr) {
		wl_err("ul_rx_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(ul_rx_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_ADDRL));
	writel_relaxed((u32)(ul_rx_phy_addr >> 32),
				(void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_ADDRH));
	pam_wifi_set_rx_fifo_total_depth(PSEL_UL, pamwifi_fifo_depth);

	/*ul free*/
	if (!ul_tx_virt_addr)
		ul_tx_virt_addr = dma_alloc_coherent(&pdev->dev, pamwifi_fifo_depth * sizeof(dma_addr_t),
										  &ul_tx_phy_addr, GFP_KERNEL);
	if (!ul_tx_virt_addr) {
		wl_err("ul_tx_virt_addr alloc failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)(ul_tx_phy_addr & 0xFFFFFFFFl),
				(void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)(ul_tx_phy_addr >> 32),
				(void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_UL, pamwifi_fifo_depth);

	/*to reset tx rdptr*/
	pamwifi_tx_wrptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_tx_rdptr = readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_RD)) & 0xFFFFl;
	pamwifi_tx_rdptr |= pamwifi_tx_wrptr << 16;
	writel_relaxed(pamwifi_tx_rdptr, (void*)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_RD));

	pamwifi_tx_wrptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_tx_rdptr = readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_RD)) & 0xFFFFl;
	pamwifi_tx_rdptr |= pamwifi_tx_wrptr << 16;
	writel_relaxed(pamwifi_tx_rdptr, (void*)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_RD));

	temp_threshold_value = readl_relaxed((void *)(PSEL_UL + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_UL + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_UL + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_UL+PAMWIFI_INT_GEN_CTL_EN));

	/*set intr dir*/
	pamwifi_dl_free_en_dir(0, 0);
	pamwifi_dl_miss_tx_en_dir(1, 1);
	pamwifi_dl_miss_rx_en_dir(0, 1);
	pamwifi_ul_fill_en_dir(0, 0);
	pamwifi_ul_free_en_dir(1, 0);

	return 0;
}

int pamwifi_config_4in1_common_fifo(struct platform_device *pdev, u32 pamwifi_tx_intr_threshold,
										      u32 pamwifi_tx_intr_threshold_en, u32 pamwifi_tx_intr_threshold_delay,
										      u32 pamwifi_tx_intr_threshold_delay_en, u32 pamwifi_fifo_depth)
{
	u32 temp_threshold_value = 0;

	writel_relaxed((u32)(pamwifi_priv->cp_cap.mux_tx_common_fifo_base_addr_l & 0xFFFFFFFFl),
				(void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_ADDRL));
	writel_relaxed((u32)pamwifi_priv->cp_cap.mux_tx_common_fifo_base_addr_h,
				(void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_ADDRH));
	pam_wifi_set_tx_fifo_total_depth(PSEL_DL_FREE,
								    pamwifi_priv->cp_cap.mux_tx_common_fifo_depth);
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value |= pamwifi_tx_intr_threshold;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_FREE + PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE));
	temp_threshold_value = readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_INT_GEN_CTL_EN));
	temp_threshold_value |= pamwifi_tx_intr_threshold_en;
	temp_threshold_value |= pamwifi_tx_intr_threshold_delay_en;
	writel_relaxed(temp_threshold_value, (void *)(PSEL_DL_FREE + PAMWIFI_INT_GEN_CTL_EN));

	/*set intr dir*/
	pamwifi_dl_fill_4in1_en_dir(1, 0);

	return 0;
}

void pamwifi_4in1_overflow_conf(u32 enable_overflow, u32 timescale,
									  u32 type1_threshold, u32 type2_threshold,
									  u32 type3_threshold, u32 type4_threshold)
{
	u32 temp;

	temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & (~0x10l);
	temp |= enable_overflow << 4;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);

	temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & (~0xFF0000l);
	temp |= timescale << 16;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);

	temp = type1_threshold | (type2_threshold << 8) |
		(type3_threshold << 16) | (type4_threshold << 24);
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_FLOW_THRESHOLD);
}

u32 pamwifi_common_configuration(struct platform_device *pdev,
										  u8 enable_4in1, u32 pamwifi_tx_intr_threshold,
										  u32 pamwifi_tx_intr_threshold_en,
										  u32 pamwifi_tx_intr_threshold_delay,
										  u32 pamwifi_tx_intr_threshold_delay_en,
										  u32 pamwifi_fifo_depth, u32 value_index_search_depth)
{
	u32 temp = 0;

	/*Setting IPA PAM WIFI fifo sts*/
	if (enable_4in1 == 1) {
		//enable 4in1
		temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & ~0x1l;
		temp |= 1;
		writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);
		pamwifi_config_4in1_common_fifo(pdev, pamwifi_tx_intr_threshold,
			pamwifi_tx_intr_threshold_en, pamwifi_tx_intr_threshold_delay,
			pamwifi_tx_intr_threshold_delay_en, pamwifi_fifo_depth);
		/*default disable overflow*/
		pamwifi_4in1_overflow_conf(0, 0xal, 0xFFl, 0xFFl, 0xFFl, 0xFFl);
	} else {
		/*disable 4in1*/
		temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & ~0x1l;
		temp |= 0;
		writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);
		pamwifi_config_type1_4_common_fifo(pdev, pamwifi_tx_intr_threshold,
			pamwifi_tx_intr_threshold_en, pamwifi_tx_intr_threshold_delay,
			pamwifi_tx_intr_threshold_delay_en, pamwifi_fifo_depth);
	}
	pamwifi_config_others_common_fifo(pdev, pamwifi_tx_intr_threshold,
		pamwifi_tx_intr_threshold_en, pamwifi_tx_intr_threshold_delay,
		pamwifi_tx_intr_threshold_delay_en, pamwifi_fifo_depth);

	/*index search depth*/
	temp = readl_relaxed((void *)REG_PAM_WIFI_INDEX_SEARCH_INDEX) & ~0x7Fl;
	temp = value_index_search_depth;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_INDEX_SEARCH_INDEX);

	temp = readl_relaxed((void *)REG_PAM_WIFI_CFG_DL_FILLED_BUFFER_CTRL) & 0xFE000F00l;
	/*set dl net_id/dst_id*/
	temp |= 0x00l << 17;
	temp |= SIPA_TERM_WIFI << 12;
	/*set watermark*/
	temp |= BIT_PAM_WIFI_CFG_DL_CP_FILLED_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_CP_MISS_BUFFER_WATERMARK(1);
	writel_relaxed(temp, (void *)REG_PAM_WIFI_CFG_DL_FILLED_BUFFER_CTRL);

	temp = 0;
	temp |= BIT_PAM_WIFI_CFG_DL_AP_FILLED_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_AP_FREE_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_CP_TYPE1_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_CP_TYPE2_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_CP_TYPE3_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_DL_CP_TYPE4_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_UL_CP_FREE_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_UL_AP_FREE_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_UL_CP_FILLED_BUFFER_WATERMARK(1);
	temp |= BIT_PAM_WIFI_CFG_UL_AP_FILLED_BUFFER_WATERMARK(1);
	writel_relaxed(temp, (void *)REG_PAM_WIFI_CFG_BUFFER_WATERMARK);

	/*set ddr mapping offset addr, match REG_PAM_WIFI_CFG_DL_FILLED_BUFFER_CTRL*/
	temp = 0x80l | (0x80l << 8) | (0x80l << 16) | (0x80l << 24);
	writel_relaxed(temp, (void *)REG_PAM_WIFI_DDR_MAPPING_OFFSET_H);

	/*Configure the msdu_header_buf table*/
	if (!pam_wifi_msdu_header_info)
		pam_wifi_msdu_header_info = dma_alloc_coherent(&pdev->dev, 8*4*16,
													  &term_pam_wifi_msdu_header_buf,
													  GFP_KERNEL);
	if (!pam_wifi_msdu_header_info) {
		wl_err("term_pam_wifi_msdu_header_buf alloc buffer failed!\n");
		return -ENOMEM;
	}
	writel_relaxed((u32)((u64)term_pam_wifi_msdu_header_buf & 0xFFFFFFFFl),
				(void *)REG_PAM_WIFI_MSDU_ADDR_L);
	temp = readl_relaxed((void *)REG_PAM_WIFI_RF_DL_CFG) & 0xFFFFFFl;
	temp |= (u32)(((u64)term_pam_wifi_msdu_header_buf) >> 32) << 24;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_RF_DL_CFG);

	/*set ul ipa node info*/
	temp = readl_relaxed((void *)REG_PAM_WIFI_UL_NODE_INFO_CONFIG) & 0xFFFC0000;
	temp |= (u32)SIPA_TERM_WIFI;
	temp |= 0xFF << 10;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_UL_NODE_INFO_CONFIG);

	return 0;
}

int sprdwl_pamwifi_get_node_dscr(u32 fifo_depth)
{
	u32 tx_wrptr, tx_rdptr, free_num, temp_wrptr, temp_rdptr, temp;
	unsigned long flags = 0;
	int i;
	struct sprdwl_pamwifi_msg_buf *pamwifi_msg_buf = NULL;

	tx_wrptr = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16;
	tx_rdptr = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_RD)) >> 16;
	temp_wrptr = tx_wrptr % fifo_depth;
	temp_rdptr = tx_rdptr % fifo_depth;
	if (temp_wrptr >= temp_rdptr) {
		free_num = temp_wrptr - temp_rdptr;
		for (i = 0; i < free_num; i++) {
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			if (!list_empty(&pamwifi_priv->pamwifi_msg_list->free_list)) {
				pamwifi_msg_buf = list_first_entry(&pamwifi_priv->pamwifi_msg_list->free_list,
											   struct sprdwl_pamwifi_msg_buf, list);
				list_del(&pamwifi_msg_buf->list);
			} else {
				wl_err("no more miss buffer\n");
				BUG_ON(1);
			}
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			memcpy(&pamwifi_msg_buf->dscr, miss_tx_virt_addr + tx_rdptr + i,
				      sizeof(struct pamwifi_miss_node_tx_dscr));
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
			if (list_empty(&pamwifi_priv->pamwifi_msg_list->free_list))
				INIT_LIST_HEAD(&pamwifi_priv->pamwifi_msg_list->busy_list);
			list_add_tail(&pamwifi_msg_buf->list, &pamwifi_priv->pamwifi_msg_list->busy_list);
			atomic_inc(&pamwifi_priv->pamwifi_msg_list->ref);
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
		}

		complete(&pamwifi_priv->tx_completed);
	} else if (temp_wrptr < temp_rdptr) {
		free_num = temp_wrptr + fifo_depth - temp_rdptr;
		temp = fifo_depth - temp_rdptr - 1;
		for (i = 0; i < temp; i++) {
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			if (!list_empty(&pamwifi_priv->pamwifi_msg_list->free_list)) {
				pamwifi_msg_buf = list_first_entry(&pamwifi_priv->pamwifi_msg_list->free_list,
											   struct sprdwl_pamwifi_msg_buf, list);
				list_del(&pamwifi_msg_buf->list);
			} else {
				wl_err("no more miss buffer\n");
				BUG_ON(1);
			}
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			memcpy(&pamwifi_msg_buf->dscr, miss_tx_virt_addr + tx_rdptr + i,
				      sizeof(struct pamwifi_miss_node_tx_dscr));
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
			if (list_empty(&pamwifi_priv->pamwifi_msg_list->free_list))
				INIT_LIST_HEAD(&pamwifi_priv->pamwifi_msg_list->busy_list);
			list_add_tail(&pamwifi_msg_buf->list, &pamwifi_priv->pamwifi_msg_list->busy_list);
			atomic_inc(&pamwifi_priv->pamwifi_msg_list->ref);
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
		}
		temp = free_num - temp;
		for (i = 0; i < temp; i++) {
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			if (!list_empty(&pamwifi_priv->pamwifi_msg_list->free_list)) {
				pamwifi_msg_buf = list_first_entry(&pamwifi_priv->pamwifi_msg_list->free_list,
											   struct sprdwl_pamwifi_msg_buf, list);
				list_del(&pamwifi_msg_buf->list);
			} else {
				wl_err("no more miss buffer\n");
				BUG_ON(1);
			}
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
			memcpy(&pamwifi_msg_buf->dscr, miss_tx_virt_addr + i,
					sizeof(struct pamwifi_miss_node_tx_dscr));
			spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
			if (list_empty(&pamwifi_priv->pamwifi_msg_list->free_list))
				INIT_LIST_HEAD(&pamwifi_priv->pamwifi_msg_list->busy_list);
			list_add_tail(&pamwifi_msg_buf->list, &pamwifi_priv->pamwifi_msg_list->busy_list);
			atomic_inc(&pamwifi_priv->pamwifi_msg_list->ref);
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
		}
		complete(&pamwifi_priv->tx_completed);
	}

	temp = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_RD)) & 0xFFFFl;
	tx_rdptr = (tx_rdptr + free_num) % (2 * fifo_depth);
	temp |= (u32)(tx_rdptr << 16);
	writel_relaxed(temp, (void *)(PSEL_DL_MISS + PAMWIFI_COMMON_TX_FIFO_RD));

	return 0;
}

irqreturn_t pamwifi_irq_handle(int irq, void *dev)
{
	u32 int_sts, cmn_fifo_intr_sts, temp;

	wl_err("%s, %d, enter!\n", __func__, __LINE__);
	int_sts = readl_relaxed((void *)REG_PAM_WIFI_INT_STS) & BIT_(9);
	cmn_fifo_intr_sts = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_EN)) & 0x300l;
	if (int_sts != 0 && cmn_fifo_intr_sts != 0) {
		sprdwl_pamwifi_get_node_dscr(1024);
		/*clr intr & sts*/
		temp = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_CLR));
		temp |= 0x3l;
		writel_relaxed(temp, (void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_CLR));
		temp &= ~0x3l;
		writel_relaxed(temp, (void *)(PSEL_DL_MISS + PAMWIFI_INT_GEN_CTL_CLR));

		temp = readl_relaxed((void *)REG_PAM_WIFI_INT_CLR);
		temp |= ~0x3E00l;
		writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_CLR);
		temp &= ~0x3E00l;
		writel_relaxed(temp, (void *)REG_PAM_WIFI_INT_CLR);
	} else {
		wl_err("int_sts:%lu, cmn_fifo_intr_sts:%lu, intr err!\n", int_sts, cmn_fifo_intr_sts);
	}

	/*tmp debug*/
	writel_relaxed(BIT_(9), (void *)REG_PAM_WIFI_INT_CLR);

	wl_err("%s, %d, exit!\n", __func__, __LINE__);
	return IRQ_HANDLED;
}

extern struct sprdwl_intf_sc2355 g_intf_sc2355;
struct sprdwl_vif *sprdwl_find_miss_vif(struct sk_buff *skb)
{
	struct sprdwl_intf *intf = (struct sprdwl_intf *)g_intf_sc2355.intf;
	int i;

	for (i = 0; i < MAX_LUT_NUM; i++) {
		if (0 == memcmp(intf->peer_entry[i].tx.sa, skb->data + 6, ETH_ALEN)/* ||
		    0 == memcmp(intf->peer_entry[i].tx.da, skb->data + 6, ETH_ALEN)*/) {
			return intf->peer_entry[i].vif;
		}
	}
	sprdwl_hex_dump("cant find vif, dump skb:", skb->data, 100);
	return NULL;
}

struct sprdwl_vif *sprdwl_find_ul_vif(struct sk_buff *skb)
{
	struct sprdwl_intf *intf = (struct sprdwl_intf *)g_intf_sc2355.intf;
	struct net_device *net;
	int i, j;

	for (i = 0; i < MAX_LUT_NUM; i++) {
		if (0 == memcmp(intf->peer_entry[i].tx.sa, skb->data, ETH_ALEN)) {
			wl_info("%s, %d\n", __func__, __LINE__);
			return intf->peer_entry[i].vif;
		} else if (0 == memcmp(intf->peer_entry[i].tx.da, skb->data + 6, ETH_ALEN)) {
			wl_info("%s, %d\n", __func__, __LINE__);
			net = intf->peer_entry[i].vif->ndev;
			if(!net)
				return NULL;
			for (j = 0; j < MAX_LUT_NUM; j++) {
				if (0 == memcmp(intf->peer_entry[j].tx.da, skb->data, ETH_ALEN)) {
					sprdwl_xmit_to_ipa_pamwifi(skb, net);
					return NULL;
				}
			}
			return intf->peer_entry[i].vif;
		}
	}
	sprdwl_hex_dump("vif is null, dump skb:", skb->data, 100);
	dev_kfree_skb(skb);
	return NULL;
}

/*retrieve miss node*/
int sprdwl_pamwifi_retrieve_buf(struct pamwifi_miss_node_rx_dscr *dscr, u32 index,
									 u32 wrptr, u32 rdptr, u32 fifo_depth)
{
	if ((wrptr + index + 1) > fifo_depth)
		memcpy(miss_rx_virt_addr + wrptr + index - fifo_depth, dscr,
			     sizeof(struct pamwifi_miss_node_rx_dscr));
	else
		memcpy(miss_rx_virt_addr + wrptr + index + 1, dscr,
			     sizeof(struct pamwifi_miss_node_rx_dscr));

	return 0;
}

extern struct sk_buff *sipa_find_sent_skb(u64 addr);
void sprdwl_analyze_pamwifi_miss_node(void)
{
	struct pamwifi_miss_node_rx_dscr rx_node;
	struct sprdwl_pamwifi_msg_buf *pamwifi_msg_buf = NULL;
	//struct sk_buff *skb = NULL, *tmp_skb = NULL;
	//struct sprdwl_vif *vif = NULL;
	unsigned long flags = 0, i;
	int num;
	u32 rx_rdptr, rx_wrptr, free_num, temp_wrptr, temp_rdptr, fifo_depth = 1024, temp;

	wl_err("%s\n", __func__);
	if (pamwifi_priv->suspend_stage & PAMWIFI_EB_SUSPEND) {
		wl_err("%s, Pam wifi already disabled!", __func__);
		return;
	}
	num = atomic_read(&pamwifi_priv->pamwifi_msg_list->ref);
	rx_wrptr = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_WR)) >> 16;
	rx_rdptr = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_RD)) >> 16;
	if (num <= 0)
		return;
	temp_wrptr = rx_wrptr % fifo_depth;
	temp_rdptr = rx_rdptr % fifo_depth;
	if (temp_wrptr >= temp_rdptr) {
		free_num = temp_rdptr + fifo_depth - temp_wrptr;
	} else if (temp_wrptr < temp_rdptr) {
		free_num = temp_rdptr - temp_wrptr;
	}

	if (num < free_num)
		free_num = num;

	for(i = 0; i < free_num; i++) {
		spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
		if (!list_empty(&pamwifi_priv->pamwifi_msg_list->busy_list)) {
			pamwifi_msg_buf = list_first_entry(&pamwifi_priv->pamwifi_msg_list->busy_list,
										   struct sprdwl_pamwifi_msg_buf, list);
			list_del(&pamwifi_msg_buf->list);
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
			/*todo*/
#if 0
			/*overflow pkt free directly, others send by special data*/
			if (!pamwifi_msg_buf->dscr.flag) {
				/*TODO by ipa*/
				skb = sipa_find_sent_skb(pamwifi_msg_buf->dscr.address);
				tmp_skb = skb_copy(skb, (GFP_DMA | GFP_ATOMIC));
				vif = sprdwl_find_miss_vif(tmp_skb);
				if (!vif) {
					dev_kfree_skb(tmp_skb);
					continue;
				}
				if (sprdwl_xmit_data2cmd_wq(tmp_skb, vif->ndev) == -EAGAIN)
					/*TODO: if miss pkt send fail?*/
					continue;
			}
#endif
			rx_node.address = pamwifi_msg_buf->dscr.address;
			rx_node.length = pamwifi_msg_buf->dscr.length;
			rx_node.offset = pamwifi_msg_buf->dscr.address;
			rx_node.src_id = pamwifi_msg_buf->dscr.src_id;
			rx_node.tos = pamwifi_msg_buf->dscr.tos;
			rx_node.flag = pamwifi_msg_buf->dscr.flag;
		} else {
			spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->busy_lock, flags);
		}
		/*free miss node*/
		sprdwl_pamwifi_retrieve_buf(&rx_node, i, temp_wrptr, temp_rdptr, fifo_depth);
	}

	temp = readl_relaxed((void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_WR)) & 0xFFFFl;
	rx_wrptr = (rx_wrptr + free_num) % (fifo_depth * 2);
	temp |= (u32)(rx_wrptr << 16);
	writel_relaxed(temp, (void *)(PSEL_DL_MISS + PAMWIFI_COMMON_RX_FIFO_WR));

	if (num < free_num) {
		mdelay(100);
		sprdwl_analyze_pamwifi_miss_node();
	}
}

static int miss_thread(void *data)
{
	while (1) {
		if (pamwifi_priv->kthread_stop == 1) {
			if (kthread_should_stop())
				return 0;
			usleep_range(50, 100);
			continue;
		} else
			wait_for_completion(&pamwifi_priv->tx_completed);
		sprdwl_analyze_pamwifi_miss_node();
	}
	return 0;
}

int sprdwl_pamwifi_recv_skb(struct notifier_block *nb,
								  unsigned long data, void *ptr)
{
	struct sprdwl_vif *vif;
	struct iphdr *iph;
	struct ipv6hdr *ipv6h;
	struct ethhdr *ethh;
	struct net_device *net;
	unsigned int real_len = 0;
	unsigned int payload_len = 0;
	bool ip_arp = true;
	struct sk_buff *skb = (struct sk_buff *)ptr;

	wl_info("%s, %d\n", __func__, __LINE__);
	vif = sprdwl_find_ul_vif(skb);
	if (!vif)
		return -1;
	net = vif->ndev;
	if (!net) {
		sprdwl_hex_dump("net_dev is null, dump skb:", skb->data, 100);
		dev_kfree_skb(skb);
		return -1;
	}

	skb_reset_mac_header(skb);
	ethh = eth_hdr(skb);

	skb->protocol = eth_type_trans(skb, net);
	skb_reset_network_header(skb);

	switch (ntohs(ethh->h_proto)) {
	case ETH_P_IP:
		iph = ip_hdr(skb);
		real_len = ntohs(iph->tot_len);
		break;
	case ETH_P_IPV6:
		ipv6h = ipv6_hdr(skb);
		payload_len = ntohs(ipv6h->payload_len);
		real_len = payload_len + sizeof(struct ipv6hdr);
		break;
	case ETH_P_ARP:
		real_len = arp_hdr_len(net);
		break;
	default:
		ip_arp = false;
		pr_debug("skb %p is neither v4 nor v6 nor arp\n", skb);
		break;
	}

	/* resize the skb->len to a real one */
	if (ip_arp)
		skb_trim(skb, real_len);

	/* TODO chechsum ... */
	skb->ip_summed = CHECKSUM_NONE;

	/*count rx packets*/
	net->stats.rx_packets++;
	net->stats.rx_bytes += skb->len;

	netif_receive_skb(skb);
	wl_info("%s, %d\n", __func__, __LINE__);
	return 0;
}

static struct notifier_block wifi_recv_skb = {
	.notifier_call = sprdwl_pamwifi_recv_skb,
};

void pamwifi_dl_flowctrl_handler(int flowctrl)
{
	struct sprdwl_intf *intf = (struct sprdwl_intf *)g_intf_sc2355.intf;
	int i;

	for (i = 0; i < MAX_LUT_NUM; i++) {
		if (!intf->peer_entry[i].vif)
			continue;

		if (flowctrl) {
		        netif_stop_queue(intf->peer_entry[i].vif->ndev);
		} else if (netif_queue_stopped(intf->peer_entry[i].vif->ndev)) {
		        netif_wake_queue(intf->peer_entry[i].vif->ndev);
		}
	}
}

void pamwifi_dl_rm_notify_cb(void *priv, enum sipa_evt_type evt,
								  unsigned long data)
{
	switch (evt) {
	case SIPA_LEAVE_FLOWCTRL:
	        wl_info("sipa_wifi SIPA LEAVE FLOWCTRL\n");
	        pamwifi_dl_flowctrl_handler(0);
	        break;
	case SIPA_ENTER_FLOWCTRL:
	        wl_info("sipa_wifi SIPA ENTER FLOWCTRL\n");
	        pamwifi_dl_flowctrl_handler(1);
	        break;
	default:
	        break;
	}
}

void pamwifi_ul_res_add_wq(struct sprdwl_vif *vif, u8 flag)
{
	struct sprdwl_work *misc_work;
	struct sprdwl_pamwifi_ul_status ul_sts;

	wl_info("%s, %d, flag: %u\n", __func__, __LINE__, flag);
	misc_work = sprdwl_alloc_work(sizeof(struct sprdwl_pamwifi_ul_status));
	if (!misc_work) {
		wl_err("%s out of memory\n", __func__);
		return;
	}
	ul_sts.sub_type = flag;
	ul_sts.value = 1;
	wl_err("%s, sub_type: %u\n", __func__, __LINE__);

	misc_work->vif = vif;
	misc_work->id = SPRDWL_WORK_UL_RES_STS_CMD;
	memcpy(misc_work->data, &ul_sts, sizeof(ul_sts));

	sprdwl_queue_work(vif->priv, misc_work);
}

/*UL resource management*/
void pamwifi_ul_resource_event(struct sprdwl_vif *vif, u8 *data, u16 len)
{
	struct sprdwl_priv *priv = vif->priv;
	u8 flag;
	int ret = 0;

	wl_info("%s, %d, flag:%u\n", __func__, __LINE__, flag);
	if ((priv->fw_stat[SPRDWL_MODE_AP] == SPRDWL_INTF_CLOSING) ||
		(priv->fw_stat[SPRDWL_MODE_AP] == SPRDWL_INTF_CLOSE))
		return;

	memcpy(&flag, data, sizeof(u8));
	wl_info("%s, %d, flag:%u\n", __func__, __LINE__, flag);
	if (flag == 1 && pamwifi_priv->ul_resource_flag == 0)
		ret = sipa_rm_request_resource(SIPA_RM_RES_CONS_WIFI_UL);
	else if (flag == 0 && pamwifi_priv->ul_resource_flag == 1)
		ret = sipa_rm_release_resource(SIPA_RM_RES_CONS_WIFI_UL);

	if (!ret && flag == 1) {
		wl_err("%s, %d\n", __func__, __LINE__);
		pamwifi_priv->ul_resource_flag = flag;
		pamwifi_ul_res_add_wq(vif, flag);
	}
}

int sprdwl_send_ul_res_cmd(struct sprdwl_priv *priv, u8 vif_ctx_id,
								 void *data, u16 len)
{
	struct sprdwl_msg_buf *msg = NULL;

	msg = sprdwl_cmd_getbuf(priv, len, vif_ctx_id,
			SPRDWL_HEAD_RSP, WIFI_CMD_UL_RES_STS);
	if (!msg)
		return -ENOMEM;
	memcpy(msg->data, data, len);
	return sprdwl_cmd_send_recv(priv, msg, CMD_WAIT_TIMEOUT, NULL, NULL);
}

static void pamwifi_ul_rm_notify_cb(void *user_data,
									    enum sipa_rm_event event,
									    unsigned long data)
{
	struct sprdwl_vif *vif = (struct sprdwl_vif *)user_data;

	wl_info("%s: event %d\n", __func__, event);

	if (!pamwifi_priv)
		return;

	if (!vif || !(vif->priv))
		return;

	wl_info("%s: event %d\n", __func__, event);

	if ((vif->priv->fw_stat[SPRDWL_MODE_AP] == SPRDWL_INTF_CLOSING) ||
		(vif->priv->fw_stat[SPRDWL_MODE_AP] == SPRDWL_INTF_CLOSE))
		return;

	switch (event) {
	case SIPA_RM_EVT_GRANTED:
		if (pamwifi_priv->ul_resource_flag == 0) {
			pamwifi_priv->ul_resource_flag = 1;
			pamwifi_ul_res_add_wq(vif, 1);
		}
		break;
	case SIPA_RM_EVT_RELEASED:
		if (pamwifi_priv->ul_resource_flag == 1) {
			pamwifi_priv->ul_resource_flag = 0;
			pamwifi_ul_res_add_wq(vif, 0);
		}
		break;
	default:
		wl_info("%s: unknown event %d\n", __func__, event);
		break;
	}
}

static void pamwifi_prepare_suspend(void)
{
	u32 value, timeout = 500;

	if (pamwifi_priv->suspend_stage & PAMWIFI_EB_SUSPEND) {
		wl_err("%s, Pam wifi already disabled!", __func__);
		return;
	}

	if (!(pamwifi_priv->suspend_stage & PAMWIFI_REG_SUSPEND)) {
		sipa_disconnect(SIPA_EP_WIFI, SIPA_DISCONNECT_START);
		value = check_pamwifi_ipa_fifo_status();
		wl_info("%s, Start to close Pam wifi!\n", __func__);
		while(value) {
			if (!timeout--) {
				wl_err("Pam wifi close fail!\n");
				break;
			}
			wl_err("Pam wifi closing!\n");
			usleep_range(10, 15);
			value = check_pamwifi_ipa_fifo_status();
		}
		sipa_disconnect( SIPA_EP_WIFI, SIPA_DISCONNECT_END);
		pamwifi_priv->suspend_stage |= PAMWIFI_REG_SUSPEND;
		/*pamwifi not idle, can not power off*/
		if (value)
			return;
	}

	pamwifi_set_enable(false);
	pamwifi_priv->suspend_stage |= PAMWIFI_EB_SUSPEND;

}

static int sprdwl_pamwifi_resume(void)
{
	u8 enable_4in1 =1;
	u32 pamwifi_tx_intr_threshold_en = 2, pamwifi_tx_intr_threshold_delay_en = 1,
		pamwifi_tx_intr_threshold = (u32)1 << 16l, pamwifi_tx_intr_threshold_delay = 0xFFFFl,
		pamwifi_fifo_depth = 1024, value_index_search_depth = 16;
	int i, j, search_depth = 16, ret = 0;
	struct sprdwl_intf *intf = (struct sprdwl_intf *)g_intf_sc2355.intf;

	/*init ipa for pamwifi*/
	pamwifi_config_ipa();
	ret = sipa_pam_connect(&pamwifi_priv->sipa_params);
	if (ret) {
		wl_err("%s, pamwifi connect ipa fail\n", __func__);
		return ret;
	}

	if (!pamwifi_priv->cp_cap.mux_tx_common_fifo_support)
		enable_4in1 = 0;
	else
		enable_4in1 = 1;
	pamwifi_common_configuration(intf->pdev, enable_4in1, pamwifi_tx_intr_threshold,
								 pamwifi_tx_intr_threshold_en, pamwifi_tx_intr_threshold_delay,
								 pamwifi_tx_intr_threshold_delay_en, pamwifi_fifo_depth,
								 value_index_search_depth);

	/*init msdu dscr*/
	if (!pamwifi_priv->cp_cap.chip_ver)
		pamwifi_ac_msdu_init(0);
	else
		pamwifi_ax_msdu_init(0);

	/*try 2 ipi mode, because marlin3 only support 1ipi*/
	pamwifi_config_ipi();

	/*recovery router table, now max_lut_index 32, so search depth is 16*/
	for (i = 0; i < search_depth; i++) {
		for (j = 0; j < 4; j++) {
			writel_relaxed(pamwifi_priv->router_table_backup[2 * i][j],
				(void *)(PSEL_RAM1 + 16 * i));
			writel_relaxed(pamwifi_priv->router_table_backup[2 * i + 1][j],
				(void *)(PSEL_RAM2 + 16 * i));
		}
	}

	return ret;
}

static int pamwifi_prepare_resume(void)
{
	u32 value;
	int ret = 0;

	wl_err("%s, %d, stage: %u\n", __func__, __LINE__, pamwifi_priv->suspend_stage);
	if (!(pamwifi_priv->suspend_stage & PAMWIFI_EB_SUSPEND))
		goto out;

	if (pamwifi_priv->suspend_stage & PAMWIFI_EB_SUSPEND) {
		pamwifi_set_enable(true);
		pamwifi_priv->suspend_stage &= ~PAMWIFI_EB_SUSPEND;
	}

	if (pamwifi_priv->suspend_stage & PAMWIFI_REG_SUSPEND) {
		ret = sprdwl_pamwifi_resume();
		if (ret)
			return ret;
		value = readl_relaxed((void *)REG_PAM_WIFI_CFG_START);
		/*start pamwifi*/
		value |= 0x1l;
		writel_relaxed(value, (void *)REG_PAM_WIFI_CFG_START);
		pamwifi_priv->suspend_stage &= ~PAMWIFI_REG_SUSPEND;
	}

out:
	wl_err("%s, %d, stage: %u\n", __func__, __LINE__, pamwifi_priv->suspend_stage);
	sipa_rm_notify_completion(SIPA_RM_EVT_GRANTED,
				SIPA_RM_RES_PROD_PAM_WIFI);

	return ret;
}

void sprdwl_pamwifi_power_work(struct work_struct *work)
{
	if (pamwifi_priv->power_status) {
		/*pam_wifi resume*/
		if (pamwifi_prepare_resume()) {
			wl_err("%s, pamwifi resume fail, resume again\n");
			queue_delayed_work(pamwifi_priv->power_wq,
				&pamwifi_priv->power_work, msecs_to_jiffies(200));
		}
	} else {
		/*pam_wifi suspend*/
		pamwifi_prepare_suspend();
	}
}

static int pamwifi_req_res(void *vif)
{
	pamwifi_priv->power_status = true;

	wl_err("%s, %d\n", __func__, __LINE__);
	cancel_delayed_work(&pamwifi_priv->power_work);
	queue_delayed_work(pamwifi_priv->power_wq, &pamwifi_priv->power_work, 0);

	return -EINPROGRESS;
}

static int pamwifi_rel_res(void *vif)
{
	pamwifi_priv->power_status = false;

	cancel_delayed_work(&pamwifi_priv->power_work);
	queue_delayed_work(pamwifi_priv->power_wq, &pamwifi_priv->power_work, 0);
	return 0;
}

struct sipa_rm_register_params ul_param;
int sprdwl_pamwifi_res_init(struct sprdwl_vif *vif)
{
	struct sipa_rm_create_params rm_params;
	int ret;

	/*UL*/
	ul_param.user_data = vif;
	ul_param.notify_cb = pamwifi_ul_rm_notify_cb;
	ret = sipa_rm_register(SIPA_RM_RES_CONS_WIFI_UL, &ul_param);
	if (ret) {
		wl_err("UL res register failed\n");
		return ret;
	}

	/*create prod*/
	rm_params.name = SIPA_RM_RES_PROD_PAM_WIFI;
	rm_params.floor_voltage = 0;
	rm_params.reg_params.notify_cb = NULL;
	rm_params.reg_params.user_data = vif;
	rm_params.request_resource = pamwifi_req_res;
	rm_params.release_resource = pamwifi_rel_res;
	ret = sipa_rm_create_resource(&rm_params);
	if (ret) {
		wl_err("res create failed\n");
		return ret;
	}

	/*add dependencys*/
	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WIFI_UL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	if (ret < 0 && ret != -EINPROGRESS) {
		wl_err("pam ipa add_dependency WIFI_UL fail.\n");
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_PAM_WIFI);
		return ret;
	}

	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WIFI_DL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	if (ret < 0 && ret != -EINPROGRESS) {
		wl_err("pam ipa add_dependency WIFI_DL fail.\n");
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_UL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_PAM_WIFI);
		return ret;
	}

	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WWAN_UL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	if (ret < 0 && ret != -EINPROGRESS) {
		wl_err("pam ipa add_dependency WWAN_UL fail.\n");
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_UL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_DL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_PAM_WIFI);
		return ret;
	}

	ret = sipa_rm_add_dependency(SIPA_RM_RES_CONS_WWAN_DL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	if (ret < 0 && ret != -EINPROGRESS) {
		wl_err("pam ipa add_dependency WWAN_DL fail.\n");
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_UL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_DL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WWAN_UL,
					SIPA_RM_RES_PROD_IPA);
		sipa_rm_delete_resource(SIPA_RM_RES_PROD_PAM_WIFI);
		return ret;
	}

	return ret;
}

static void sprdwl_pamwifi_res_uninit(void)
{
	sipa_rm_release_resource(SIPA_RM_RES_CONS_WIFI_UL);
	sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_UL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	sipa_rm_delete_dependency(SIPA_RM_RES_CONS_WIFI_DL,
					SIPA_RM_RES_PROD_PAM_WIFI);
	sipa_rm_delete_resource(SIPA_RM_RES_PROD_PAM_WIFI);
	sipa_rm_deregister(SIPA_RM_RES_CONS_WIFI_UL, &ul_param);
}

void sprdwl_pamwifi_enable(struct sprdwl_vif *vif)
{
	int ret;
	u32 temp;

	pamwifi_priv->kthread_stop = 0;
	//set_reg_bits_all_one(REG_PAM_WIFI_CFG_START,  BIT_PAM_WIFI_CFG_START_PAM_WIFI_START);

	sprdwl_pamwifi_res_init(vif);
	ret = sipa_pam_connect(&pamwifi_priv->sipa_params);
	if (ret) {
		wl_err("%s, pamwifi connect ipa fail\n", __func__);
		pamwifi_priv->suspend_stage = PAMWIFI_REG_SUSPEND;
	}

	ret = sipa_nic_open(
                SIPA_TERM_WIFI,
                -1,
                pamwifi_dl_rm_notify_cb,
                NULL);
	wl_info("%s, nic_id: %d\n", __func__, ret);
	/*TODO dummp0 ??*/
	pamwifi_priv->nic_id = ret;

	temp = readl_relaxed((void *)REG_PAM_WIFI_CFG_START);
	/*start pamwifi*/
	temp |= 0x1l;
	writel_relaxed(temp, (void *)REG_PAM_WIFI_CFG_START);
	pamwifi_priv->suspend_stage = PAMWIFI_READY;

}

void sprdwl_pamwifi_disable(struct sprdwl_vif *vif)
{
	u32 value = 0;
	int timeout = 1000;

	sipa_nic_close(pamwifi_priv->nic_id);
	sprdwl_pamwifi_res_uninit();

	if (!(pamwifi_priv->suspend_stage & PAMWIFI_EB_SUSPEND)) {
		if (!(pamwifi_priv->suspend_stage & PAMWIFI_REG_SUSPEND)) {
			sipa_disconnect(SIPA_EP_WIFI, SIPA_DISCONNECT_START);
			value = check_pamwifi_ipa_fifo_status();
			wl_info("%s, Start to close Pam wifi!\n", __func__);
			while(value) {
				value = check_pamwifi_ipa_fifo_status();
				if (!timeout--) {
					wl_err("Pam wifi close fail!\n");
					break;
				}
				wl_info("Pam wifi closing!\n");
				usleep_range(10, 15);
			}
			sipa_disconnect( SIPA_EP_WIFI, SIPA_DISCONNECT_END);
		}
		pamwifi_priv->suspend_stage |= PAMWIFI_REG_SUSPEND;
		wl_info("%d,Pam wifi close success!!\n", __LINE__);
		/*stop pam wifi*/
		clear_reg_bits(REG_PAM_WIFI_CFG_START, BIT_PAM_WIFI_CFG_START_PAM_WIFI_START);

		read_ul_free_dl_fill_wrptr();

		pamwifi_set_enable(false);
		pamwifi_priv->suspend_stage |= PAMWIFI_EB_SUSPEND;
	}

}

u64 pam_wifi_base_addr_remap;
extern struct device *sprdwl_dev;
int sprdwl_pamwifi_probe(struct platform_device *pdev)
{
	struct resource *res;
#if 0
	struct regmap *sys_regmap;
	u32 reg_info[2];
	int ret;

	sys_regmap = syscon_regmap_lookup_by_name(pdev->dev.of_node,
				       "enable");
	if (IS_ERR(sys_regmap))
		pr_err("%s :get sys regmap fail!\n", __func__);
	ret = syscon_get_args_by_name(pdev->dev.of_node,
								  "enable", 2,
								  reg_info);

	if (sys_regmap) {
		ret = regmap_update_bits(sys_regmap,
							reg_info[0],
							reg_info[1],
							reg_info[1]);
		if (ret < 0) {
			wl_err("%s: regmap update bits failed", __func__);
			return;
		}
	}
#endif
	pamwifi_priv = kzalloc(sizeof(struct sprdwl_pamwifi_priv), GFP_KERNEL);
	if (!pamwifi_priv) {
		wl_err("pamwifi_priv alloc fail!\n");
		return -ENOMEM;
	}
	//pamwifi_priv->pamwifi_glb_base = devm_ioremap_nocache(&pdev->dev, 0x25000004l, 0x4l);
	pamwifi_priv->pamwifi_glb_base = ioremap((phys_addr_t)0x25000004l, 0x10);
	if (pamwifi_priv->pamwifi_glb_base == NULL) {
		wl_err("PAM_WIFI power on fail!\n");
		return -ENODEV;
	}

#if 1
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "pam_wifi_reg_base_remap");
	if (!res) {
		iounmap(pamwifi_priv->pamwifi_glb_base);
		wl_err("wifi get res fail!\n");
		return -ENODEV;
	}
	pam_wifi_base_addr_remap = (unsigned long)devm_ioremap_nocache(sprdwl_dev, res ->start, resource_size(res));
#else
	pam_wifi_base_addr_remap = (u64)ioremap((phys_addr_t)0x25200000l, 0x3000l);
	if (pamwifi_priv->pamwifi_glb_base == NULL) {
		devm_iounmap(&pdev->dev, pamwifi_priv->pamwifi_glb_base);
		wl_err("pam_wifi_base_addr_remap remap fail!\n");
		return -ENODEV;
	}
#endif
	wl_info("pam_wifi_base_addr_remap=0x%llx\n",
		pam_wifi_base_addr_remap);

	return 0;
}

/*check pamwifi idle sts*/
u32 check_pamwifi_ipa_fifo_status(void)
{
	u32 pamwifi_ul_idle, pamwifi_dl_idle, pamwifi_idle, pamwifi_ul_rx_wrptr,
		pamwifi_ul_tx_wrptr, pamwifi_dl_rx_wrptr, pamwifi_dl_tx_wrptr,
		pamwifi_ul_rx_rdptr, 	pamwifi_ul_tx_rdptr, pamwifi_dl_rx_rdptr,
		pamwifi_dl_tx_rdptr;

	pamwifi_dl_idle = readl_relaxed((void *)(REG_PAM_WIFI_CFG_START)) & BIT_(5);
	pamwifi_ul_idle = readl_relaxed((void *)(REG_PAM_WIFI_CFG_START)) & BIT_(4);
	pamwifi_idle = readl_relaxed((void *)(REG_PAM_WIFI_CFG_START)) & BIT_(3);
	pamwifi_ul_rx_wrptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_ul_tx_wrptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_dl_rx_wrptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_dl_tx_wrptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_ul_rx_rdptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_ul_tx_rdptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_dl_rx_rdptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_dl_tx_rdptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_RD)) >> 16) & 0xFFFFl;

	if ((pamwifi_ul_idle == BIT_(4)) && (pamwifi_ul_rx_wrptr == pamwifi_ul_tx_wrptr) &&
		pamwifi_dl_idle == BIT_(5) && (pamwifi_dl_rx_wrptr == pamwifi_dl_tx_wrptr)) {
		wl_err("%s, dl_idle_sts:%u, ul_idle_sts:%u, idle_sts:%u, wrptr:%u, %u, %u, %u, rdptr:%u, %u, %u, %u\n",
			    pamwifi_dl_idle, pamwifi_ul_idle, pamwifi_ul_rx_wrptr, pamwifi_ul_tx_wrptr,
			    pamwifi_dl_rx_wrptr, pamwifi_dl_tx_wrptr, pamwifi_ul_rx_rdptr,
			    pamwifi_ul_tx_rdptr, pamwifi_dl_rx_rdptr,pamwifi_dl_tx_rdptr);
		return 0;
	} else
		return 1;
}

void read_ul_free_dl_fill_wrptr(void)
{
	u32 pamwifi_ul_rx_wrptr, 	pamwifi_ul_tx_wrptr, pamwifi_dl_rx_wrptr, pamwifi_dl_tx_wrptr,
		pamwifi_ul_rx_rdptr, 	pamwifi_ul_tx_rdptr, pamwifi_dl_rx_rdptr, pamwifi_dl_tx_rdptr;

	pamwifi_ul_rx_wrptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_ul_tx_wrptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_dl_rx_wrptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_dl_tx_wrptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_WR)) >> 16) & 0xFFFFl;
	pamwifi_ul_rx_rdptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_RX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_ul_tx_rdptr = (readl_relaxed((void *)(PSEL_UL + PAMWIFI_COMMON_TX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_dl_rx_rdptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_RX_FIFO_RD)) >> 16) & 0xFFFFl;
	pamwifi_dl_tx_rdptr = (readl_relaxed((void *)(PSEL_DL_FREE + PAMWIFI_COMMON_TX_FIFO_RD)) >> 16) & 0xFFFFl;

	wl_err("%s, ul free wrptr:%u, ul free rdptr:%u, dl filled wrptr: %u, dl filled rdptr: %u, ul filled wrptr:%u, ul filled rdptr:%u, dl free wrptr: %u, dl free rdptr: %u,\n",
		__func__, pamwifi_ul_tx_wrptr, pamwifi_ul_tx_rdptr, pamwifi_dl_tx_wrptr, pamwifi_dl_tx_rdptr,
		pamwifi_ul_rx_wrptr, pamwifi_ul_rx_rdptr, pamwifi_dl_rx_wrptr, pamwifi_dl_rx_rdptr);
}

void sprdwl_pamwifi_free_rx_buf(void)
{
	skb_queue_purge(&pamwifi_priv->buffer_list);
}

void sprdwl_deinit_pamwifi_fifo(struct platform_device *pdev, u32 fifo_depth)
{
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_type1_virt_addr, dl_type1_phy_addr);
	dl_type1_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_type2_virt_addr, dl_type2_phy_addr);
	dl_type2_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_type3_virt_addr, dl_type3_phy_addr);
	dl_type3_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_type4_virt_addr, dl_type4_phy_addr);
	dl_type4_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_free_virt_addr, dl_free_phy_addr);
	dl_free_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 miss_tx_virt_addr, miss_tx_phy_addr);
	miss_tx_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 miss_rx_virt_addr, miss_rx_phy_addr);
	miss_rx_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 dl_4in1_virt_addr, dl_4in1_phy_addr);
	dl_4in1_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 ul_tx_virt_addr, ul_tx_phy_addr);
	ul_tx_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, fifo_depth*sizeof(dma_addr_t),
					 ul_rx_virt_addr, ul_rx_phy_addr);
	ul_rx_virt_addr = NULL;
	dma_free_coherent(&pdev->dev, 1*16, pam_wifi_msdu_header_info,
					 term_pam_wifi_msdu_header_buf);
	pam_wifi_msdu_header_info = NULL;
	wl_info("%d,Pam wifi close success!!\n", __LINE__);
}

int sprdwl_pamwifi_init(struct platform_device *pdev, struct sprdwl_priv *priv)
{
	u8 enable_4in1 =1;
	u32 pamwifi_tx_intr_threshold_en = 2, pamwifi_tx_intr_threshold_delay_en = 1,
		pamwifi_tx_intr_threshold = (u32)64 << 16l, pamwifi_tx_intr_threshold_delay = 20000,
		pamwifi_fifo_depth = 1024, value_index_search_depth = 16;
	struct sprdwl_pamwifi_msg_buf *pamwifi_msg_buf;
	int i, ret;

	ret = sprdwl_pamwifi_probe(pdev);
	if (ret) {
		wl_err("%s pamwifi probe fail\n");
		return ret;
	}

	memcpy(&pamwifi_priv->cp_cap, &priv->cp_cap, sizeof(struct pam_wifi_cap_cp));

	/*enable pamwifi eb*/
	pamwifi_set_enable(true);

	/*check cp2 cap*/
	/*if (!pamwifi_priv->cp_cap.cp_pam_wifi_support) {
		wl_err("cp2 do not support pam_wifi!\n");
		return;
	}*/

	/*init ipa for pamwifi*/
	pamwifi_config_ipa();

	if (!pamwifi_priv->cp_cap.mux_tx_common_fifo_support)
		enable_4in1 = 0;
	else
		enable_4in1 = 1;
	pamwifi_common_configuration(pdev, enable_4in1, pamwifi_tx_intr_threshold,
		pamwifi_tx_intr_threshold_en, pamwifi_tx_intr_threshold_delay,
		pamwifi_tx_intr_threshold_delay_en, pamwifi_fifo_depth, value_index_search_depth);

	/*init msdu dscr*/
	if (!pamwifi_priv->cp_cap.chip_ver)
		pamwifi_ac_msdu_init(0);
	else
		pamwifi_ax_msdu_init(0);

	/*try 2 ipi mode, because marlin3 only support 1ipi*/
	pamwifi_config_ipi();

	/*register pamwifi irq*/
	pamwifi_priv->pam_wifi_irq = platform_get_irq_byname(pdev, "pam-wifi-irq");
	ret = request_irq(pamwifi_priv->pam_wifi_irq, pamwifi_irq_handle,
		IRQF_NO_SUSPEND, "pam_wifi_irq", NULL);
	wl_info("pam_wifi_irq-%d , ret: %d!!!\n", pamwifi_priv->pam_wifi_irq, ret);

	/*miss kthread*/
	pamwifi_priv->miss_thread = kthread_create(miss_thread, NULL,"sprdwl_miss_thread");
	init_completion(&pamwifi_priv->tx_completed);
	wake_up_process(pamwifi_priv->miss_thread);

	/*init miss buf*/
	pamwifi_priv->pamwifi_msg_list = kzalloc(sizeof(struct sprdwl_pamwifi_msg_list), GFP_KERNEL);
	pamwifi_priv->pamwifi_msg_list->max_num = pamwifi_fifo_depth;
	spin_lock_init(&pamwifi_priv->pamwifi_msg_list->free_lock);
	spin_lock_init(&pamwifi_priv->pamwifi_msg_list->busy_lock);
	INIT_LIST_HEAD(&pamwifi_priv->pamwifi_msg_list->free_list);
	INIT_LIST_HEAD(&pamwifi_priv->pamwifi_msg_list->busy_list);
	for (i = 0; i < pamwifi_fifo_depth; i++) {
		pamwifi_msg_buf = kzalloc(sizeof(struct sprdwl_pamwifi_msg_buf), GFP_KERNEL);
		if (pamwifi_msg_buf) {
			INIT_LIST_HEAD(&pamwifi_msg_buf->list);
			list_add_tail(&pamwifi_msg_buf->list, &pamwifi_priv->pamwifi_msg_list->free_list);
		} else {
			wl_err("miss buffer alloc failed!\n");
			return -ENOMEM;
		}
	}

	sipa_dummy_register_wifi_recv_handler(&wifi_recv_skb);

	/*create power workqueue*/
	INIT_DELAYED_WORK(&pamwifi_priv->power_work, sprdwl_pamwifi_power_work);

	pamwifi_priv->power_wq = create_workqueue("pamwifi_power_wq");
	if (!pamwifi_priv->power_wq) {
		wl_err("pamwifi power wq create failed\n");
		return -ENOMEM;
	}

	return 0;
}

void sprdwl_pamwifi_uninit(struct platform_device *pdev)
{
	int read_on_reg;
	unsigned long flags = 0;
	struct sprdwl_pamwifi_msg_buf *pamwifi_msg_buf = NULL;
	struct sprdwl_pamwifi_msg_list *pamwifi_msg_list = NULL;

	pamwifi_msg_list = pamwifi_priv->pamwifi_msg_list;

	pamwifi_priv->kthread_stop = 1;
	if (pamwifi_priv->miss_thread) {
		complete(&pamwifi_priv->tx_completed);
		kthread_stop(pamwifi_priv->miss_thread);
		pamwifi_priv->miss_thread = NULL;
	}

	disable_irq(pamwifi_priv->pam_wifi_irq);
	free_irq(pamwifi_priv->pam_wifi_irq, NULL);

	sprdwl_deinit_pamwifi_fifo(pdev, 1024);

	read_on_reg = __raw_readl(pamwifi_priv->pamwifi_glb_base);
	read_on_reg &= ~0x40l;
	__raw_writel(read_on_reg, pamwifi_priv->pamwifi_glb_base);
	//devm_iounmap(&pdev->dev, pamwifi_priv->pamwifi_glb_base);
	//devm_iounmap(&pdev->dev, (void __iomem *)pam_wifi_base_addr_remap);
	/*destroy power workqueue*/
	destroy_workqueue(pamwifi_priv->power_wq);
	sipa_dummy_unregister_wifi_recv_handler(&wifi_recv_skb);

	spin_lock_irqsave(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);
	if (!list_empty(&pamwifi_priv->pamwifi_msg_list->free_list)) {
		pamwifi_msg_buf = list_first_entry(&pamwifi_priv->pamwifi_msg_list->free_list,
										   struct sprdwl_pamwifi_msg_buf, list);
		list_del(&pamwifi_msg_buf->list);
		kfree(pamwifi_msg_buf);
	}
	spin_unlock_irqrestore(&pamwifi_priv->pamwifi_msg_list->free_lock, flags);

	kfree(pamwifi_msg_list);
	kfree(pamwifi_priv);
}

int sprdwl_pamwifi_pkt_checksum(struct sk_buff *skb, struct net_device *ndev)
{
	struct udphdr *udphdr;
	struct tcphdr *tcphdr;
	struct iphdr *iphdr;
	struct ipv6hdr *ipv6hdr;
	__sum16 checksum = 0;
	unsigned char iphdrlen = 0;
	struct ethhdr *ethhdr = (struct ethhdr *)skb->data;

	if (ethhdr->h_proto == htons(ETH_P_IPV6)) {
		ipv6hdr = (struct ipv6hdr *)(skb->data + ETHER_HDR_LEN);
		iphdrlen = sizeof(*ipv6hdr);
	} else if (ethhdr->h_proto == htons(ETH_P_IP)) {
		iphdr = (struct iphdr *)(skb->data + ETHER_HDR_LEN);
		iphdrlen = ip_hdrlen(skb);
	}
	udphdr = (struct udphdr *)(skb->data + ETHER_HDR_LEN + iphdrlen);
	tcphdr = (struct tcphdr *)(skb->data + ETHER_HDR_LEN + iphdrlen);

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		checksum =
		(__force __sum16)do_csum(
		skb->data + ETHER_HDR_LEN + iphdrlen,
		skb->len - ETHER_HDR_LEN - iphdrlen);
		if ((ethhdr->h_proto == htons(ETH_P_IPV6) && ipv6hdr->nexthdr == IPPROTO_UDP) ||
			(ethhdr->h_proto == htons(ETH_P_IP) && iphdr->protocol != IPPROTO_UDP)) {
			udphdr->check = ~checksum;
			wl_info("csum:%x,udp check:%x\n",
			checksum, udphdr->check);
		} else if ((ethhdr->h_proto == htons(ETH_P_IPV6) && ipv6hdr->nexthdr == IPPROTO_TCP) ||
		(ethhdr->h_proto == htons(ETH_P_IP) && iphdr->protocol != IPPROTO_TCP)) {
			tcphdr->check = ~checksum;
			wl_info("csum:%x,tcp check:%x\n",
			checksum, tcphdr->check);
		} else
			return 1;

		skb->ip_summed = CHECKSUM_NONE;
		return 0;
	}
	return 1;
}

int sprdwl_xmit_to_ipa_pamwifi(struct sk_buff *skb, struct net_device *ndev)
{
	struct sprdwl_vif *vif = netdev_priv(ndev);
	struct sprdwl_intf *intf = (struct sprdwl_intf *)vif->priv->hw_priv;
	unsigned char lut_index;
	struct ethhdr *ethhdr = (struct ethhdr *)skb->data;
	int ret = 0;

	unsigned int qos_index = 0;
	struct sprdwl_peer_entry *peer_entry = NULL;
	unsigned char tid = 0, tos = 0;

	wl_debug("skb %p sprdwl_xmit_to_ipa_pamwifi invoked\n", skb);

	wl_debug("skb %p %40ph\n", skb, skb->data);
	lut_index = sprdwl_find_lut_index(intf, vif);
	/*filter pkt to pam wifi*/
	if ((ethhdr->h_proto == htons(ETH_P_IPV6) ||ethhdr->h_proto == htons(ETH_P_IP)) &&
		lut_index > 5) {
		/*add tx ba*/
		intf->tx_num[lut_index]++;
		qos_index = get_tid_qosindex(skb, MSDU_DSCR_RSVD + DSCR_LEN, &tid, &tos);
		peer_entry = &intf->peer_entry[lut_index];
		prepare_addba(intf, lut_index, peer_entry, tid);

		intf->skb_da = skb->data;
		if (skb_headroom(skb) < (32 + NET_IP_ALIGN)) {
			struct sk_buff *tmp_skb = skb;

			skb = skb_realloc_headroom(skb, 32 + NET_IP_ALIGN);
			dev_kfree_skb(tmp_skb);
			if (!skb) {
				netdev_err(ndev, "%send to pam wifi, skb_realloc_headroom failed\n",
						  __func__);
				return NETDEV_TX_OK;
			}
		}
		ret = sipa_nic_tx(pamwifi_priv->nic_id, SIPA_TERM_WIFI, -1, skb);
		if (unlikely(ret != 0)) {
			wl_err("sipa_wifi fail to send skb, ret %d\n", ret);
			if (ret == -ENOMEM || ret == -EAGAIN) {
				ndev->stats.tx_fifo_errors++;
				if (sipa_nic_check_flow_ctrl(pamwifi_priv->nic_id)) {
					netif_stop_queue(ndev);
					wl_err("stop queue on dev %s\n", ndev->name);
				}
				sipa_nic_trigger_flow_ctrl_work(pamwifi_priv->nic_id, ret);
				return NETDEV_TX_BUSY;
			}
		}
		wl_debug("%s, succeed to send to ipa\n", __func__);
		vif->ndev->stats.tx_bytes += skb->len;
		vif->ndev->stats.tx_packets++;
		return NETDEV_TX_OK;
	}

	sprdwl_pamwifi_pkt_checksum(skb, ndev);
	//sprdwl_hex_dump("sprdwl xmit dump:", skb->data, 100);
	sprdwl_xmit_data2cmd_wq(skb, ndev);

	return NETDEV_TX_OK;
}

