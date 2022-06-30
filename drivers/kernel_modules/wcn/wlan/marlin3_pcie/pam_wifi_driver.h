#ifndef _PAM_WIFI_TESTCASE_H
#define _PAM_WIFI_TESTCASE_H

#include <linux/io.h>
#include <linux/sipa.h>
#include "pam_wifi.h"
//#include "sipa_hal_priv.h"
#include <misc/marlin_platform.h>
#include "mm.h"
#include "sprdwl.h"
#include "core_sc2355.h"
#include "if_sc2355.h"
#include "tx_msg_sc2355.h"
#include "qos.h"
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <net/ip.h>
#include <linux/kthread.h>
#include "work.h"

enum pamwifi_suspend_stage {
	//pamwifi is enabled and resume
	PAMWIFI_READY,
	//PAMWIFI_FORCE_SUSPEND = BIT(0),
	PAMWIFI_EB_SUSPEND = BIT(1),
	PAMWIFI_REG_SUSPEND = BIT(2),
};

#define PAMWIFI_RESUME_STAGE (PAMWIFI_EB_SUSPEND |\
								PAMWIFI_REG_SUSPEND)

enum pamwifi_fifo_type{
	pamwifi_ul_free,
	pamwifi_ul_fill,
	pamwifi_dl_free,
	pamwifi_dl_fill_4in1,
	pamwifi_dl_fill_type1,
	pamwifi_dl_fill_type2,
	pamwifi_dl_fill_type3,
	pamwifi_dl_fill_type4,
	pamwifi_dl_miss_rx,
	pamwifi_dl_miss_tx,
	pamwifi_dl_4in1_type1_overflow,
	pamwifi_dl_4in1_type2_overflow,
	pamwifi_dl_4in1_type3_overflow,
	pamwifi_dl_4in1_type4_overflow,
	pamwifi_max_id
};

typedef struct pamwifi_interrupt_table_tag {
	enum pamwifi_fifo_type  id;
	u32 int_owner;
} pamwifi_interrupt_table_t;

struct ax_tx_msdu_dscr {
	struct {
		/*0:cmd, 1:event, 2:normal data,*/
		/*3:special data, 4:PCIE remote addr*/
		unsigned char type:3;
		/*direction of address buffer of cmd/event,*/
		/*0:Tx, 1:Rx*/
		unsigned char direction_ind:1;
		unsigned char need_rsp:1;
		/*ctxt_id*/
		unsigned char interface:3;
	} common;
	unsigned char offset:4;
	unsigned char reserved:4;
	struct {
		/*1:need HW to do checksum*/
		unsigned char checksum_offload:1;
		/*0:udp, 1:tcp*/
		unsigned char checksum_type:1;
		/*1:use SW rate,no aggregation 0:normal*/
		unsigned char sw_rate:1;
		/*WDS frame*/
		unsigned char wds:1;
		/*1:frame sent from SWQ to MH,
		 *0:frame sent from TXQ to MH,
		   default:0
		 */
		unsigned char swq_flag:1;
		unsigned char rsvd:1;
		/*used by PCIe address buffer, need set default:0*/
		unsigned char next_buffer_type:1;
		/*used by PCIe address buffer, need set default:0*/
		unsigned char pcie_mh_readcomp:1;
	} tx_ctrl;
	unsigned short pkt_len;
	struct {
		unsigned char msdu_tid:4;
		unsigned short sta_lut_index:10;
		unsigned char encryption_bypass:1;
		unsigned char ap_buf_flas:1;
	} buffer_info;
	unsigned char color_bit:2;
	unsigned char seq_num:8;
	unsigned char rsvd:6;
	unsigned short tcp_udp_header_offset;
};

struct pamwifi_miss_node_tx_dscr{
	u64 address : 34;
	u16 length : 15;
	u8 offset : 7;
	u8 src_id : 5;
	u8 tos : 2;
	u8 flag : 1;
};

struct pamwifi_miss_node_rx_dscr{
	u64 address : 34;
	u16 length : 11;
	u16 offset : 11;
	u8 src_id : 5;
	u8 tos : 2;
	u8 flag : 1;
};

struct sprdwl_pamwifi_msg_list{
	struct list_head free_list;
	struct list_head busy_list;
	int max_num;
	atomic_t ref;
	spinlock_t free_lock;
	spinlock_t busy_lock;
};

struct sprdwl_pamwifi_msg_buf{
	struct list_head list;
	struct pamwifi_miss_node_tx_dscr dscr;
};

struct pamwifi_cap_tlv {
	//PAM_WIFI_AP_CAP_TLV_TYPE = 6
	//u16 type;
	//u16 len;
	u8 ap_pam_wifi_support;
	u8 mux_tx_cmn_fifo_support;
	//0:mode 1; 1: mode 1/2; 2:mode 1/2/4
	u8 ipi_mode_support;
	//mux_tx_common_fifo should equal to it
	u16 dl_rx_cmn_fifo_depth;
}__packed;

struct sprdwl_pamwifi_priv{
	int kthread_stop;
	u32 router_table_backup[32][4];
	u32 suspend_stage;
	unsigned int pam_wifi_irq;
	struct sipa_connect_params sipa_params;
	struct sipa_to_pam_info sipa_info;
	enum sipa_nic_id nic_id;
	struct sk_buff_head buffer_list;


	struct delayed_work power_work;
	struct workqueue_struct *power_wq;
	bool power_status;
	//requested: 1, released: 0
	u8 ul_resource_flag;

	struct task_struct *miss_thread;
	struct completion tx_completed;
	struct sprdwl_pamwifi_msg_list *pamwifi_msg_list;
	struct pamwifi_cap_tlv ap_cap;
#ifdef ENABLE_PAM_WIFI
	struct pam_wifi_cap_cp cp_cap;
#endif
	void __iomem *pamwifi_glb_base;
};

struct sprdwl_pamwifi_ul_status{
	u8 sub_type;
	u8 value;
};

#define SIPA_TERM_WIFI 0x2

void pamwifi_update_router_table(struct sprdwl_priv *priv, struct sprdwl_sta_lut_ind *sta_lut,
	u8 vif_mode, u32 index, int flag);
void pamwifi_ul_resource_event(struct sprdwl_vif *vif, u8 *data, u16 len);
int sprdwl_send_ul_res_cmd(struct sprdwl_priv *priv, u8 vif_ctx_id,
				void *data, u16 len);
void sprdwl_pamwifi_enable(struct sprdwl_vif *vif);
void sprdwl_pamwifi_disable(struct sprdwl_vif *vif);
void pamwifi_update_router_table_test(void);
void sprdwl_deinit_pamwifi_fifo(struct platform_device *pdev, u32 fifo_depth);
int sprdwl_xmit_to_ipa_pamwifi(struct sk_buff *skb, struct net_device *ndev);
u32 pam_wifi_update_tx_fifo_wptr(u64 fifo_base, u32 wptr);
u32 check_pamwifi_ipa_fifo_status(void);
void clear_reg_bits(u64 u4_addr, u32 mask);
int sprdwl_pamwifi_fill_rx_buf(struct sprdwl_intf *intf);
int sprdwl_pamwifi_init(struct platform_device *pdev, struct sprdwl_priv *priv);
void sprdwl_pamwifi_uninit(struct platform_device *pdev);
void pamwifi_dl_fill_4in1_en_dir(u32 enable, u32 dir);
void pamwifi_ul_free_en_dir(u32 enable, u32 dir);
//to remove
extern bool sipa_nic_check_flow_ctrl(enum sipa_nic_id nic_id);
int sprdwl_pamwifi_probe(struct platform_device *pdev);
void read_ul_free_dl_fill_wrptr(void);
#endif
