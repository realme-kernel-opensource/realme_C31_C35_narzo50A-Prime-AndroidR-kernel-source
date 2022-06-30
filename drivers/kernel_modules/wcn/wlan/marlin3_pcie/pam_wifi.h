#ifndef _PAM_WIFI_PAM_WIFI_REG_H
#define _PAM_WIFI_PAM_WIFI_REG_H

#define BIT_(x) ((u32 )((u32 )1 << x))

extern u64 pam_wifi_base_addr_remap;
#define PAM_WIFI_REG_BASE  pam_wifi_base_addr_remap

//[14:8] == 7'b000_0000
#define    PSEL_RF				(PAM_WIFI_REG_BASE+(0x00<<8))
//[14:7] == 8'b000_0001_1
#define    PSEL_DL_TYPE1			(PAM_WIFI_REG_BASE+(0x180))
//[14:7] == 8'b000_0010_0
#define    PSEL_DL_TYPE2			(PAM_WIFI_REG_BASE+(0x200))
//[14:7] == 8'b000_0010_1
#define    PSEL_DL_TYPE3			(PAM_WIFI_REG_BASE+(0x280))
//[14:7] == 8'b000_0011_0
#define    PSEL_DL_TYPE4			(PAM_WIFI_REG_BASE+(0x300))
//[14:7] == 8'b000_0011_1
#define    PSEL_DL_FREE			(PAM_WIFI_REG_BASE+(0x380))
//[14:7] == 8'b000_0100_0
#define    PSEL_UL				(PAM_WIFI_REG_BASE+(0x400))
//[14:7] == 8'b000_0101_0
#define    PSEL_DL_MISS			(PAM_WIFI_REG_BASE+(0x500))

//[14:11] == 4'b000_1   //Wrong in mail for 6:DA
#define    PSEL_RAM1				(PAM_WIFI_REG_BASE+(0x800))
//[14:11] == 4'b001_0   //Wrong in mail for 6
#define    PSEL_RAM2				 (PAM_WIFI_REG_BASE+(0x1000))

#define    REG_PAM_WIFI_AP_UL_FILLED_FIFO_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0000 )
#define    REG_PAM_WIFI_AP_UL_FREE_FIFO_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0004 )
#define    REG_PAM_WIFI_AP_DL_FILLED_FIFO_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0008 )
#define    REG_PAM_WIFI_AP_DL_FREE_FIFO_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x000C )
#define    REG_PAM_WIFI_AP_UL_FREE_FIFO_BASE_ADDRH		( PAM_WIFI_REG_BASE + 0x0010 )
#define    REG_PAM_WIFI_AP_UL_FREE_STS_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0014 )
#define    REG_PAM_WIFI_AP_UL_FILLED_STS_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0018 )
#define    REG_PAM_WIFI_AP_DL_FREE_STS_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x001C )
#define    REG_PAM_WIFI_AP_DL_FILLED_STS_BASE_ADDRL		( PAM_WIFI_REG_BASE + 0x0020 )
#define    REG_PAM_WIFI_AP_DL_FILLED_STS_BASE_ADDRH		( PAM_WIFI_REG_BASE + 0x0024 )
#define    REG_PAM_WIFI_CFG_DL_FILLED_BUFFER_CTRL		( PAM_WIFI_REG_BASE + 0x0028 )
#define    REG_PAM_WIFI_CFG_BUFFER_WATERMARK			( PAM_WIFI_REG_BASE + 0x002C )
#define    REG_PAM_WIFI_CFG_BUFFER_CTRL					( PAM_WIFI_REG_BASE + 0x0030 )
#define    REG_PAM_WIFI_UL_FREE_DDR_MAPPING_OFFSET_L	( PAM_WIFI_REG_BASE + 0x0034 )
#define    REG_PAM_WIFI_UL_FILLED_DDR_MAPPING_OFFSET_L	( PAM_WIFI_REG_BASE + 0x0038 )
#define    REG_PAM_WIFI_DL_FREE_DDR_MAPPING_OFFSET_L	( PAM_WIFI_REG_BASE + 0x003C )
#define    REG_PAM_WIFI_DL_FILLED_DDR_MAPPING_OFFSET_L	( PAM_WIFI_REG_BASE + 0x0040 )
#define    REG_PAM_WIFI_DDR_MAPPING_OFFSET_H				( PAM_WIFI_REG_BASE + 0x0044 )
#define    REG_PAM_WIFI_MSDU_ADDR_L						( PAM_WIFI_REG_BASE + 0x0048 )
#define    REG_PAM_WIFI_UL_NODE_INFO_CONFIG				( PAM_WIFI_REG_BASE + 0x004C )
#define    REG_PAM_WIFI_CFG_START							( PAM_WIFI_REG_BASE + 0x0050 )
#define    REG_PAM_WIFI_RF_DL_CFG							( PAM_WIFI_REG_BASE + 0x0054 )
#define    REG_PAM_WIFI_RF_DL_FLOW_THRESHOLD				( PAM_WIFI_REG_BASE + 0x0058 )
#define    REG_PAM_WIFI_RF_DL_FLOW_PKT_NUM_CNT_1		( PAM_WIFI_REG_BASE + 0x005C )
#define    REG_PAM_WIFI_RF_DL_FLOW_PKT_NUM_CNT_2		( PAM_WIFI_REG_BASE + 0x0060 )
#define    REG_PAM_WIFI_AXI_MST_CFG						( PAM_WIFI_REG_BASE + 0x0064 )
#define    REG_PAM_WIFI_CH_WR_PRIO						( PAM_WIFI_REG_BASE + 0x0068 )
#define    REG_PAM_WIFI_CH_RD_PRIO							( PAM_WIFI_REG_BASE + 0x006C )
#define    REG_PAM_WIFI_BUFFER_TIMEOUT_VAL				( PAM_WIFI_REG_BASE + 0x0070 )
#define    REG_PAM_WIFI_IP_VER								( PAM_WIFI_REG_BASE + 0x0074 )
#define    REG_PAM_WIFI_INT_EN								( PAM_WIFI_REG_BASE + 0x0078 )
#define    REG_PAM_WIFI_INT_CLR								( PAM_WIFI_REG_BASE + 0x007C )
#define    REG_PAM_WIFI_INT_STS								( PAM_WIFI_REG_BASE + 0x0080 )
#define    REG_PAM_WIFI_IPI_UL1_BASE_ADDRL				( PAM_WIFI_REG_BASE + 0x0084 )
#define    REG_PAM_WIFI_IPI_UL2_BASE_ADDRL				( PAM_WIFI_REG_BASE + 0x0088 )
#define    REG_PAM_WIFI_IPI_DL1_BASE_ADDRL				( PAM_WIFI_REG_BASE + 0x008C )
#define    REG_PAM_WIFI_IPI_DL2_BASE_ADDRL				( PAM_WIFI_REG_BASE + 0x0090 )
#define    REG_PAM_WIFI_IPI_BASE_ADDRH					( PAM_WIFI_REG_BASE + 0x0094 )
#define    REG_PAM_WIFI_IPI_UL1_BASE_WDATA				( PAM_WIFI_REG_BASE + 0x0098 )
#define    REG_PAM_WIFI_IPI_UL2_BASE_WDATA				( PAM_WIFI_REG_BASE + 0x009C )
#define    REG_PAM_WIFI_IPI_DL1_BASE_WDATA				( PAM_WIFI_REG_BASE + 0x00A0 )
#define    REG_PAM_WIFI_IPI_DL2_BASE_WDATA				( PAM_WIFI_REG_BASE + 0x00A4 )
#define    REG_PAM_WIFI_COMMON_FIFO_STS_UPDATE			( PAM_WIFI_REG_BASE + 0x00A8 )
#define    REG_PAM_WIFI_TOS_PRIO							( PAM_WIFI_REG_BASE + 0x00B0 )
#define    REG_PAM_WIFI_SW_DEBUG_MEM_BASE_ADDR			( PAM_WIFI_REG_BASE + 0x00B4 )
#define    REG_PAM_WIFI_INDEX_SEARCH_INDEX				( PAM_WIFI_REG_BASE + 0x00B8 )
#define    REG_PAM_WIFI_INDEX_MISS_ADDRL					( PAM_WIFI_REG_BASE + 0x00BC )
#define    REG_PAM_WIFI_COMMON_FIFO_OFFSET				( PAM_WIFI_REG_BASE + 0x00C0 )
#define    REG_PAM_WIFI_SW_DEBUG_CURT_STS				( PAM_WIFI_REG_BASE + 0x00C4 )
#define    REG_PAM_WIFI_SW_DEBUG_RESP_STS				( PAM_WIFI_REG_BASE + 0x00C8 )
#define    REG_PAM_WIFI_DUMMY_REG							( PAM_WIFI_REG_BASE + 0x00D0 )

//---------------------------------------------------------------------------
// Register Name   : REG_PAM_WIFI_PAM_WIFI_PAM_WIFI_INT_CLR
// Register Offset : 0x0080
// Description     : pam_wifi_int_clr
//---------------------------------------------------------------------------

#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_4IN1_TYPE4_OVERFLOW	(BIT_(13))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_4IN1_TYPE3_OVERFLOW	(BIT_(12))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_4IN1_TYPE2_OVERFLOW	(BIT_(11))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_4IN1_TYPE1_OVERFLOW	(BIT_(10))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_INDEX_MISS_TX		(BIT_(9))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_INDEX_MISS_RX		(BIT_(8))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FILL_TYPE4			(BIT_(7))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FILL_TYPE3			(BIT_(6))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FILL_TYPE2			(BIT_(5))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FILL_TYPE1			(BIT_(4))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FILL_4IN1				(BIT_(3))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_DL_FREE					(BIT_(2))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_UL_FILL					(BIT_(1))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_CLR_UL_FREE					(BIT_(0))

//---------------------------------------------------------------------------
// Register Name   : REG_PAM_WIFI_PAM_WIFI_PAM_WIFI_INT_RAW
// Register Offset : 0x0084
// Description     : pam_wifi_int_raw
//---------------------------------------------------------------------------

#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_4IN1_TYPE4_OVERFLOW	(BIT_(29))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_4IN1_TYPE3_OVERFLOW	(BIT_(28))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_4IN1_TYPE2_OVERFLOW	(BIT_(27))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_4IN1_TYPE1_OVERFLOW	(BIT_(26))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_INDEX_MISS_TX		(BIT_(25))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_INDEX_MISS_RX		(BIT_(24))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FILL_TYPE4			(BIT_(23))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FILL_TYPE3			(BIT_(22))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FILL_TYPE2			(BIT_(21))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FILL_TYPE1			(BIT_(20))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FILL_4IN1			(BIT_(19))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_DL_FREE					(BIT_(18))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_UL_FILL					(BIT_(17))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_RAW_UL_FREE					(BIT_(16))

//---------------------------------------------------------------------------
// Register Name   : REG_PAM_WIFI_PAM_WIFI_PAM_WIFI_INT_STS
// Register Offset : 0x0088
// Description     : pam_wifi_int_sts
//---------------------------------------------------------------------------

#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE4_OVERFLOW	(BIT_(13))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE3_OVERFLOW	(BIT_(12))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE2_OVERFLOW	(BIT_(11))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_4IN1_TYPE1_OVERFLOW	(BIT_(10))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_INDEX_MISS_TX		(BIT_(9))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_INDEX_MISS_RX		(BIT_(8))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE4			(BIT_(7))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE3			(BIT_(6))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE2			(BIT_(5))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_TYPE1			(BIT_(4))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FILL_4IN1				(BIT_(3))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_DL_FREE					(BIT_(2))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_UL_FILL					(BIT_(1))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_STS_UL_FREE					(BIT_(0))

//---------------------------------------------------------------------------
// Register Name   : REG_PAM_WIFI_PAM_WIFI_PAM_WIFI_INT_EN
// Register Offset : 0x007C
// Description     : pam_wifi_int_en
//---------------------------------------------------------------------------

#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_4IN1_TYPE4_OVERFOLOW	(BIT_(13))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_4IN1_TYPE3_OVERFOLOW	(BIT_(12))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_4IN1_TYPE2_OVERFOLOW	(BIT_(11))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_4IN1_TYPE1_OVERFOLOW	(BIT_(10))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_INDEX_MISS_TX			(BIT_(9))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_INDEX_MISS_RX			(BIT_(8))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FILL_TYPE4				(BIT_(7))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FILL_TYPE3				(BIT_(6))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FILL_TYPE2				(BIT_(5))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FILL_TYPE1				(BIT_(4))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FILL_4IN1					(BIT_(3))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_DL_FREE						(BIT_(2))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_UL_FILL						(BIT_(1))
#define    BIT_PAM_WIFI_PAM_WIFI_INT_EN_INT_EN_UL_FREE						(BIT_(0))

#define	BIT_PAM_WIFI_CFG_DL_CP_FILLED_BUFFER_WATERMARK(_X_)		(((_X_) << 4) & 0x000000F0)
#define	BIT_PAM_WIFI_CFG_DL_CP_MISS_BUFFER_WATERMARK(_X_)		((_X_) & 0x0000000F)
#define	BIT_PAM_WIFI_CFG_DL_AP_FILLED_BUFFER_WATERMARK(_X_)		(((_X_) << 28) & 0xF0000000)
#define	BIT_PAM_WIFI_CFG_DL_AP_FREE_BUFFER_WATERMARK(_X_)		(((_X_) << 24) & 0x0F000000)
#define	BIT_PAM_WIFI_CFG_DL_CP_TYPE1_BUFFER_WATERMARK(_X_)		(((_X_) << 16) & 0x00030000)
#define	BIT_PAM_WIFI_CFG_DL_CP_TYPE2_BUFFER_WATERMARK(_X_)		(((_X_) << 18) & 0x000C0000)
#define	BIT_PAM_WIFI_CFG_DL_CP_TYPE3_BUFFER_WATERMARK(_X_)		(((_X_) << 20) & 0x00300000)
#define	BIT_PAM_WIFI_CFG_DL_CP_TYPE4_BUFFER_WATERMARK(_X_)		(((_X_) << 22) & 0x00C00000)
#define	BIT_PAM_WIFI_CFG_UL_CP_FREE_BUFFER_WATERMARK(_X_)		(((_X_) << 12) & 0x0000F000)
#define	BIT_PAM_WIFI_CFG_UL_AP_FREE_BUFFER_WATERMARK(_X_)		(((_X_) << 8) & 0x00000F00)
#define	BIT_PAM_WIFI_CFG_UL_CP_FILLED_BUFFER_WATERMARK(_X_)		(((_X_) << 4) & 0x000000F0)
#define	BIT_PAM_WIFI_CFG_UL_AP_FILLED_BUFFER_WATERMARK(_X_)		((_X_) & 0x0000000F)

/*Common fifo reg*/
#define PAMWIFI_COMMON_RX_FIFO_DEPTH			0x00l
#define PAMWIFI_COMMON_RX_FIFO_WR				0x04l
#define PAMWIFI_COMMON_RX_FIFO_RD				0x08l
#define PAMWIFI_COMMON_TX_FIFO_DEPTH			0x0Cl
#define PAMWIFI_COMMON_TX_FIFO_WR				0x10l
#define PAMWIFI_COMMON_TX_FIFO_RD				0x14l
#define PAMWIFI_COMMON_RX_FIFO_ADDRL			0x18l
#define PAMWIFI_COMMON_RX_FIFO_ADDRH			0x1Cl
#define PAMWIFI_COMMON_TX_FIFO_ADDRL			0x20l
#define PAMWIFI_COMMON_TX_FIFO_ADDRH			0x24l
#define PAMWIFI_PERFETCH_FIFO_CTL					0x28l
#define PAMWIFI_INT_GEN_CTL_TX_FIFO_VALUE		0x2Cl
#define PAMWIFI_INT_GEN_CTL_EN					0x30l
#define PAMWIFI_INT_GEN_CTL_CLR					0x48l

#define PAM_WIFI_HW_LOCK_TO           (20*1000)
//---------------------------------------------------------------------------
// Register Name   : REG_PAM_WIFI_CFG_START
// Register Offset : 0x0050
// Description     :
//---------------------------------------------------------------------------
#define    BIT_PAM_WIFI_CFG_START_PAM_WIFI_TABLE_RD_STOP_EN_MSB                              (17)

#define    BIT_PAM_WIFI_CFG_START_PAM_WIFI_TABLE_RD_STOP                                     (BIT_(17))
#define    BIT_PAM_WIFI_CFG_START_SOFT_TABLE_UPDATE_REQ                                      (BIT_(16))
#define    BIT_PAM_WIFI_CFG_START_PAM_WIFI_PAUSE_REQ                                         (BIT_(6))
#define    BIT_PAM_WIFI_CFG_START_PAM_WIFI_START                                             (BIT_(0))

#endif
