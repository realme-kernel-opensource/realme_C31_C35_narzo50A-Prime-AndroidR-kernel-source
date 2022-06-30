#include "mm_dvfs.h"
#include "mmsys_dvfs_comm.h"

struct mmsys_clk_para clk_vote_table[5] = {
	{JPG_CLK, JPG_CLK4096, 1},
	{VDSP_MTX_DATA_CLK, VDSP_MTX_CLK5120, 1},/*hww ? 512?*/
	{ISP_CLK, ISP_CLK4096, 1},
	{DCAM0_1_AXI_CLK, DCAM0_1_AXI_CLK5120, 1},
	{MM_MTX_DATA_CLK, MM_MTX_DATA_CLK5120, 1},
};


struct ip_dvfs_map_cfg  cpp_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_CPP_INDEX0_MAP, VOLT65, CPP_CLK1280,
		CPP_CLK_INDEX_1280, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_CPP_INDEX1_MAP, VOLT65, CPP_CLK1920,
		CPP_CLK_INDEX_1920, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_CPP_INDEX2_MAP, VOLT65, CPP_CLK2560,
		CPP_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_CPP_INDEX3_MAP, VOLT65, CPP_CLK3072,
		CPP_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_CPP_INDEX4_MAP, VOLT70, CPP_CLK3840,
		CPP_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_CPP_INDEX5_MAP, VOLT70, CPP_CLK3840,
		CPP_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_CPP_INDEX6_MAP, VOLT70, CPP_CLK3840,
		CPP_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_CPP_INDEX7_MAP, VOLT70, CPP_CLK3840,
		CPP_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg depth_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DEPTH_INDEX0_MAP, VOLT65, DEPTH_CLK1280,
		DEPTH_CLK_INDEX_1280, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DEPTH_INDEX1_MAP, VOLT65, DEPTH_CLK1920,
		DEPTH_CLK_INDEX_1920, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DEPTH_INDEX2_MAP, VOLT65, DEPTH_CLK2560,
		DEPTH_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DEPTH_INDEX3_MAP, VOLT65, DEPTH_CLK3072,
		DEPTH_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_DEPTH_INDEX4_MAP, VOLT70, DEPTH_CLK3840,
		DEPTH_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DEPTH_INDEX5_MAP, VOLT70, DEPTH_CLK3840,
		DEPTH_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DEPTH_INDEX6_MAP, VOLT70, DEPTH_CLK3840,
		DEPTH_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DEPTH_INDEX7_MAP, VOLT70, DEPTH_CLK3840,
		DEPTH_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg dcam0_1_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DCAM0_1_INDEX0_MAP,
		VOLT65, DCAM0_1_CLK1536, DCAM0_1_CLK_INDEX_1536,
		0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DCAM0_1_INDEX1_MAP,
		VOLT65, DCAM0_1_CLK2560, DCAM0_1_CLK_INDEX_2560,
		0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DCAM0_1_INDEX2_MAP,
		VOLT65, DCAM0_1_CLK3072, DCAM0_1_CLK_INDEX_3072,
		0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DCAM0_1_INDEX3_MAP,
		VOLT65, DCAM0_1_CLK4096, DCAM0_1_CLK_INDEX_4096,
		0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_DCAM0_1_INDEX4_MAP,
		VOLT70, DCAM0_1_CLK5120, DCAM0_1_CLK_INDEX_5120,
		0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DCAM0_1_INDEX5_MAP,
		VOLT70, DCAM0_1_CLK5120, DCAM0_1_CLK_INDEX_5120,
		0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DCAM0_1_INDEX6_MAP,
		VOLT70, DCAM0_1_CLK5120, DCAM0_1_CLK_INDEX_5120,
		0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DCAM0_1_INDEX7_MAP,
		VOLT70, DCAM0_1_CLK5120, DCAM0_1_CLK_INDEX_5120,
		0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg dcam0_1_axi_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX0_MAP, VOLT65,
		DCAM0_1_AXI_CLK1536, DCAM0_1_AXI_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX1_MAP, VOLT65,
		DCAM0_1_AXI_CLK2560, DCAM0_1_AXI_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX2_MAP, VOLT65,
		DCAM0_1_AXI_CLK3072, DCAM0_1_AXI_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX3_MAP, VOLT65,
		DCAM0_1_AXI_CLK4096, DCAM0_1_AXI_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX4_MAP, VOLT70,
		DCAM0_1_AXI_CLK5120, DCAM0_1_AXI_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX5_MAP, VOLT70,
		DCAM0_1_AXI_CLK5120, DCAM0_1_AXI_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX6_MAP, VOLT70,
		DCAM0_1_AXI_CLK5120, DCAM0_1_AXI_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DCAM0_1_AXI_INDEX7_MAP, VOLT70,
		DCAM0_1_AXI_CLK5120, DCAM0_1_AXI_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg dcam2_3_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DCAM2_3_INDEX0_MAP,
		VOLT65, DCAM2_3_CLK960, DCAM2_3_CLK_INDEX_960,
		3, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DCAM2_3_INDEX1_MAP,
		VOLT65, DCAM2_3_CLK1280, DCAM2_3_CLK_INDEX_1280,
		0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DCAM2_3_INDEX2_MAP,
		VOLT65, DCAM2_3_CLK1536, DCAM2_3_CLK_INDEX_1536,
		0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DCAM2_3_INDEX3_MAP,
		VOLT65, DCAM2_3_CLK1920, DCAM2_3_CLK_INDEX_1920,
		0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_DCAM2_3_INDEX4_MAP,
		VOLT70, DCAM2_3_CLK2560, DCAM2_3_CLK_INDEX_2560,
		0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DCAM2_3_INDEX5_MAP,
		VOLT70, DCAM2_3_CLK2560, DCAM2_3_CLK_INDEX_2560,
		0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DCAM2_3_INDEX6_MAP,
		VOLT70, DCAM2_3_CLK2560, DCAM2_3_CLK_INDEX_2560,
		0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DCAM2_3_INDEX7_MAP,
		VOLT70, DCAM2_3_CLK2560, DCAM2_3_CLK_INDEX_2560,
		0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg dcam2_3_axi_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX0_MAP, VOLT65,
		DCAM2_3_AXI_CLK960, DCAM2_3_AXI_CLK_INDEX_960, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX1_MAP, VOLT65,
		DCAM2_3_AXI_CLK1280, DCAM2_3_AXI_CLK_INDEX_1280, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX2_MAP, VOLT65,
		DCAM2_3_AXI_CLK1536, DCAM2_3_AXI_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX3_MAP, VOLT65,
		DCAM2_3_AXI_CLK1920, DCAM2_3_AXI_CLK_INDEX_1920, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX4_MAP, VOLT70,
		DCAM2_3_AXI_CLK2560, DCAM2_3_AXI_CLK_INDEX_2560, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX5_MAP, VOLT70,
		DCAM2_3_AXI_CLK2560, DCAM2_3_AXI_CLK_INDEX_2560, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX6_MAP, VOLT70,
		DCAM2_3_AXI_CLK2560, DCAM2_3_AXI_CLK_INDEX_2560, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DCAM2_3_AXI_INDEX7_MAP, VOLT70,
		DCAM2_3_AXI_CLK2560, DCAM2_3_AXI_CLK_INDEX_2560, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg dcam_mtx_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_DCAM_MTX_INDEX0_MAP, VOLT65,
		DCAM_MTX_CLK1536, DCAM_MTX_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_DCAM_MTX_INDEX1_MAP, VOLT65,
		DCAM_MTX_CLK3072, DCAM_MTX_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_DCAM_MTX_INDEX2_MAP, VOLT65,
		DCAM_MTX_CLK4096, DCAM_MTX_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_DCAM_MTX_INDEX3_MAP, VOLT70,
		DCAM_MTX_CLK5120, DCAM_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{4, REG_MM_DVFS_AHB_DCAM_MTX_INDEX4_MAP, VOLT70,
		DCAM_MTX_CLK5120, DCAM_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_DCAM_MTX_INDEX5_MAP, VOLT70,
		DCAM_MTX_CLK5120, DCAM_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_DCAM_MTX_INDEX6_MAP, VOLT70,
		DCAM_MTX_CLK5120, DCAM_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_DCAM_MTX_INDEX7_MAP, VOLT70,
		DCAM_MTX_CLK5120, DCAM_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
};


struct ip_dvfs_map_cfg fd_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_FD_INDEX0_MAP, VOLT65,
		FD_CLK1280, FD_CLK_INDEX_1280, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_FD_INDEX1_MAP, VOLT65,
		FD_CLK1920, FD_CLK_INDEX_1920, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_FD_INDEX2_MAP, VOLT65,
		FD_CLK2560, FD_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_FD_INDEX3_MAP, VOLT65,
		FD_CLK3072, FD_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_FD_INDEX4_MAP, VOLT70,
		FD_CLK3840, FD_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_FD_INDEX5_MAP, VOLT70,
		FD_CLK3840, FD_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_FD_INDEX6_MAP, VOLT70,
		FD_CLK3840, FD_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_FD_INDEX7_MAP, VOLT70,
		FD_CLK3840, FD_CLK_INDEX_3840, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg isp_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_ISP_INDEX0_MAP, VOLT65,
		ISP_CLK1536, ISP_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_ISP_INDEX1_MAP, VOLT65,
		ISP_CLK2560, ISP_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_ISP_INDEX2_MAP, VOLT65,
		ISP_CLK3072, ISP_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_ISP_INDEX3_MAP, VOLT65,
		ISP_CLK4096, ISP_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_ISP_INDEX4_MAP, VOLT65,
		ISP_CLK4096, ISP_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{5, REG_MM_DVFS_AHB_ISP_INDEX5_MAP, VOLT75,
		ISP_CLK5120, ISP_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{6, REG_MM_DVFS_AHB_ISP_INDEX6_MAP, VOLT75,
		ISP_CLK5120, ISP_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{7, REG_MM_DVFS_AHB_ISP_INDEX7_MAP, VOLT75,
		ISP_CLK5120, ISP_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
};

struct ip_dvfs_map_cfg jpg_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_JPG_INDEX0_MAP, VOLT65,
		JPG_CLK1536, JPG_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_JPG_INDEX1_MAP, VOLT65,
		JPG_CLK2560, JPG_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_JPG_INDEX2_MAP, VOLT65,
		JPG_CLK3072, JPG_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_JPG_INDEX3_MAP, VOLT65,
		JPG_CLK4096, JPG_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_JPG_INDEX4_MAP, VOLT70,
		JPG_CLK5120, JPG_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_JPG_INDEX5_MAP, VOLT70,
		JPG_CLK5120, JPG_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_JPG_INDEX6_MAP, VOLT70,
		JPG_CLK5120, JPG_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_JPG_INDEX7_MAP, VOLT70,
		JPG_CLK5120, JPG_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg mtx_data_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX0_MAP, VOLT65,
		MM_MTX_DATA_CLK3072, MM_MTX_DATA_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX1_MAP, VOLT65,
		MM_MTX_DATA_CLK3072, MM_MTX_DATA_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX2_MAP, VOLT65,
		MM_MTX_DATA_CLK3072, MM_MTX_DATA_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX3_MAP, VOLT65,
		MM_MTX_DATA_CLK4096, MM_MTX_DATA_CLK_INDEX_4096, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX4_MAP, VOLT70,
		MM_MTX_DATA_CLK5120, MM_MTX_DATA_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX5_MAP, VOLT70,
		MM_MTX_DATA_CLK5120, MM_MTX_DATA_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{6, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX6_MAP, VOLT70,
		MM_MTX_DATA_CLK5120, MM_MTX_DATA_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
	{7, REG_MM_DVFS_AHB_MM_MTX_DATA_INDEX7_MAP, VOLT70,
		MM_MTX_DATA_CLK5120, MM_MTX_DATA_CLK_INDEX_5120, 0, 0, 0, 0, "0.7v"},
};

struct ip_dvfs_map_cfg vdsp_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_VDSP_INDEX0_MAP, VOLT65,
		VDSP_CLK260, VDSP_CLK_INDEX_260,
		VDSP_M_CLK130, VDSP_M_CLK_INDEX_130,
		0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_VDSP_INDEX1_MAP, VOLT65,
		VDSP_CLK3072, VDSP_CLK_INDEX_3072,
		VDSP_M_CLK1536, VDSP_M_CLK_INDEX_1536,
		0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_VDSP_INDEX2_MAP, VOLT65,
		VDSP_CLK5120, VDSP_CLK_INDEX_5120,
		VDSP_M_CLK2560, VDSP_M_CLK_INDEX_2560,
		0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_VDSP_INDEX3_MAP, VOLT65,
		VDSP_CLK6144, VDSP_CLK_INDEX_6144,
		VDSP_M_CLK3072, VDSP_M_CLK_INDEX_3072,
		0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_VDSP_INDEX4_MAP, VOLT70,
		VDSP_CLK8192, VDSP_CLK_INDEX_8192,
		VDSP_M_CLK4096, VDSP_M_CLK_INDEX_4096,
		0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_VDSP_INDEX5_MAP, VOLT75,
		VDSP_CLK10140, VDSP_CLK_INDEX_10140,
		VDSP_M_CLK5070, VDSP_M_CLK_INDEX_5070,
		0, 0, "0.75v"},
	{6, REG_MM_DVFS_AHB_VDSP_INDEX6_MAP, VOLT75,
		VDSP_CLK10140, VDSP_CLK_INDEX_10140,
		VDSP_M_CLK5070, VDSP_M_CLK_INDEX_5070,
		0, 0, "0.75v"},
	{7, REG_MM_DVFS_AHB_VDSP_INDEX7_MAP, VOLT75,
		VDSP_CLK10140, VDSP_CLK_INDEX_10140,
		VDSP_M_CLK5070, VDSP_M_CLK_INDEX_5070,
		0, 0, "0.75v"},
};

struct ip_dvfs_map_cfg vdma_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_VDMA_INDEX0_MAP, VOLT65,
		VDMA_CLK260, VDMA_CLK_INDEX_260, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_VDMA_INDEX1_MAP, VOLT65,
		VDMA_CLK1536, VDMA_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_VDMA_INDEX2_MAP, VOLT65,
		VDMA_CLK2560, VDMA_CLK_INDEX_2560, 0, 0, 0, 0, "0.65v"},
	{3, REG_MM_DVFS_AHB_VDMA_INDEX3_MAP, VOLT65,
		VDMA_CLK3072, VDMA_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{4, REG_MM_DVFS_AHB_VDMA_INDEX4_MAP, VOLT70,
		VDMA_CLK4096, VDMA_CLK_INDEX_4096, 0, 0, 0, 0, "0.7v"},
	{5, REG_MM_DVFS_AHB_VDMA_INDEX5_MAP, VOLT75,
		VDMA_CLK5120, VDMA_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{6, REG_MM_DVFS_AHB_VDMA_INDEX6_MAP, VOLT75,
		VDMA_CLK5120, VDMA_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{7, REG_MM_DVFS_AHB_VDMA_INDEX7_MAP, VOLT75,
		VDMA_CLK5120, VDMA_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
};

struct ip_dvfs_map_cfg vdsp_mtx_dvfs_config_table[8] = {
	{0, REG_MM_DVFS_AHB_VDSP_MTX_INDEX0_MAP, VOLT65,
		VDSP_MTX_CLK1536, VDSP_MTX_CLK_INDEX_1536, 0, 0, 0, 0, "0.65v"},
	{1, REG_MM_DVFS_AHB_VDSP_MTX_INDEX1_MAP, VOLT65,
		VDSP_MTX_CLK3072, VDSP_MTX_CLK_INDEX_3072, 0, 0, 0, 0, "0.65v"},
	{2, REG_MM_DVFS_AHB_VDSP_MTX_INDEX2_MAP, VOLT70,
		VDSP_MTX_CLK4096, VDSP_MTX_CLK_INDEX_4096, 0, 0, 0, 0, "0.7v"},
	{3, REG_MM_DVFS_AHB_VDSP_MTX_INDEX3_MAP, VOLT75,
		VDSP_MTX_CLK5120, VDSP_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{4, REG_MM_DVFS_AHB_VDSP_MTX_INDEX4_MAP, VOLT75,
		VDSP_MTX_CLK5120, VDSP_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{5, REG_MM_DVFS_AHB_VDSP_MTX_INDEX5_MAP, VOLT75,
		VDSP_MTX_CLK5120, VDSP_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{6, REG_MM_DVFS_AHB_VDSP_MTX_INDEX6_MAP, VOLT75,
		VDSP_MTX_CLK5120, VDSP_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
	{7, REG_MM_DVFS_AHB_VDSP_MTX_INDEX7_MAP, VOLT75,
		VDSP_MTX_CLK5120, VDSP_MTX_CLK_INDEX_5120, 0, 0, 0, 0, "0.75v"},
};


