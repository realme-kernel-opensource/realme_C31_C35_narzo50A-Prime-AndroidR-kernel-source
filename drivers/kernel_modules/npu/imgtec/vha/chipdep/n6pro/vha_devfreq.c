/*
 * DVFS functions for qogirn6pro NPU.
 *
 * Copyright (C) 2018 Spreadtrum Communications Inc.
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

#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <dt-bindings/soc/sprd,qogirn6pro-regs.h>
#include <dt-bindings/soc/sprd,qogirn6pro-mask.h>
#include "chipdep/n6pro/sprd,qogirn6pro-npu-mask.h"
#include "chipdep/n6pro/sprd,qogirn6pro-npu-regs.h"
#include <linux/moduleparam.h>
#include <linux/soc/sprd/hwfeature.h>
#include <linux/devfreq.h>
#include <linux/pm_opp.h>
#include "vha_common.h"
#include "vha_chipdep.h"
#include <linux/sprd_npu_cooling.h>


#include <linux/kernel.h>
#define VHA_POLL_MS 100
#define VHA_UPTHRESHOLD 85
#define VHA_DOWNDIFFERENTIAL 10
#define VHA_PM_TIME_SHIFT 8

struct npu_reg_info {
	struct regmap *regmap_ptr;
	u32 args[2];
};

struct npu_freq_info {
	int index;
	int freq;    //kHz
	int volt;    //uV
};

struct npu_dvfs_context {
	int npu_on;
	int npu_dvfs_on;
	int pvr_last_cfg, mtx_last_cfg, ocm_last_cfg;
	struct semaphore *sem;

	struct npu_freq_info *pvr_freq_list;
	struct npu_freq_info *mtx_freq_list;
	struct npu_freq_info *ocm_freq_list;
	int freq_list_len;

	struct npu_freq_info *pvr_freq_cur;
	struct npu_freq_info *pvr_freq_default;
	struct npu_freq_info *mtx_freq_cur;
	struct npu_freq_info *mtx_freq_default;
	struct npu_freq_info *ocm_freq_cur;
	struct npu_freq_info *ocm_freq_default;

	struct npu_reg_info dvfs_reg;
};

DEFINE_SEMAPHORE(npu_dvfs_sem);
static struct npu_dvfs_context npu_dvfs_ctx=
{
	.npu_on = 0,
	.npu_dvfs_on = 0,

	.sem=&npu_dvfs_sem,
};

void vha_force_freq(struct device *dev, unsigned long freq);
static ssize_t force_npu_freq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long force_freq;

	ret = sscanf(buf, "%lu\n", &force_freq);
	pm_runtime_get_sync(dev);
	npu_dvfs_ctx.npu_on = 1;
	vha_force_freq(dev, force_freq);
	npu_dvfs_ctx.npu_on = 0;
	pm_runtime_put(dev);
	return count;
}

static ssize_t force_npu_freq_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count = sprintf(buf, "%d\n", npu_dvfs_ctx.pvr_freq_cur->freq);
	return count;
}

static ssize_t npu_auto_dvfs_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;

	count = sprintf(buf, "%d\n", npu_dvfs_ctx.npu_dvfs_on);
	return count;
}

static ssize_t npu_auto_dvfs_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int enable;
	int ret;

	ret = sscanf(buf, "%u\n", &enable);
	if (ret < 1) {
		dev_warn(dev, "enable para err: %d", ret);
		return count;
	}
	if (enable == 1)
		npu_dvfs_ctx.npu_dvfs_on = 1;
	else if (enable == 0)
		npu_dvfs_ctx.npu_dvfs_on = 0;

	return count;
}

static DEVICE_ATTR(force_npu_freq, 0664,
		force_npu_freq_show, force_npu_freq_store);
static DEVICE_ATTR(npu_auto_dvfs, 0664,
		npu_auto_dvfs_show, npu_auto_dvfs_store);

static struct attribute *dev_entries[] = {
	&dev_attr_force_npu_freq.attr,
	&dev_attr_npu_auto_dvfs.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.name   = "npu_dvfs",
	.attrs  = dev_entries,
};

static int vha_set_freq_volt(int pvr_index, int mtx_index, int ocm_index)
{
	if (npu_dvfs_ctx.npu_on) {
		down(npu_dvfs_ctx.sem);
		regmap_update_bits(npu_dvfs_ctx.dvfs_reg.regmap_ptr,
					REG_AI_DVFS_APB_POWERVR_DVFS_INDEX_CFG,
					MASK_AI_DVFS_APB_POWERVR_DVFS_INDEX,
					pvr_index);
		regmap_update_bits(npu_dvfs_ctx.dvfs_reg.regmap_ptr,
					REG_AI_DVFS_APB_MAIN_MTX_DVFS_INDEX_CFG,
					MASK_AI_DVFS_APB_MAIN_MTX_DVFS_INDEX,
					mtx_index);
		regmap_update_bits(npu_dvfs_ctx.dvfs_reg.regmap_ptr,
					REG_AI_DVFS_APB_OCM_DVFS_INDEX_CFG,
					MASK_AI_DVFS_APB_OCM_DVFS_INDEX,
					ocm_index);
		up(npu_dvfs_ctx.sem);
		npu_dvfs_ctx.pvr_freq_cur = &npu_dvfs_ctx.pvr_freq_list[pvr_index];
		npu_dvfs_ctx.mtx_freq_cur = &npu_dvfs_ctx.mtx_freq_list[mtx_index];
		npu_dvfs_ctx.ocm_freq_cur = &npu_dvfs_ctx.ocm_freq_list[ocm_index];
		npu_dvfs_ctx.pvr_last_cfg = pvr_index;
		npu_dvfs_ctx.mtx_last_cfg = mtx_index;
		npu_dvfs_ctx.ocm_last_cfg = ocm_index;
	}
	return 0;
}

static int npu_dvfs_ctx_init(struct device *dev)
{
	unsigned int i = 0;
	int ret;

	npu_dvfs_ctx.dvfs_reg.regmap_ptr = syscon_regmap_lookup_by_name(dev->of_node,
									"dvfs_reg");
	ret = syscon_get_args_by_name(dev->of_node, "dvfs_reg", 2, npu_dvfs_ctx.dvfs_reg.args);
	if (ret != 2) {
		dev_err(dev, "Failed to get dvfs reg:%d\n", ret);
		return ret;
	}

	npu_dvfs_ctx.freq_list_len = of_property_count_elems_of_size(dev->of_node,
								"sprd,dvfs-powervr-tbl",
								3*sizeof(u32));
	npu_dvfs_ctx.pvr_freq_list = vmalloc(sizeof(struct npu_freq_info)*npu_dvfs_ctx.freq_list_len);
	npu_dvfs_ctx.mtx_freq_list = vmalloc(sizeof(struct npu_freq_info)*npu_dvfs_ctx.freq_list_len);
	npu_dvfs_ctx.ocm_freq_list = vmalloc(sizeof(struct npu_freq_info)*npu_dvfs_ctx.freq_list_len);

	for(i=0; i< npu_dvfs_ctx.freq_list_len; i++){
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-powervr-tbl", 3*i+2,
						&npu_dvfs_ctx.pvr_freq_list[i].index);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-powervr-tbl", 3*i,
						&npu_dvfs_ctx.pvr_freq_list[i].freq);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-powervr-tbl", 3*i+1,
						&npu_dvfs_ctx.pvr_freq_list[i].volt);

		of_property_read_u32_index(dev->of_node, "sprd,dvfs-mtx-tbl", 3*i+2,
						&npu_dvfs_ctx.mtx_freq_list[i].index);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-mtx-tbl", 3*i,
						&npu_dvfs_ctx.mtx_freq_list[i].freq);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-mtx-tbl", 3*i+1,
						&npu_dvfs_ctx.mtx_freq_list[i].volt);

		of_property_read_u32_index(dev->of_node, "sprd,dvfs-ocm-tbl", 3*i+2,
						&npu_dvfs_ctx.ocm_freq_list[i].index);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-ocm-tbl", 3*i,
						&npu_dvfs_ctx.ocm_freq_list[i].freq);
		of_property_read_u32_index(dev->of_node, "sprd,dvfs-ocm-tbl", 3*i+1,
						&npu_dvfs_ctx.ocm_freq_list[i].volt);
	}

	ret = sysfs_create_group(&(dev->kobj), &dev_attr_group);
	if (ret) {
		dev_err(dev, "sysfs create fail: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "sprd,npu_dvfs_default", &i);
	if (ret) {
		dev_warn(dev, "read dvfs default fail: %d\n", ret);
		i = 0;
	}
	if (i >= npu_dvfs_ctx.freq_list_len) {
		dev_err(dev, "set fault failed, out of range: %d\n", i);
		return -ERANGE;
	}

	npu_dvfs_ctx.pvr_freq_default = &npu_dvfs_ctx.pvr_freq_list[i];
	npu_dvfs_ctx.mtx_freq_default = &npu_dvfs_ctx.mtx_freq_list[i];
	npu_dvfs_ctx.ocm_freq_default = &npu_dvfs_ctx.ocm_freq_list[i];

	vha_set_freq_volt(i, i, i);

	return ret;
}

static int vha_freq_search(struct npu_freq_info freq_list[], int len, unsigned long key)
{
	int low = 0, high = len - 1, mid;
	while (low <= high){
		mid = (low + high) / 2;
		if (key == freq_list[mid].freq * 1000UL){
			return mid;
		}
		if (key < freq_list[mid].freq * 1000UL){
			high = mid - 1;
		}else {
			low = mid + 1;
		}
	}
	return -1;
}

static int vha_volt_search(struct npu_freq_info volt_list[], int len, int key)
{
	int low = 0, high = len - 1, mid;
	if (0 > key){
		return -1;
	}
	while (low <= high){
		mid = (low + high) / 2;
		if (key == volt_list[mid].volt){
			return mid;
		}
		if (key < volt_list[mid].volt){
			high = mid - 1;
		}else {
			low = mid + 1;
		}
	}
	return -1;
}

static int vha_vote(int pvr_volt)
{
	int ocm_index = -1;
	ocm_index = vha_volt_search(npu_dvfs_ctx.ocm_freq_list,
				    npu_dvfs_ctx.freq_list_len,
				    pvr_volt);
	return ocm_index;
}

static void vha_pm_get_dvfs_utilisation(struct vha_dev *vha, ktime_t now)
{
	ktime_t diff;
	u32 ns_time;

	diff = ktime_sub(now, vha->cur_state.time_period_start);
	if (ktime_to_ns(diff) < 0) {
		return ;
	}
	ns_time = (u32)(ktime_to_ns(diff) >> VHA_PM_TIME_SHIFT);
	if (vha->cur_state.vha_active) {
		vha->cur_state.value.time_busy += ns_time;
	}else {
		vha->cur_state.value.time_idle += ns_time;
	}
	vha->cur_state.time_period_start = now;
}

void vha_update_dvfs_state(struct vha_dev *vha, bool vha_active, ktime_t *endtimestamp)
{
	ktime_t now;
	unsigned long flags;
	if (!vha) {
		return ;
	}
	spin_lock_irqsave(&vha->cur_state.lock, flags);
	if (!endtimestamp) {
		now = ktime_get();
		endtimestamp = &now;
	}
	vha_pm_get_dvfs_utilisation(vha, *endtimestamp);
	vha->cur_state.vha_active = vha_active;

	spin_unlock_irqrestore(&vha->cur_state.lock, flags);
}

static void vha_devfreq_exit(struct device *dev)
{
	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	struct devfreq_dev_profile *dp;

	if (!vha) {
		return ;
	}
	dp = vha->devfreq->profile;
	kfree(dp->freq_table);
}

static int vha_devfreq_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_pm_opp *opp;
	unsigned long nominal_freq;
	unsigned long exact_freq;
	unsigned long volt;
	int ret;
	int index = -1, ocm_index = -1;

	nominal_freq = *freq;
	/* get appropriate opp*/
	opp = devfreq_recommended_opp(dev, &nominal_freq, flags);
	if (IS_ERR_OR_NULL(opp)) {
		dev_err(dev, "Failed to get opp (%ld)\n", PTR_ERR(opp));
		return PTR_ERR(opp);
	}
	/* get voltage and frequency*/
	volt = dev_pm_opp_get_voltage(opp);
	exact_freq = dev_pm_opp_get_freq(opp);
	dev_pm_opp_put(opp);
	if (exact_freq == npu_dvfs_ctx.pvr_freq_cur->freq * 1000UL)
		return 0;
	index = vha_freq_search(npu_dvfs_ctx.pvr_freq_list,
					npu_dvfs_ctx.freq_list_len,
					exact_freq);
	ocm_index = vha_vote(volt);
	if (index < 0 || ocm_index < 0 || index > npu_dvfs_ctx.freq_list_len || ocm_index > npu_dvfs_ctx.freq_list_len) {
		dev_err(dev, "Failed to get freq or ocm_index\n");
		return -1;
	}
	ret = vha_set_freq_volt(index, index, ocm_index);

	return ret;
}

void vha_force_freq(struct device *dev, unsigned long freq)
{
	unsigned long target_freq = freq * 1000UL;
	vha_devfreq_target(dev, &target_freq, 0);
}

static int vha_devfreq_cur_freq(struct device *dev, unsigned long *freq)
{
	*freq = npu_dvfs_ctx.pvr_freq_cur->freq * 1000UL;
	return 0;
}

static void vha_get_dvfs_metrics(struct vha_dev *vha,
				struct vha_devfreq_metrics *last,
				struct vha_devfreq_metrics *diff)
{
	struct vha_devfreq_metrics *cur = &vha->cur_state.value;
	unsigned long flags;
	spin_lock_irqsave(&vha->cur_state.lock, flags);

	vha_pm_get_dvfs_utilisation(vha, ktime_get());
	memset(diff, 0, sizeof(*diff));

	diff->time_busy = cur->time_busy - last->time_busy;
	diff->time_idle = cur->time_idle - last->time_idle;

	*last = *cur;

	spin_unlock_irqrestore(&vha->cur_state.lock, flags);
	return;
}

static int vha_devfreq_status(struct device *dev, struct devfreq_dev_status *state)
{
	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	struct vha_devfreq_metrics diff;

	vha_get_dvfs_metrics(vha, &vha->last_devfreq_metrics, &diff);
	state->busy_time = diff.time_busy;
	state->total_time = diff.time_busy + diff.time_idle;
	state->current_frequency = npu_dvfs_ctx.pvr_freq_cur-> freq * 1000UL;
	state->private_data = NULL;
	return 0;
}

int vha_devfreq_init(struct vha_dev *vha)
{
	struct devfreq_dev_profile *dp;
	struct devfreq_simple_ondemand_data *data;

	int ret;

	npu_dvfs_ctx.npu_on = 1;
	ret = npu_dvfs_ctx_init(vha->dev);
	if (ret) {
		dev_err(vha->dev, "failed to init npu_dvfs_ctx: %d\n", ret);
		return ret;
	}
	ret = dev_pm_opp_of_add_table(vha->dev);
	if (ret) {
		dev_err(vha->dev, "failed to init OPP table: %d\n", ret);
		return ret;
	}

	dp = &vha->devfreq_profile;
	dp->polling_ms = VHA_POLL_MS;
	dp->initial_freq = npu_dvfs_ctx.pvr_freq_default->freq * 1000UL;
	dp->target = vha_devfreq_target;
	dp->get_cur_freq = vha_devfreq_cur_freq;
	dp->get_dev_status = vha_devfreq_status;
	dp->exit = vha_devfreq_exit;

	data = vmalloc(sizeof(struct devfreq_simple_ondemand_data));
	data->upthreshold = VHA_UPTHRESHOLD;
	data->downdifferential = VHA_DOWNDIFFERENTIAL;

	vha->devfreq = devm_devfreq_add_device(vha->dev, dp, "simple_ondemand", data);
	if (IS_ERR(vha->devfreq)) {
		kfree(dp->freq_table);
		dev_err(vha->dev, "add devfreq device fail\n");
		return PTR_ERR(vha->devfreq);
	}

	vha->devfreq->max_freq = dp->freq_table[dp->max_state - 1];
	vha->devfreq->min_freq = dp->freq_table[0];

	vha->devfreq_init = true;
	npu_dvfs_ctx.npu_dvfs_on = 1;

	//dev_set_drvdata(&vha->devfreq->dev, vha);
	ret = devfreq_register_opp_notifier(vha->dev, vha->devfreq);
	if (ret) {
		dev_err(vha->dev, "Failed to register OPP notifier (%d)\n", ret);
		if (devfreq_remove_device(vha->devfreq)) {
			dev_err(vha->dev, "Failed to terminate devfreq\n");
		} else
			vha->devfreq = NULL;
		return ret;
	}

	ret = npu_cooling_device_register(vha->devfreq);
	if (ret) {
		vha->cooling_device = false;
		dev_err(vha->dev, "Failed to register npu cooling device\n", ret);
		return ret;
	}
	vha->cooling_device = true;

	return ret;
}

void vha_devfreq_term(struct vha_dev *vha)
{
	dev_dbg(vha->dev, "Term NPU devfreq\n");

	sysfs_remove_group(&(vha->dev->kobj), &dev_attr_group);
	if (!vha->devfreq) {
		dev_err(vha->dev, "Devfreq freed\n");
		return ;
	}
	if (vha->devfreq_init) {
		if (vha->cooling_device) {
			npu_cooling_device_unregister();
			vha->cooling_device = false;
		}
		//dev_pm_opp_of_remove_table(vha->dev);
		devfreq_unregister_opp_notifier(vha->dev, vha->devfreq);
		vha->devfreq_init = false;
		npu_dvfs_ctx.npu_on = 0;
	}
}

void vha_devfreq_suspend(struct device *dev)
{
	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	if (vha->devfreq_init) {
		if (npu_dvfs_ctx.npu_dvfs_on)
			devfreq_suspend_device(vha->devfreq);
		npu_dvfs_ctx.npu_on = 0;
	}
	return ;
}

void vha_devfreq_resume(struct device *dev)
{

	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	if (!vha)
		return;
	if (vha->devfreq_init) {
		npu_dvfs_ctx.npu_on = 1;
		vha_set_freq_volt(npu_dvfs_ctx.pvr_last_cfg,
				npu_dvfs_ctx.mtx_last_cfg,
				npu_dvfs_ctx.ocm_last_cfg);
		if (npu_dvfs_ctx.npu_dvfs_on) {
			devfreq_resume_device(vha->devfreq);
		}
	}
	return ;
}
