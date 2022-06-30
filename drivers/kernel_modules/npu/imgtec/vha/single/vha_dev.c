/*
 *****************************************************************************
 * Copyright (c) Imagination Technologies Ltd.
 *
 * The contents of this file are subject to the MIT license as set out below.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the terms of the
 * GNU General Public License Version 2 ("GPL")in which case the provisions of
 * GPL are applicable instead of those above.
 *
 * If you wish to allow use of your version of this file only under the terms
 * of GPL, and not to allow others to use your version of this file under the
 * terms of the MIT license, indicate your decision by deleting the provisions
 * above and replace them with the notice and other provisions required by GPL
 * as set out in the file called "GPLHEADER" included in this distribution. If
 * you do not delete the provisions above, a recipient may use your version of
 * this file under the terms of either the MIT license or GPL.
 *
 * This License is also included in this distribution in the file called
 * "MIT_COPYING".
 *
 *****************************************************************************/

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/moduleparam.h>
#include <linux/pm_runtime.h>

#include <uapi/vha.h>
#include "vha_common.h"
#include "vha_plat.h"
#include "vha_regs.h"

#if defined(CFG_SYS_VAGUS)
#include <hwdefs/nn_sys_cr_vagus.h>
#endif

#define ERR_EVENT_DESC(b) VHA_CR_OS(VHA_EVENT_STATUS_VHA_##b##_EN), __stringify(b)

static void vha_dev_disable_events(struct vha_dev *vha)
{
	img_pdump_printf("-- Clear CNN events\n");
	IOWRITE64_PDUMP(VHA_EVNTS_DEFAULT, VHA_CR_OS(VHA_EVENT_CLEAR));
	img_pdump_printf("-- Disable CNN events\n");
	IOWRITE64_PDUMP(0, VHA_CR_OS(VHA_EVENT_ENABLE));
	/* Clear the START bit !
	 * Note: It is stated that writing 0 to this bit has no effect,
	 * however in error cases, some hw blocks may start
	 * to process previous requests after turning on the clocks
	 * which was previously disabled */
	IOWRITE64_PDUMP(0, VHA_CR_OS(CNN_CONTROL));

	/* Disable core events */
	img_pdump_printf("-- Disable CORE events\n");
	IOWRITE64_PDUMP(0, VHA_CR_OS(VHA_EVENT_ENABLE));
}

__maybe_unused
static void vha_dev_enable_clocks(struct vha_dev *vha)
{
	uint64_t __maybe_unused sys_clks = 0;
	uint64_t __maybe_unused main_clks = 0;

	/* Always AUTO gating  when needed */
	sys_clks = VHA_SYS_CLOCKS_DEFAULT(AUTO);
	main_clks = VHA_MAIN_CLOCKS_DEFAULT(AUTO);
	/* Enable sys clocks ! */
	img_pdump_printf("-- Enable SYS clocks\n");
	IOWRITE64_PDUMP(sys_clks, VHA_CR_SYS_CLK_CTRL0);
	/* Enable main clocks ! */
	img_pdump_printf("-- Enable MAIN clocks\n");
	IOWRITE64_PDUMP(main_clks, VHA_CR_CLK_CTRL0);
#if defined(CFG_SYS_VAGUS)
	img_pdump_printf("-- Enable NN_SYS clocks\n");
	IOWRITE64_PDUMP_REGIO(NN_SYS_CR_CLK_CTRL_MODE_AUTO,
			NN_SYS_CR_BASE, NN_SYS_CR_CLK_CTRL, "REG_NNSYS");
#endif
}

static void vha_dev_ready(struct vha_dev *vha)
{
#ifndef CONFIG_VHA_DUMMY
	if (!vha->is_ready)
		return;
#endif
	dev_dbg(vha->dev, "%s\n", __func__);

	vha_dev_wait(vha);

	/* Finally enable ALL events */
	img_pdump_printf("-- Enable ALL events\n");
	IOWRITE64_PDUMP(VHA_EVNTS_DEFAULT, VHA_CR_OS(VHA_EVENT_ENABLE));
	img_pdump_printf("-- Clear ALL events\n");
	IOWRITE64_PDUMP(VHA_EVNTS_DEFAULT, VHA_CR_OS(VHA_EVENT_CLEAR));
#ifdef HW_AX2
	img_pdump_printf("-- Clear CNN status\n");
	IOWRITE64_PDUMP(0, VHA_CR_OS(CNN_STATUS));
#endif
	img_pdump_printf("-- Clear MMU fault status\n");
	IOWRITE64_PDUMP(0, VHA_CR_OS(MMU_FAULT_STATUS1));
	img_pdump_printf("-- Clear SLC debug status\n");
	IOWRITE64_PDUMP(0, VHA_CR_SLC_STATUS_DEBUG);
	img_pdump_printf("-- Reset PERF counters\n");
	IOWRITE64_PDUMP(0, VHA_CR_PERF_RESET_FULL);
}

__maybe_unused
static int vha_dev_reset(struct vha_dev *vha)
{
	img_pdump_printf("-- Set RESET bits\n");
#if defined(CFG_SYS_VAGUS)
	IOWRITE64_PDUMP_REGIO(NN_SYS_CR_RESET_CTRL_NN_SYS_EN,
			NN_SYS_CR_BASE, NN_SYS_CR_RESET_CTRL, "REG_NNSYS");
#endif
	/* Perform reset procedure */
	IOWRITE64_PDUMP(VHA_RESET_DEFAULT, VHA_CR_RESET_CTRL);

	/* poll for reset deassertion
	 * count=16, delay=256cycles
	 */
	img_pdump_printf("-- Wait for RESET deassertion\n");
#if defined(CFG_SYS_VAGUS)
	IOPOLL64_PDUMP_REGIO(0, 16, 256, NN_SYS_CR_RESET_CTRL_MASKFULL,
			NN_SYS_CR_BASE, NN_SYS_CR_RESET_CTRL, "REG_NNSYS");
#endif
	IOPOLL64_PDUMP(0, 16, 256, VHA_CR_RESET_CTRL_MASKFULL,
					VHA_CR_RESET_CTRL);
	return 0;
}

__maybe_unused
static int vha_dev_disable_clocks(struct vha_dev *vha)
{
	/* If auto gating was turned on, wait for clocks idle state */
	img_pdump_printf("-- Wait for clocks IDLE state\n");
	IOPOLL64_PDUMP(0, 100, 1000,
			VHA_CR_CLK_STATUS0_MASKFULL,
			VHA_CR_CLK_STATUS0);
#if defined(CFG_SYS_VAGUS)
	IOPOLL64_PDUMP_REGIO(0, 100, 1000, NN_SYS_CR_CLK_STATUS_MASKFULL,
			NN_SYS_CR_BASE, NN_SYS_CR_CLK_STATUS, "REG_NNSYS");
#endif
	/* Wait for MMU,CCM,RDI,XBAR  IDLE state */
	img_pdump_printf("-- Wait for memory bus interface IDLE state\n");
	IOPOLL64_PDUMP(0xFFFF, 100, 1000, VHA_CR_SLC_IDLE_MASKFULL,
			VHA_CR_SLC_IDLE);

	/* Finally disable clocks */
	img_pdump_printf("-- Disable MAIN clocks\n");
	IOWRITE64_PDUMP(0, VHA_CR_CLK_CTRL0); /* main */
	img_pdump_printf("-- Disable SYS clocks\n");
	IOWRITE64_PDUMP(0, VHA_CR_SYS_CLK_CTRL0); /* sys */
#if defined(CFG_SYS_VAGUS)
	img_pdump_printf("-- NN_SYS clocks\n");
	IOWRITE64_PDUMP_REGIO(0, NN_SYS_CR_BASE,
			NN_SYS_CR_CLK_CTRL, "REG_NNSYS"); /* nn_sys */
#endif
	return 0;
}

/* start the device */
int vha_dev_start(struct vha_dev *vha)
{
	int ret = 0;

	/* Cancel APM request if new inference comes */
	cancel_delayed_work(&vha->apm_dworks[0].dwork);

	if (vha->state == VHA_STATE_ON)
		return 0; /* not an error */

	dev_dbg(vha->dev, "%s\n", __func__);

/* Assuming OS0 is the privileged one */
#if _OSID_ == 0 /* For HW_AX2 this is always true */
	/////////////// POWER ON //////////////////////////
	img_pdump_printf("-- POWER_ON_BEGIN\n");

	/* Prepare device ...  */
	ret = vha_dev_prepare(vha);
	if (ret) {
		dev_err(vha->dev, "%s: Error preparing device!\n", __func__);
		return ret;
	}
	/* Reset device */
	ret = vha_dev_reset(vha);
	if (ret){
		dev_err(vha->dev, "%s: Error reseting device!\n", __func__);
		return ret;
	}
	/* Enable device clocks */
	vha_dev_enable_clocks(vha);
	img_pdump_printf("-- POWER_ON_END\n");
	/* Call device specific setup */
	vha_dev_setup(vha);
	/////////////////////////////////////////////////////
#endif

	vha_dev_ready(vha);

	vha->state = VHA_STATE_ON;
	/* Remember the time hw is powered on */
	getnstimeofday(&vha->stats.hw_start);
	return ret;
}

/* stop the device */
int vha_dev_stop(struct vha_dev *vha, bool reset)
{
	int ret = 0;

	if (vha->state == VHA_STATE_OFF)
		return 0;  /* not an error */

	/* Cancel APM request if we are about to power off the core */
	cancel_delayed_work(&vha->apm_dworks[0].dwork);

	dev_dbg(vha->dev, "%s\n", __func__);
	/* Disable events at first */
	vha_dev_disable_events(vha);

	vha->is_ready = false;
/* Assuming OS0 is the privileged one */
#if _OSID_ == 0 /* For HW_AX2 */
	/////////////// POWER_OFF //////////////////////////
	img_pdump_printf("-- POWER_OFF_BEGIN\n");
	/* Reset core in case of error or pending inference */
	if (reset)
		ret = vha_dev_reset(vha);
	if(ret)
		dev_warn(vha->dev,
			"%s: Problem with resetting device!\n",
			__func__);

	/* Disable device clocks */
	ret = vha_dev_disable_clocks(vha);
	if(ret)
		dev_warn(vha->dev,
			"%s: Problem with disabling clocks!\n",
			__func__);

	img_pdump_printf("-- POWER_OFF_END\n");
#endif

	vha->state = VHA_STATE_OFF;
	/* Update the up time of the core */
	if (!vha->do_calibration) {
		uint64_t tmp = 0;
		struct timespec now;
		getnstimeofday(&now);
		if (get_timespan_us(&vha->stats.hw_start, &now, &tmp)) {
			do_div(tmp, 1000UL);
			vha->stats.uptime_ms += tmp;
			if (vha->stats.uptime_ms)
				vha_update_utilization(vha);
			else
				dev_dbg(vha->dev,
					"%s Too short execution time to calculate utilization!\n",
					__func__);
		} else
			WARN_ON(1);
	}

	vha->active_mmu_ctx = VHA_INVALID_ID;

	spin_lock_irq(&vha->irq_lock);
	vha->irq_status = 0;
	vha->irq_count = 0;
	vha->stream_count = 0;
	spin_unlock_irq(&vha->irq_lock);

	return ret;
}

void vha_update_utilization(struct vha_dev *vha)
{
	uint64_t tmp;
	tmp = vha->stats.cnn_total_proc_us;
	do_div(tmp, vha->stats.uptime_ms);
	vha->stats.cnn_utilization = tmp;
}

/* Top half */
irqreturn_t vha_handle_irq(struct device *dev)
{
	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	int ret = IRQ_HANDLED;
	uint64_t event_status;
#ifdef VHA_DEVFREQ
	ktime_t now;
#endif
	if (!vha)
		return IRQ_NONE;

	event_status = IOREAD64(vha->reg_base, VHA_CR_OS(VHA_EVENT_STATUS));
	/* On fpga platform it is possible to get
	 * a spurious interrupt when the hw died
	 * Do not proceed, just throw a warning */
	if (event_status == VHA_DEAD_HW || event_status == ~0) {
		WARN_ONCE(1, "Hardware is dead!");
		return IRQ_NONE;
	}

#ifdef VHA_SCF
	if (vha->core_props.supported.parity &&
			!vha->parity_disable) {
		bool par_bit = img_mem_calc_parity(event_status &
				~VHA_CR_BITMASK(VHA_EVENT_STATUS_TYPE, PARITY));
		if (par_bit !=
				VHA_CR_GETBITS(VHA_EVENT_STATUS_TYPE, PARITY,
						event_status)) {
			dev_err(dev, "Event status register parity error!\n");
			/* Use the real event to indicate the error */
			event_status |=  VHA_CR_OS(VHA_EVENT_STATUS_VHA_PARITY_ERROR_EN);
		}
		/* Clear the PARITY bit - it's not a valid event */
		VHA_CR_CLEARBITS(event_status, VHA_EVENT_STATUS_TYPE, PARITY);
	}
#endif

	if (event_status & VHA_EVNTS_DEFAULT) {
		uint64_t cnn_status;
		uint8_t count;

		/* clear the interrupt:
		 * best not to write pdump in interrupts */
		IOWRITE64(vha->reg_base, VHA_CR_OS(VHA_EVENT_CLEAR),
				event_status & VHA_EVNTS_DEFAULT);

		/* Read the stream count as single IRQ may be raised for multiple kicks */
		cnn_status = IOREAD64(vha->reg_base, VHA_CR_OS(CNN_STATUS));

#ifdef VHA_SCF
		if (vha->core_props.supported.parity &&
				!vha->parity_disable) {
			bool par_bit = img_mem_calc_parity(cnn_status &
					~VHA_CR_BITMASK_OS(CNN_STATUS, PARITY));
			if (par_bit != VHA_CR_GETBITS_OS(CNN_STATUS, PARITY, cnn_status)) {
				dev_err(dev, "CNN status register parity error!\n");
				/* Use the real event to indicate the error */
				event_status |=  VHA_CR_OS(VHA_EVENT_STATUS_VHA_PARITY_ERROR_EN);
			}
		}
#endif
		/* Read the stream count as single IRQ may be raised for multiple kicks */
		count = VHA_CR_GETBITS_OS(CNN_STATUS, STREAM_COUNT, cnn_status);

		spin_lock(&vha->irq_lock);
		/* store the status to be processed later */
		vha->irq_status |= event_status;
		if (vha->low_latency == VHA_LL_SELF_KICK)
			/* Two separate IRQs may be raised for multiple kicks */
			vha->irq_count += count - vha->stream_count;
		else
			/* Only single IRQ may be raised otherwise ... */
			vha->irq_count = count - vha->stream_count;
		vha->stream_count = count;
#ifdef VHA_DEVFREQ
		now = ktime_get();
		vha_update_dvfs_state(vha, false, &now);
#endif
		/* Record hw processing end timestamps */
		vha->stats.hw_proc_end_prev = vha->stats.hw_proc_end;
		getnstimeofday(&vha->stats.hw_proc_end);
		spin_unlock(&vha->irq_lock);

		ret = IRQ_WAKE_THREAD;
	} else
		return IRQ_NONE;

	dev_dbg(dev, "IRQ 0x%08llx\n", event_status);

	return ret;
}

static bool vha_rollback_cnn_cmds(struct vha_dev *vha)
{
	bool processing = false;
	/* Not processed commands are still on the pending list
	 * of each session, so just mark the hw pending lists as empty */
	if (vha->pendcmd[VHA_CNN_CMD].cmd) {
		vha->pendcmd[VHA_CNN_CMD].cmd->in_hw = false;
		vha->pendcmd[VHA_CNN_CMD].cmd->queued = false;
		vha->pendcmd[VHA_CNN_CMD].cmd = NULL;
		processing = true;
		vha->stats.cnn_kicks_aborted++;
	}
	/* low_latency ...*/
	if (vha->queuedcmd[VHA_CNN_CMD].cmd) {
		vha->queuedcmd[VHA_CNN_CMD].cmd->in_hw = false;
		vha->queuedcmd[VHA_CNN_CMD].cmd->queued = false;
		vha->queuedcmd[VHA_CNN_CMD].cmd = NULL;
		if (vha->low_latency == VHA_LL_SELF_KICK)
			vha->stats.cnn_kicks_aborted++;
	}
	dev_dbg(vha->dev, "%s: (%d)\n", __func__, processing);

	return processing;
}

bool vha_rollback_cmds(struct vha_dev *vha)
{
	return vha_rollback_cnn_cmds(vha);
}

/*
 * handles the command (of given cmd_idx) already processed by the hw.
 */
static bool vha_handle_cmd(struct vha_dev *vha, int cmd_idx, int status)
{
	struct vha_cmd *cmd = NULL;

	if (cmd_idx >= VHA_CMD_MAX)
		return false;

	cmd = vha->pendcmd[cmd_idx].cmd;
	if (unlikely(!cmd)) {
		dev_dbg(vha->dev, "No command. Probably it has been aborted\n");
		return false;
	}

	{
		uint64_t proc_time = 0;
		struct timespec *from = &cmd->hw_proc_start;
		struct timespec *to = &vha->stats.hw_proc_end;

		if (timespec_compare(&vha->stats.hw_proc_end_prev, &cmd->hw_proc_start) >= 0)
			from = &vha->stats.hw_proc_end_prev;

		if (get_timespan_us(from, to, &proc_time)) {
			vha->stats.last_proc_us = proc_time;
		} else {
			vha->stats.last_proc_us = 0;
		}
		/* Update cnn stats */
		vha_cnn_update_stats(vha);
	}

	if (cmd_idx == VHA_CNN_CMD)
		vha_cnn_cmd_completed(cmd, status);

	if (status) {
		/* Rollback any queued command ... */
		vha_rollback_cnn_cmds(vha);
		/* Notify immediately current command */
		vha_cmd_notify(cmd);

		return false;
	}

	if (vha->queuedcmd[cmd_idx].cmd)
		vha->pendcmd[cmd_idx].cmd = vha->queuedcmd[cmd_idx].cmd;
	else
		vha->pendcmd[cmd_idx].cmd = NULL;

	vha->queuedcmd[cmd_idx].cmd = NULL;
	dev_dbg(vha->dev,
			"%s: %p -> new pending %p\n",
			__func__, cmd, vha->pendcmd[cmd_idx].cmd);

	vha_cmd_notify(cmd);

	return true;
}

static void vha_do_queued_cmd(struct vha_dev *vha, int cmd_idx)
{
	struct vha_cmd *cmd, *pend;

	cmd = vha->queuedcmd[cmd_idx].cmd;

	dev_dbg(vha->dev,
			"%s: queued %p pending %p\n",
			__func__, cmd, vha->pendcmd[cmd_idx].cmd);

	if (!cmd || (cmd &&
				((vha->low_latency == VHA_LL_DISABLED ||
				vha->low_latency == VHA_LL_SELF_KICK) ||
						!cmd->queued))) {
		dev_dbg(vha->dev, "%s: skipping!\n", __func__);
		return;
	}

	/* store actual pending command as it will be modified */
	pend = vha->pendcmd[cmd_idx].cmd;

	/* at this point we should be able to process the cmd */
	vha_do_cnn_cmd(cmd);

	/* restore pending */
	vha->pendcmd[cmd_idx].cmd = pend;
}

static int vha_report_failure(struct vha_dev *vha, uint64_t status,
		const struct vha_biterr bits[], int bits_size)
{
	int error = 0;
	int i;
	int cmdid = -1;

	if (vha->pendcmd[VHA_CNN_CMD].cmd)
		cmdid = vha->pendcmd[VHA_CNN_CMD].cmd->user_cmd.cmd_id;

	if (vha_observers.error)
		vha_observers.error(vha->id, cmdid, status);

	/* event status in human readable form */
	for (i = 0; i < bits_size; i++) {
		if (status & bits[i].b) {
			dev_err(vha->dev,
				" event status: %s\n",
				bits[i].s);
			/* convert from register bits into POSIX errno
			* if multiple errors, then arbitrary errno choice */
			error = bits[i].e;
		}
	}

	return error;
}

/* if vha event register reports CNN events, so handle them */
static int vha_handle_cnn_event(struct vha_dev *vha, uint64_t event_status)
{
	int err = 0;

	if (vha_check_calibration(vha))
		return 0;

	if (event_status & VHA_CNN_ERR_EVNTS) {
		static const struct vha_biterr err_bits[] = {
			{-ETIMEDOUT, ERR_EVENT_DESC(CNN0_MEM_WDT)},
#ifdef HW_AX2
			{-ETIMEDOUT, ERR_EVENT_DESC(CNN0_WDT)},
#endif
			{-EIO,       ERR_EVENT_DESC(CNN0_ERROR)}
		};

		err = vha_report_failure(vha,
				event_status, err_bits, ARRAY_SIZE(err_bits));

		vha_cnn_dump_status(vha);
	}

	/* Poke the hw if there were already
	 * command queued in the hw */
	if (!err)
		vha_do_queued_cmd(vha, VHA_CNN_CMD);
	/* Handle actual command */
	if (vha_handle_cmd(vha, VHA_CNN_CMD, err) == false)
		err = -ENOENT;

	return err;
}

#ifdef CONFIG_VHA_DUMMY_SIMULATE_HW_PROCESSING_TIME
/* Simulating hw execution time by scheduling this delayed work. */
void vha_dummy_worker(struct work_struct *work)
{
	struct vha_dev *vha = container_of(work, struct vha_dev, dummy_dwork.work);

	mutex_lock(&vha->lock);

	if (vha->pendcmd[VHA_CNN_CMD].cmd) {
		/* Record hw processing end timestamps */
		vha->stats.hw_proc_end_prev = vha->stats.hw_proc_end;
		getnstimeofday(&vha->stats.hw_proc_end);
		/* Handle current pending command */
		vha_handle_cnn_event(vha, VHA_CNN_CMPLT_EVNT);
		/* Schedule following commands */
		vha_chk_cmd_queues(vha, true);
	}

	mutex_unlock(&vha->lock);
}
#endif

/* Bottom half */
irqreturn_t vha_handle_thread_irq(struct device *dev)
{
	struct vha_dev *vha = vha_dev_get_drvdata(dev);
	irqreturn_t ret = IRQ_HANDLED;
	uint64_t status;
	uint8_t count, c = 0;
	int err = 0;

	if (!vha)
		return IRQ_NONE;

	mutex_lock(&vha->lock);

	spin_lock_irq(&vha->irq_lock);
	status = vha->irq_status;
	vha->irq_status = 0;
	count = vha->irq_count;
	vha->irq_count = 0;
	if (!count) {
		uint64_t proc_time = 0;

		if (get_timespan_us(&vha->stats.hw_proc_start, &vha->stats.hw_proc_end,
					&proc_time)) {
			vha->stats.last_proc_us = proc_time;
		} else {
			vha->stats.last_proc_us = 0;
		}
	}
	spin_unlock_irq(&vha->irq_lock);
	/* Command may have been aborted before this handler is executed */
	if (!status) {
		mutex_unlock(&vha->lock);
		return ret;
	}

	dev_dbg(dev, "%s: status:%llx count:%d\n",
			__func__, status, count);

	do {
		if (status & VHA_CORE_EVNTS) {
			static const struct vha_biterr err_bits[] = {
				{-EIO,       ERR_EVENT_DESC(AXI_ERROR)},
				{-EFAULT,    ERR_EVENT_DESC(MMU_PAGE_FAULT)},
#ifdef HW_AX3
#ifdef VHA_SCF
				{-EIO,       ERR_EVENT_DESC(MMU_PARITY_ERROR)},
				{-EIO,       ERR_EVENT_DESC(PARITY_ERROR)},
				{-EIO,       ERR_EVENT_DESC(LOCKSTEP_ERROR)},
#endif
				{-ETIMEDOUT, ERR_EVENT_DESC(HL_WDT)},
				{-EIO,       ERR_EVENT_DESC(ERROR)}
#endif
			};

#ifdef HW_AX3
			if (status & VHA_EVENT_TYPE(HL_WDT)
					&& vha->is_ready)
				if (vha_check_calibration(vha))
					break;

			if ((status & VHA_CORE_EVNTS)==
					VHA_EVENT_TYPE(READY)
					&& !vha->is_ready) {
				vha->is_ready = true;
				vha_dev_ready(vha);
				if (vha->do_calibration) {
					vha_cnn_start_calib(vha);
					break;
				} else
					vha_chk_cmd_queues(vha, true);
			}
#endif

			err = vha_report_failure(vha, status,
					err_bits, ARRAY_SIZE(err_bits));
			if (err) {
				dev_err(vha->dev, "NNA hw failure: %llx\n", status);
				dev_err(vha->dev, "   CLK_STATUS0:%llx ",
					IOREAD64(vha->reg_base, VHA_CR_CLK_STATUS0));
				dev_err(vha->dev, " VHA_EVENT_STATUS:%llx ", status);
			}

			if (status & VHA_EVENT_TYPE(MMU_PAGE_FAULT))
				/* dump mmu status */
				vha_mmu_status(vha);
		}

		/* If no core level error process cnn events */
		if (!err && status & VHA_CNN_EVNTS)
			err = vha_handle_cnn_event(vha, status);
#ifdef HW_AX3
		else if (status == VHA_EVENT_TYPE(ERROR)) {
			/* Resubmit command next time if no CNN error detected
			 * and only ERROR bit is set.
			 * That means other OS caused the error */
			vha_rollback_cnn_cmds(vha);
		}
#endif
		else if (err && vha->is_ready) { /* Core level error */
			if (vha_handle_cmd(vha, VHA_CNN_CMD, err) == false)
				err = -ENOENT;
		}

		c++;
	} while (c < count && !err);

	if (err) {
		vha->stats.total_failures += count ? count : 1;
		vha_dev_stop(vha, true);
		/* Check queues ... */
		vha_chk_cmd_queues(vha, true);
	} else {
		/* Run in BH context! */
		vha_chk_cmd_queues(vha, false);
	}
	vha->stats.cnn_kicks_completed += count;
	mutex_unlock(&vha->lock);

	return ret;
}

bool vha_rm_session_cmds(struct vha_session *session)
{
	struct vha_dev *vha = session->vha;
	bool pend_removed = false;
	bool queued_removed = false;
	bool reschedule = false;

	/* Check if pend/queued commands will be removed. */
	if (vha->pendcmd[VHA_CNN_CMD].cmd &&
			vha->pendcmd[VHA_CNN_CMD].cmd->session == session) {
		dev_warn(vha->dev,
				"Removing a session while cnn cmd is still pending\n");
		pend_removed = true;
#ifdef CONFIG_VHA_DUMMY_SIMULATE_HW_PROCESSING_TIME
		cancel_delayed_work(&vha->dummy_dwork);
#endif
	}
	if (vha->queuedcmd[VHA_CNN_CMD].cmd &&
			vha->queuedcmd[VHA_CNN_CMD].cmd->session == session) {
		dev_warn(vha->dev,
				"Removing a session while cnn cmd is still queued\n");
		queued_removed = true;
	}

	/* Update session scheduling. */
	if (vha->queuedcmd[VHA_CNN_CMD].cmd &&
			(pend_removed && !queued_removed)) {
		if (vha->queuedcmd[VHA_CNN_CMD].cmd->session !=
					list_entry(&vha->sched_sessions, struct vha_session,
								sched_list))
			while(list_first_entry(&vha->sched_sessions, struct vha_session,
						sched_list) != vha->queuedcmd[VHA_CNN_CMD].cmd->session)
				list_rotate_left(&vha->sched_sessions);
	}

	/* Remove pend/queued commands if needed. */
	if (pend_removed || queued_removed) {
		vha_rollback_cnn_cmds(vha);
		/* Need to reschedule too. */
		reschedule = true;
	}

	return reschedule;
}

bool vha_rm_session_cmds_masked(struct vha_session *session, uint32_t cmd_id,
		uint32_t cmd_id_mask)
{
	struct vha_dev *vha = session->vha;
	bool reschedule = false;
	bool pend_removed = false;
	bool queued_removed = false;

	/* Check if pend/queued commands will be removed. */
	if (vha->pendcmd[VHA_CNN_CMD].cmd &&
			(vha->pendcmd[VHA_CNN_CMD].cmd->session == session) &&
			(vha->pendcmd[VHA_CNN_CMD].cmd->user_cmd.cmd_id & cmd_id_mask)
																	== cmd_id) {
		pend_removed = true;
#ifdef CONFIG_VHA_DUMMY_SIMULATE_HW_PROCESSING_TIME
		cancel_delayed_work(&vha->dummy_dwork);
#endif
	}
	if (vha->queuedcmd[VHA_CNN_CMD].cmd &&
			(vha->queuedcmd[VHA_CNN_CMD].cmd->session == session) &&
			(vha->queuedcmd[VHA_CNN_CMD].cmd->user_cmd.cmd_id & cmd_id_mask)
																	== cmd_id) {
		queued_removed = true;
	}

	/* Update session scheduling. */
	if (vha->queuedcmd[VHA_CNN_CMD].cmd &&
			(pend_removed && !queued_removed)) {
		if (vha->queuedcmd[VHA_CNN_CMD].cmd->session !=
					list_entry(&vha->sched_sessions, struct vha_session,
								sched_list))
			while(list_first_entry(&vha->sched_sessions, struct vha_session,
						sched_list) != vha->queuedcmd[VHA_CNN_CMD].cmd->session)
				list_rotate_left(&vha->sched_sessions);
	}

	/* Remove pend/queued commands if needed. */
	if (pend_removed || queued_removed) {
		vha_rollback_cnn_cmds(vha);
		reschedule = true;
	}

	return reschedule;
}

bool vha_is_busy(struct vha_dev *vha)
{
#ifndef CONFIG_VHA_DUMMY
	if (!vha->is_ready)
		return true;
#endif

	if (vha->low_latency != VHA_LL_DISABLED) {
		return vha->pendcmd[VHA_CNN_CMD].cmd != NULL ||
				vha->queuedcmd[VHA_CNN_CMD].cmd != NULL;
	}
	return vha->pendcmd[VHA_CNN_CMD].cmd != NULL;
}

/* returns true if the cmd queue is full */
bool vha_is_queue_full(struct vha_dev *vha, struct vha_cmd *cmd)
{
	if (vha->low_latency != VHA_LL_DISABLED) {
		if (vha->low_latency == VHA_LL_SELF_KICK
#ifdef HW_AX3
			/* if current command we are trying to queue belongs to a different session than pending one */
			&& (vha->pendcmd[VHA_CNN_CMD].cmd != NULL && cmd != NULL &&
					vha->pendcmd[VHA_CNN_CMD].cmd->session != cmd->session)
			/* if session of the command we are trying to queue, shares the hw mmu ctx with the session of pending cmd */
			&& (cmd->session->mmu_ctxs[VHA_MMU_REQ_MODEL_CTXID].hw_id ==
					vha->pendcmd[VHA_CNN_CMD].cmd->session->mmu_ctxs[VHA_MMU_REQ_MODEL_CTXID].hw_id)
			/* Sanity if hw mmu ctx is really shared at this point */
			&& (vha->mmu_ctxs[cmd->session->mmu_ctxs[VHA_MMU_REQ_MODEL_CTXID].hw_id] > 1)
			) {
#else
			) {
			dev_warn(vha->dev, "%s: LL=2 not supported!\n", __func__);
#endif
			/* skip low latency mode */
			return vha->pendcmd[VHA_CNN_CMD].cmd != NULL;
		}

		return vha->pendcmd[VHA_CNN_CMD].cmd != NULL &&
					vha->queuedcmd[VHA_CNN_CMD].cmd != NULL;
	}
	return vha->pendcmd[VHA_CNN_CMD].cmd != NULL;
}

void vha_scheduler_loop(struct vha_dev *vha)
{
	struct vha_cmd *cmd, *tmp;
	struct vha_session *session, *wait_session = NULL;
	bool queued, in_hw;
	bool retry = false;
	bool retrying = false;
	enum do_cmd_status cmd_status = CMD_OK;

	if (vha_is_queue_full(vha, NULL)) {
		/* Postpone worker task if command queue is full. */
		dev_dbg(vha->dev, "%s Queue full. Postpone worker task!\n", __func__);
		return;
	}

	do {
		list_for_each_entry(session, &vha->sched_sessions, sched_list) {
			list_for_each_entry_safe(cmd, tmp, &session->cmds, list) {

				/* Reset previous state. */
				queued = false;
				in_hw = false;
				/* For hw commands... */
				if (CMD_EXEC_ON_HW(cmd)) {
					if (!VHA_IS_DUMMY(vha)) {
						/* Start device. */
						if(vha_dev_start(vha))
							return;
					}
					/* Store previous state. */
					queued = cmd->queued;
					in_hw = cmd->in_hw;
				}

				/* Attempt to schedule command for execution. */
				cmd_status = vha_do_cmd(cmd);
				if (cmd_status == CMD_NOTIFIED) {
					continue;
				}

				/* For hw commands... */
				if (CMD_EXEC_ON_HW(cmd)) {
					/* For low latency processing... */
					if (vha->low_latency != VHA_LL_DISABLED) {
						if (cmd->in_hw) {
							if (!retrying) {
								/* Set for retrying in case nothing will be queued
								 * in this round */
								retry = true;
							} else {
								/* Retry only once. */
								retry = false;
								/* It's a retry round, so nothing was queued in the previous
								 * one. Check the command following the one in the hardware
								 * for this session then. */
								continue;
							}
							/* If command is already in hw, go to next session. */
							break;
						}
						if (!cmd->inbufs_ready) {
							/* If command is waiting for input buffers, set this session
							 * as a starting point for next scheduling round. */
							if (wait_session == NULL)
								wait_session = session;
							/* Go to the next session. */
							break;
						}
						if (cmd->queued != queued) {
							/* If command has just been queued, set following session
							 * as a starting point for the next scheduling round. */
							if (wait_session == NULL) {
								wait_session = list_next_entry(session, sched_list);
								if (&wait_session->sched_list == &vha->sched_sessions)
									wait_session = list_first_entry(&vha->sched_sessions,
																	struct vha_session,
																	sched_list);
							}
							/* Quit this scheduling round as there's no room for more
							 * commands to be scheduled. */
							goto skip_cmds;
						}
						if (cmd->queued == queued && cmd->in_hw == in_hw) {
							/* If command has neither been scheduled for execution nor queued,
							 * quit this scheduling round as there's no room for
							 * commands to be scheduled. */
							goto skip_cmds;
						}
					} else {
						/* For non low latency processing... */
						if (cmd->in_hw) {
							/* If command is already in hw, set following session
							 * as a starting point for the next scheduling round. */
							if (wait_session == NULL) {
								wait_session = list_next_entry(session, sched_list);
								if (&wait_session->sched_list == &vha->sched_sessions)
									wait_session = list_first_entry(&vha->sched_sessions,
																	struct vha_session,
																	sched_list);
							}
							/* Quit this scheduling round as there's no room for more
							 * commands to be scheduled. */
							goto skip_cmds;
						}
						if (!cmd->inbufs_ready) {
							/* If command is waiting for input buffers, set this session
							 * as a starting point for next scheduling round. */
							if (wait_session == NULL)
								wait_session = session;
							/* Go to the next session. */
							break;
						}
						if (cmd->in_hw == in_hw) {
							/* If command has not been scheduled for execution,
							 * quit this scheduling round as there's no room for
							 * commands to be scheduled. */
							goto skip_cmds;
						}
					}
				}
			}
		}
		retrying = true;
	} while (retry);

	if (!VHA_IS_DUMMY(vha)) {
		/* Schedule APM if needed */
		if (!vha_is_busy(vha) &&
				!vha->no_clock_disable) {
			if (!vha->pm_delay) {
				if (vha_dev_stop(vha, false)) {
					dev_warn(vha->dev, "%s: Failed to soft stop device. trying with reset",
						__func__);
					if (vha_dev_stop(vha, true))
						dev_err(vha->dev, "%s: Failed to stop device with reset!", __func__);
				}
			}
			else
				vha_sched_apm(vha, &vha->apm_dworks[0]);
		}
	}

skip_cmds:
	/* Set a starting point session for next scheduling round. */
	if (wait_session != NULL)
		session = wait_session;
	if (session != list_entry(&vha->sched_sessions, struct vha_session,
														sched_list))
		while(list_first_entry(&vha->sched_sessions, struct vha_session,
													sched_list) != session)
			list_rotate_left(&vha->sched_sessions);
}

void vha_dev_apm_stop(struct vha_dev *vha, struct vha_apm_work *apm_work)
{
	if (!vha->do_calibration &&
			(vha->pendcmd[VHA_CNN_CMD].cmd == NULL &&
			vha->queuedcmd[VHA_CNN_CMD].cmd == NULL))
		if (vha_dev_stop(vha, false)) {
			dev_warn(vha->dev, "%s: Failed to soft stop device. trying with reset",
				__func__);
			if (vha_dev_stop(vha, true))
				dev_err(vha->dev, "%s: Failed to stop device with reset!", __func__);
		}
}

int vha_dev_get_props(struct vha_dev *vha, uint32_t onchipmem_size)
{
	struct vha_core_props *props = &vha->core_props;
	uint64_t ip_config;
	uint32_t ocm_size_kb = 0;

	memset(props, 0, sizeof(*props));

#ifdef CONFIG_VHA_DUMMY
	/* Note: dummy dev always reads zeroes from registers */
	props->product_id  = 0x8070605040302010ULL;
	props->core_id  = (long)HW_SERIES << (int)VHA_CR_CORE_ID_BRANCH_ID_SHIFT;
	props->core_id += 0x010203040506ULL;   // provide a dummy core id
	props->dummy_dev = true;
	props->num_cnn_core_devs = 1;
#else
	props->product_id  = IOREAD64(vha->reg_base, VHA_CR_PRODUCT_ID);
	props->core_id  = IOREAD64(vha->reg_base, VHA_CR_CORE_ID);
#endif
	props->skip_bvnc_check = false;
	/*
	 * New mmu version 3 and onwards operates on 40bit physical & virtual addresses
	 */
	props->mmu_width = 40;

	/* HW from 1.1 onwards */
	ip_config = IOREAD64(vha->reg_base, VHA_CR_CORE_IP_CONFIG);
#ifdef HW_AX3
	props->mmu_ver = VHA_CR_GETBITS(CORE_IP_CONFIG, MMU_VERSION, ip_config);
#endif
	/* Mirage uses MMU version 3 hardware */
	if (!props->mmu_ver)
		props->mmu_ver = 3;
			;
	if (VHA_CR_GETBITS(CORE_IP_CONFIG, CNN_SUPPORTED, ip_config))
		props->num_cnn_core_devs = 1;
	if (VHA_CR_GETBITS(CORE_IP_CONFIG, RTM_SUPPORTED, ip_config))
		props->supported.rtm = 1;
#ifdef HW_AX3
	if (VHA_CR_GETBITS(CORE_IP_CONFIG, PARITY_REGISTERS, ip_config))
		props->supported.parity = 1;

#if defined(CONFIG_VHA_DUMMY) && defined(VHA_SCF)
	/* Force parity for pdump generation */
	props->supported.parity = 1;
#endif
#endif

	if ((props->num_cnn_core_devs == 0)
		|| VHA_CR_GETBITS(CORE_ID, BRANCH_ID, props->core_id) != HW_SERIES) {
		dev_err(vha->dev, "%s: Wrong core configuration detected. "
			"Expected BVNC %d.x.x.x, got %llu.x.x.x. "
			"Maybe kernel module was built with wrong params.\n",
			__func__, HW_SERIES,
			VHA_CR_GETBITS(CORE_ID, BRANCH_ID, props->core_id));
		return -ENODEV;
	}

	props->soc_axi  = IOREAD64(vha->reg_base, VHA_CR_SOC_AXI);

	dev_info(vha->dev, "%s: Product id: %#llx\n",
			__func__, props->product_id);
	dev_info(vha->dev, "%s: Core id: %#llx\n",
			__func__, props->core_id);
	dev_info(vha->dev, "%s: MMU version:%d (%dbit)\n",
			__func__, props->mmu_ver, props->mmu_width);
	dev_dbg(vha->dev, "%s: supported: %#x\n",
			__func__, props->features);
	dev_dbg(vha->dev, "%s: soc_axi: %#llx\n",
			__func__, props->soc_axi);
	{
		uint64_t tmp = IOREAD64(vha->reg_base,
				VHA_CR_CORE_IP_INTEGRATOR_ID);
		dev_dbg(vha->dev, "%s: ip integrator id: %#llx\n",
				__func__, tmp);
		tmp = IOREAD64(vha->reg_base, VHA_CR_CORE_IP_CHANGELIST);
		dev_dbg(vha->dev, "%s: ip change list: %llu\n", __func__, tmp);
	}

#if defined(CFG_SYS_VAGUS)
	ocm_size_kb = IOREAD64(vha->reg_base, NN_SYS_CR(CORE_IP_CONFIG)) &
				~NN_SYS_CR_CORE_IP_CONFIG_NN_SYS_OCM_RAM_SIZE_4KB_CLRMSK;
	ocm_size_kb *= 4;
#endif

	if (ocm_size_kb) {
		vha->core_props.locm_size_bytes = ocm_size_kb * 1024;
		/* User may wanted to limit OCM ... */
		if (onchipmem_size) {
			if (onchipmem_size < vha->core_props.locm_size_bytes) {
				dev_warn(vha->dev, "%s:Limiting onchip memory to %u bytes (available:%u)\n",
						__func__, onchipmem_size, vha->core_props.locm_size_bytes);
				vha->core_props.locm_size_bytes = onchipmem_size;
			} else if (onchipmem_size > vha->core_props.locm_size_bytes) {
				dev_err(vha->dev, "%s: User defined onchip memory size exceeded (%u > %u))\n",
						__func__, onchipmem_size, vha->core_props.locm_size_bytes);
			}
		}
	} else {
		vha->core_props.locm_size_bytes = onchipmem_size;
	}

	dev_info(vha->dev, "%s: Total onchip memory: %u [kB]\n",
			__func__, vha->core_props.locm_size_bytes / 1024);

	dev_info(vha->dev, "%s: Devices: DUMMY:%u CNN:%u\n", __func__,
			props->dummy_dev ? props->num_cnn_core_devs : 0,
			props->dummy_dev ? 0 : props->num_cnn_core_devs);

	return 0;
}

void vha_dev_ocm_configure(struct vha_dev *vha)
{
#if defined(CFG_SYS_VAGUS)
	dev_dbg(vha->dev, "%s: OCM address range: %#lx - %#lx\n",
			__func__, vha->ocm_paddr,
			vha->ocm_paddr + vha->core_props.locm_size_bytes - 1);
	IOWRITE64(vha->reg_base, NN_SYS_CR(NOC_LOWER_ADDR1), vha->ocm_paddr);
	IOWRITE64(vha->reg_base, NN_SYS_CR(NOC_UPPER_ADDR1),
			vha->ocm_paddr + vha->core_props.locm_size_bytes - 1);
	img_pdump_printf("-- Setup NN_SYS OCM phys address range\n"
		"WRW "_PMEM_":$0 :OCM:BLOCK_CACHE:0x0\n"
		"WRW64 :REG_NNSYS:%#x "_PMEM_":$0\n"
		"WRW "_PMEM_":$0 :OCM:BLOCK_CACHE:%#x\n"
		"WRW64 :REG_NNSYS:%#x "_PMEM_":$0\n",
		NN_SYS_CR_NOC_LOWER_ADDR1, vha->core_props.locm_size_bytes-1,
		NN_SYS_CR_NOC_UPPER_ADDR1);
#endif
}

/* prepare CRC and DEBUG data buffers */
void vha_dbg_prepare_hwbufs(struct vha_session *session, uint8_t mask, struct vha_crc_config_regs *regs)
{
	struct vha_dev *vha = session->vha;
	(void)mask;

	if (session->cnn_dbg.cnn_crc_buf[0]) {
		struct vha_buffer *buf = session->cnn_dbg.cnn_crc_buf[0];
		uint64_t val64;

		/* enable CRC: address + mode */
		val64 = VHA_CR_SETBITS_OS(CNN_CRC_CONTROL, CNN_CRC_ENABLE,
				session->cnn_dbg.cnn_crc_mode);
		img_pdump_printf("-- CRC_CONTROL=%u buf 'CRC' size=%zx\n",
				session->cnn_dbg.cnn_crc_mode, buf->size);
		IOWRITE_PDUMP_BUFADDR(session, buf, 0, VHA_CR_OS(CNN_CRC_ADDRESS));

		IOWRITE64_PDUMP(val64, VHA_CR_OS(CNN_CRC_CONTROL));

#ifdef HW_AX3
		img_pdump_printf("-- CRC_MASK=%#x\n", session->cnn_dbg.cnn_crc_mask);
		IOWRITE64_PDUMP(session->cnn_dbg.cnn_crc_mask, VHA_CR_OS(CNN_CRC_MASK_CTRL));
#endif
	}
	if (session->cnn_dbg.cnn_dbg_buf[0] && session->cnn_dbg.cnn_dbg_pdump_enable) {
		struct vha_buffer *buf = session->cnn_dbg.cnn_dbg_buf[0];
		uint64_t val64;

		/* enable DEBUG: address, perf mode, band mode */
		img_pdump_printf("-- DEBUG_CONTROL=%u,%u buf 'DBG' size=%zx\n",
				session->cnn_dbg.cnn_dbg_modes[0], session->cnn_dbg.cnn_dbg_modes[1],
				buf->size);
		IOWRITE_PDUMP_BUFADDR(session, buf, 0,
					VHA_CR_OS(CNN_DEBUG_ADDRESS));
		val64 = VHA_CR_ALIGN_SETBITS_OS(CNN_DEBUG_SIZE,
						CNN_DEBUG_SIZE,
						buf->size);
		IOWRITE64_PDUMP(val64, VHA_CR_OS(CNN_DEBUG_SIZE));

		/* Set the CONTROL register only if requested */
		if (session->cnn_dbg.cnn_dbg_modes[0] > 0 || session->cnn_dbg.cnn_dbg_modes[1] > 0) {
			val64 = VHA_CR_SETBITS_OS(CNN_DEBUG_CONTROL,
						 CNN_PERF_ENABLE,
						 session->cnn_dbg.cnn_dbg_modes[0]);
			val64 |= VHA_CR_SETBITS_OS(CNN_DEBUG_CONTROL,
						 CNN_BAND_ENABLE,
						 session->cnn_dbg.cnn_dbg_modes[1]);
			IOWRITE64_PDUMP(val64, VHA_CR_OS(CNN_DEBUG_CONTROL));
		}
	}
}

/* flush CRC and DEBUG data buffers */
void vha_dbg_flush_hwbufs(struct vha_session *session, char checkpoint, uint8_t mask)
{
	struct vha_dev* vha = session->vha;
	(void)mask;
	if (session->cnn_dbg.cnn_dbg_flush != checkpoint)
		return;

	if (session->cnn_dbg.cnn_crc_buf[0] && !session->use_cmd_crc_buf) {
		struct vha_buffer *buf = session->cnn_dbg.cnn_crc_buf[0];
		/*
		 * TOBEDONE: calculate CRC buffer size based
		 * on num passes, num layers, etc
		 */
		img_pdump_printf("-- Save signatures\n");
		img_pdump_printf("IF CHECK_CRCS\n");
		img_pdump_printf("COM Checking CRCs ...\n");
		vha_pdump_sab_buf(session, PDUMP_CRC,
					buf, 0, buf->size);
		img_pdump_printf("ELSE CHECK_CRCS\n");
		img_pdump_printf("COM Not checking CRCs!\n");
		img_pdump_printf("FI CHECK_CRCS\n");
	}
	if (session->cnn_dbg.cnn_dbg_buf[0] && session->cnn_dbg.cnn_dbg_pdump_enable) {
		struct vha_buffer *buf = session->cnn_dbg.cnn_dbg_buf[0];
		/* read the size of the DEBUG buffer */
		uint64_t size = IOREAD64(vha->reg_base, VHA_CR_OS(CNN_DEBUG_STATUS));
		/*
		 * SAB the DBG buffer, even though "it is not deterministic"
		 */
		size = VHA_CR_GETBITS_OS(CNN_DEBUG_STATUS,
					CNN_DEBUG_OFFSET,
					size);
		img_pdump_printf("-- Save DEBUG info\n");

		vha_pdump_sab_buf(session, PDUMP_DBG, buf, 0, buf->size);
	}
}

/* stop capturing CRC and DEBUG data */
void vha_dbg_stop_hwbufs(struct vha_session *session, uint8_t mask)
{
	struct vha_dev *vha = session->vha;
	(void)mask;

	/* Flush hw debug buffers */
	vha_dbg_flush_hwbufs(session, 0, 0);

	if (session->cnn_dbg.cnn_crc_buf[0] || session->use_cmd_crc_buf) {
		IOWRITE64_PDUMP(0, VHA_CR_OS(CNN_CRC_CONTROL));
	}
	if (session->cnn_dbg.cnn_dbg_buf[0]) {
		/* read the size of the DEBUG buffer */
		uint64_t size = IOREAD64(vha->reg_base, VHA_CR_OS(CNN_DEBUG_STATUS));

		if (session->cnn_dbg.cnn_dbg_modes[0] > 0 || session->cnn_dbg.cnn_dbg_modes[1] > 0) {
			IOWRITE64_PDUMP(0, VHA_CR_OS(CNN_DEBUG_CONTROL));
			/* just give a hint in the pdump:
			 * dummy device returns 0 */
			img_pdump_printf(
					"-- POL64 :REG:%#x 0 0 0 1 1 -- DEBUG_STATUS=%llx\n",
					 VHA_CR_OS(CNN_DEBUG_STATUS),
				size);
		}
	}
}

uint64_t vha_dbg_rtm_read(struct vha_dev *vha, uint64_t addr)
{
	/* Turn on all clocks forcefully */
	IOWRITE64(vha->reg_base, VHA_CR_SYS_CLK_CTRL0, VHA_SYS_CLOCKS_DEFAULT(ON));
	IOWRITE64(vha->reg_base, VHA_CR_CLK_CTRL0, VHA_MAIN_CLOCKS_DEFAULT(ON));

	/* Set up address of the signal */
	IOWRITE64(vha->reg_base, VHA_CR_RTM_CTRL, addr | VHA_CR_RTM_CTRL_RTM_ENABLE_EN);


	/* but N_OF_RTM_STAGES is not accessible by SW*/
	/* so waiting 1 ms for now */
	msleep(1);

	/* Read the data */
	return IOREAD64(vha->reg_base, VHA_CR_RTM_DATA);
}

/* List of predefined registers to be shown in debugfs */
const struct vha_reg vha_regs[] = {
#define REG_DESC(reg) VHA_CR_##reg, VHA_CR_##reg##_MASKFULL
#define REG_DESC_OS(reg) VHA_CR_OS(reg), VHA_CR_OS(reg##_MASKFULL)
	{"main_clocks_control  ", REG_DESC(CLK_CTRL0)},
	{"main_clocks_status   ", REG_DESC(CLK_STATUS0)},
	{"sys_clocks_control   ", REG_DESC(SYS_CLK_CTRL0)},
	{"sys_clocks_status    ", REG_DESC(SYS_CLK_STATUS0)},
	{"product_id           ", REG_DESC(PRODUCT_ID)},
	{"core_id              ", REG_DESC(CORE_ID)},
	{"soc_axi              ", REG_DESC(SOC_AXI)},
	{"integrator_id        ", REG_DESC(CORE_IP_INTEGRATOR_ID)},
	{"ip_changelist        ", REG_DESC(CORE_IP_CHANGELIST)},
	{"core_ip_config       ", REG_DESC(CORE_IP_CONFIG)},
	{"reset                ", REG_DESC(RESET_CTRL)},
	{"event_enable         ", REG_DESC_OS(VHA_EVENT_ENABLE)},
	{"event_status         ", REG_DESC_OS(VHA_EVENT_STATUS)},
	{"cnn_control          ", REG_DESC_OS(CNN_CONTROL)},
	{"cnn_status           ", REG_DESC_OS(CNN_STATUS)},
#ifdef HW_AX2
	{"cnn_wdt_cmpmatch     ", REG_DESC(CNN_WDT_COMPAREMATCH)},
	{"cnn_wdt_control      ", REG_DESC(CNN_WDT_CTRL)},
	{"cnn_wdt_timer        ", REG_DESC(CNN_WDT_TIMER)},
#endif
	{"cnn_mem_wdt_cmpmatch ", REG_DESC(CNN_MEM_WDT_COMPAREMATCH)},
	{"cnn_mem_wdt_control  ", REG_DESC(CNN_MEM_WDT_CTRL)},
	{"cnn_mem_wdt_timer    ", REG_DESC(CNN_MEM_WDT_TIMER)},
	{"mmu_control          ", REG_DESC_OS(MMU_CTRL)},
	{"mmu_context          ", REG_DESC_OS(MMU_CBASE_MAPPING_CONTEXT)},
	{"mmu_mapping          ", REG_DESC_OS(MMU_CBASE_MAPPING)},
	{"mmu_status           ", REG_DESC(MMU_STATUS)},
	{"mmu_fault_status1    ", REG_DESC_OS(MMU_FAULT_STATUS1)},
	{"mmu_fault_status2    ", REG_DESC_OS(MMU_FAULT_STATUS2)},
	{"slc_control          ", REG_DESC(SLC_CTRL)},
#if 0
	{"slc_bypass_control   ", REG_DESC(SLC_BYPASS_CTRL)},
#endif
	{"slc_status1          ", REG_DESC(SLC_STATUS1)},
	{"slc_status2          ", REG_DESC(SLC_STATUS2)},
	{"slc_status3          ", REG_DESC(SLC_STATUS3)},
	{"slc_idle             ", REG_DESC(SLC_IDLE)},
	{"bif_outstanding_read ", REG_DESC(BIF_OUTSTANDING_READ)},
#undef REG_DESC
#undef REG_DESC_OS
	{NULL                   , 0},
};

