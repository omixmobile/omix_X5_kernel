/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include "firmware/ilitek_fw.h"
#include "ilitek.h"


/* Debug level */
bool ipio_debug_level = DEBUG_OUTPUT;
EXPORT_SYMBOL(ipio_debug_level);

static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;

#if RESUME_BY_DDI
static struct workqueue_struct	*resume_by_ddi_wq;
static struct work_struct	resume_by_ddi_work;

static void ilitek_resume_by_ddi_work(struct work_struct *work)
{
	mutex_lock(&idev->touch_mutex);

	if (idev->gesture)
		disable_irq_wake(idev->irq_num);

	/* Set tp as demo mode and reload code if it's iram. */
	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	if (idev->fw_upgrade_mode == UPGRADE_IRAM)
		ilitek_tddi_fw_upgrade_handler(NULL);
	else
		ilitek_tddi_reset_ctrl(idev->reset);

	ilitek_plat_irq_enable();
	ipio_info("TP resume end by wq\n");
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	idev->tp_suspend = false;
	idev->skip_wake = true;
	mutex_unlock(&idev->touch_mutex);
}

void ilitek_resume_by_ddi(void)
{
	if (!resume_by_ddi_wq) {
		ipio_info("resume_by_ddi_wq is null\n");
		return;
	}

	mutex_lock(&idev->touch_mutex);

	ipio_info("TP resume start called by ddi\n");

	/*
	 * To match the timing of sleep out, the first of mipi cmd must be sent within 10ms
	 * after TP reset. Because of that, we create a wq doing host download for resume.
	 */
	atomic_set(&idev->fw_stat, ENABLE);
	ilitek_tddi_reset_ctrl(idev->reset);
	ilitek_ice_mode_ctrl(ENABLE, OFF);
	idev->ddi_rest_done = true;
	idev->resume_by_ddi = true;
	mdelay(5);
	queue_work(resume_by_ddi_wq, &(resume_by_ddi_work));

	mutex_unlock(&idev->touch_mutex);
}
#endif

int ilitek_tddi_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;

	if (atomic_read(&idev->fw_stat)) {
		ipio_err("fw upgrade processing, ignore\n");
		return -EMP_FW_PROC;
	}

	atomic_set(&idev->mp_stat, ENABLE);

	if (idev->actual_tp_mode != P5_X_FW_TEST_MODE) {
		ret = ilitek_tddi_switch_tp_mode(P5_X_FW_TEST_MODE);
		if (ret < 0) {
			ipio_err("Switch MP mode failed\n");
			ret = -EMP_MODE;
			goto out;
		}
	}

	ret = ilitek_tddi_mp_test_main(apk, lcm_on);

out:
	/*
	 * If there's running mp test with lcm off, we suspose that
	 * users will soon call resume from suspend. TP mode will be changed
	 * from MP to AP mode until resume finished.
	 */
	if (!lcm_on) {
		atomic_set(&idev->mp_stat, DISABLE);
		return ret;
	}

	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	if (idev->fw_upgrade_mode == UPGRADE_IRAM) {
		if (ilitek_tddi_fw_upgrade_handler(NULL) < 0)
			ipio_err("FW upgrade failed during mp test\n");
	} else {
		if (ilitek_tddi_reset_ctrl(idev->reset) < 0)
			ipio_err("TP Reset failed during mp test\n");
	}

	atomic_set(&idev->mp_stat, DISABLE);
	return ret;
}

int ilitek_tddi_switch_tp_mode(u8 mode)
{
	int ret = 0;
	bool ges_dbg = false;

	atomic_set(&idev->tp_sw_mode, START);

	idev->actual_tp_mode = mode;

	/* able to see cdc data in gesture mode */
	if (idev->tp_data_format == DATA_FORMAT_DEBUG &&
		idev->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		ges_dbg = true;

	switch (idev->actual_tp_mode) {
	case P5_X_FW_AP_MODE:
		ipio_info("Switch to AP mode\n");
		if (idev->fw_upgrade_mode == UPGRADE_IRAM) {
			if (ilitek_tddi_fw_upgrade_handler(NULL) < 0)
				ipio_err("FW upgrade failed\n");
		} else {
			ret = ilitek_tddi_reset_ctrl(idev->reset);
		}
		if (ret < 0)
			ipio_err("TP Reset failed\n");

		break;
	case P5_X_FW_GESTURE_MODE:
		ret = idev->gesture_move_code(idev->gesture_mode);
		if (ret < 0)
			ipio_err("Move gesture code failed\n");
		if (ges_dbg) {
			ipio_info("Enable gesture debug func\n");
			ilitek_set_tp_data_len(DATA_FORMAT_GESTURE_DEBUG);
		}
		break;
	case P5_X_FW_TEST_MODE:
		ipio_info("Switch to Test mode\n");
		ret = idev->mp_move_code();
		break;
	default:
		ipio_err("Unknown TP mode: %x\n", mode);
		ret = -1;
		break;
	}

	if (ret < 0)
		ipio_err("Switch TP mode (%d) failed \n", mode);

	ipio_debug("Actual TP mode = %d\n", idev->actual_tp_mode);
	atomic_set(&idev->tp_sw_mode, END);
	return ret;
}

int ilitek_tddi_gesture_recovery(void)
{
	int ret = 0;
	bool lock = mutex_is_locked(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, START);

	if (!lock)
		mutex_lock(&idev->touch_mutex);

	ipio_info("Doing gesture recovery\n");
	idev->force_fw_update = true;
	ret = idev->ges_recover();
	idev->force_fw_update = false;

	if (!lock)
		mutex_unlock(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, END);
	return ret;
}

void ilitek_tddi_spi_recovery(void)
{
	bool lock = mutex_is_locked(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, START);

	if (!lock)
		mutex_lock(&idev->touch_mutex);

	ipio_info("Doing spi recovery\n");
	idev->force_fw_update = true;
	if (ilitek_tddi_fw_upgrade_handler(NULL) < 0)
		ipio_err("FW upgrade failed\n");
	idev->force_fw_update = false;

	if (!lock)
		mutex_unlock(&idev->touch_mutex);

	atomic_set(&idev->esd_stat, END);
}

int ilitek_tddi_wq_esd_spi_check(void)
{
	return (idev->spi_ack() != SPI_ACK) ? -1 : 0;
}

int ilitek_tddi_wq_esd_i2c_check(void)
{
	ipio_debug("");
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	if (idev->esd_recover() < 0) {
		ipio_err("SPI ACK failed, doing spi recovery\n");
		ilitek_tddi_spi_recovery();
	}
	complete_all(&idev->esd_done);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug("Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode;

	if (read_power_status(str) < 0)
		ipio_err("Read power status failed\n");

	ipio_debug("Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
		|| strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug("Charging mode\n");
			if (ilitek_tddi_ic_func_ctrl("plug", DISABLE) < 0) // plug in
				ipio_err("Write plug in failed\n");
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug("Not charging mode\n");
			if (ilitek_tddi_ic_func_ctrl("plug", ENABLE) < 0) // plug out
				ipio_err("Write plug out failed\n");
			charge_mode = 2;
		}
	}
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
}

void ilitek_tddi_wq_ctrl(int type, int ctrl)
{
	switch (type) {
	case WQ_ESD:
		if (ENABLE_WQ_ESD || idev->wq_ctrl) {
			if (!esd_wq) {
				ipio_err("wq esd is null\n");
				break;
			}
			idev->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug("execute esd check\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(WQ_ESD_DELAY)))
					ipio_debug("esd check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ipio_debug("cancel esd wq\n");
			}
		}
		break;
	case WQ_BAT:
		if (ENABLE_WQ_BAT || idev->wq_ctrl) {
			if (!bat_wq) {
				ipio_err("WQ BAT is null\n");
				break;
			}
			idev->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_debug("execute bat check\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(WQ_BAT_DELAY)))
					ipio_debug("bat check was already on queue\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ipio_debug("cancel bat wq\n");
			}
		}
		break;
	default:
		ipio_err("Unknown WQ type, %d\n", type);
		break;
	}
}

static void ilitek_tddi_wq_init(void)
{
	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);

#if RESUME_BY_DDI
	resume_by_ddi_wq = create_singlethread_workqueue("resume_by_ddi_wq");
	WARN_ON(!resume_by_ddi_wq);
	INIT_WORK(&resume_by_ddi_work, ilitek_resume_by_ddi_work);
#endif
}
extern int tpgesture_status;
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-26> #24812 start
extern int PMU_db_pos_neg_disable_delay(int ms);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-26> #24812 end
int ilitek_tddi_sleep_handler(int mode)
{
	int ret = 0;
	bool sense_stop = true;

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->tp_sleep, START);

	if (atomic_read(&idev->fw_stat) ||
		atomic_read(&idev->mp_stat)) {
		ipio_info("fw upgrade or mp still running, ignore sleep requst\n");
		atomic_set(&idev->tp_sleep, END);
		mutex_unlock(&idev->touch_mutex);
		return 0;
	}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);
	ilitek_plat_irq_disable();

	ipio_info("Sleep Mode = %d\n", mode);

	if (idev->ss_ctrl)
		sense_stop = true;
	else if ((idev->chip->core_ver >= CORE_VER_1430))
		sense_stop = false;
	else
		sense_stop = true;

	switch (mode) {
	case TP_SUSPEND:
		ipio_info("TP suspend start\n");
	#ifdef CONFIG_AI_BSP_GESTURE_SWITCH	
	//#ifdef ILITEK_ENABLE_GESTURE
		if(1 == tpgesture_status){
			ipio_info("tpgesture_status = %d\n", tpgesture_status);
			idev->gesture = 1;
		}else
			idev->gesture = 0;
		//memset(tpgesture_value,0,10);
	//	ipio_info("clean tpgesture_value==%s\n ", tpgesture_value);
	#endif
		if (sense_stop) {
			if (ilitek_tddi_ic_func_ctrl("sense", DISABLE) < 0)
				ipio_err("Write sense stop cmd failed\n");

			if (ilitek_tddi_ic_check_busy(50, 20) < 0)
				ipio_err("Check busy timeout during suspend\n");
		}

		if (idev->gesture) {
			ilitek_tddi_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(idev->irq_num);
			ilitek_plat_irq_enable();
		} else {
			if (ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN) < 0)
				ipio_err("Write sleep in cmd failed\n");
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-26> #24812 start				
				PMU_db_pos_neg_disable_delay(15);	
				mdelay(5);
				PMU_db_pos_neg_disable_delay(15);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-26> #24812 end			
		}
		ipio_info("TP suspend end\n");
		idev->tp_suspend = true;
		idev->skip_wake = false;
		break;
	case TP_DEEP_SLEEP:
		ipio_info("TP deep suspend start\n");
		if (sense_stop) {
			if (ilitek_tddi_ic_func_ctrl("sense", DISABLE) < 0)
				ipio_err("Write sense stop cmd failed\n");

			if (ilitek_tddi_ic_check_busy(50, 20) < 0)
				ipio_err("Check busy timeout during deep suspend\n");
		}

		if (idev->gesture) {
			ilitek_tddi_switch_tp_mode(P5_X_FW_GESTURE_MODE);
			enable_irq_wake(idev->irq_num);
			ilitek_plat_irq_enable();
		} else {
			if (ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN) < 0)
				ipio_err("Write deep sleep in cmd failed\n");
		}
		ipio_info("TP deep suspend end\n");
		idev->tp_suspend = true;
		idev->skip_wake = false;
		break;
	case TP_RESUME:
		if (!idev->resume_by_ddi) {
			ipio_info("TP resume start\n");

			if (idev->gesture)
				disable_irq_wake(idev->irq_num);

			/* Set tp as demo mode and reload code if it's iram. */
			idev->actual_tp_mode = P5_X_FW_AP_MODE;
			if (idev->fw_upgrade_mode == UPGRADE_IRAM) {
				if (ilitek_tddi_fw_upgrade_handler(NULL) < 0)
					ipio_err("FW upgrade failed during resume\n");
			} else {
				if (ilitek_tddi_reset_ctrl(idev->reset) < 0)
					ipio_err("TP Reset failed during resume\n");
			}
			ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
			ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
			idev->tp_suspend = false;
			idev->skip_wake = true;
			ipio_info("TP resume end\n");
		}
		ilitek_plat_irq_enable();
		break;
	default:
		ipio_err("Unknown sleep mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	ilitek_tddi_touch_release_all_point();
	atomic_set(&idev->tp_sleep, END);
	mutex_unlock(&idev->touch_mutex);
	return ret;
}
//Antaiui <AI_BSP_CTP> <hehl> <2020-06-02> add tp display start
#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
#include  <linux/ai_device_check.h>
extern int ai_set_device_info(struct ai_device_info ai_dev_info);
extern int ai_ilitek_fw_ver;
int ilitek_register_hardware_info(void)
{
    int ret = 0;
    struct ai_device_info ai_tp_hw_info;
    ai_tp_hw_info.ai_dev_type = AI_DEVICE_TYPE_TP;
    snprintf(ai_tp_hw_info.name, AI_DEVICE_NAME_LEN, "ILITEK_TDDI-FW[0x%x]",ai_ilitek_fw_ver);
    ai_set_device_info(ai_tp_hw_info);
    return ret;
}
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-06-02> add tp display end
int ilitek_tddi_fw_upgrade_handler(void *data)
{
	int ret = 0;

	atomic_set(&idev->fw_stat, START);

	idev->fw_update_stat = FW_STAT_INIT;
	ret = ilitek_tddi_fw_upgrade(idev->fw_open);
	if (ret != 0) {
		ipio_info("FW upgrade fail\n");
		idev->fw_update_stat = FW_UPDATE_FAIL;
	} else {
		ipio_info("FW upgrade pass\n");
		idev->fw_update_stat = FW_UPDATE_PASS;
//Antaiui <AI_BSP_CTP> <hehl> <2020-06-02> add tp display start
#ifdef CONFIG_AI_BSP_MTK_DEVICE_CHECK
   		ilitek_register_hardware_info();
#endif
//Antaiui <AI_BSP_CTP> <hehl> <2020-06-02> add tp display end
	}

	if (!idev->boot) {
		idev->boot = true;
		ipio_info("Registre touch to input subsystem\n");
		ilitek_plat_input_register();
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	}

	atomic_set(&idev->fw_stat, END);
	return ret;
}

int ilitek_set_tp_data_len(int format)
{
	u8 cmd[2] = {0}, ctrl = 0;
	u16 self_key = 2;
	int ret = 0, tp_mode = idev->actual_tp_mode, len = 0;

	switch (format) {
	case DATA_FORMAT_DEMO:
	case DATA_FORMAT_GESTURE_DEMO:
		len = P5_X_DEMO_MODE_PACKET_LEN;
		ctrl = DATA_FORMAT_DEMO_CMD;
		break;
	case DATA_FORMAT_DEBUG:
	case DATA_FORMAT_GESTURE_DEBUG:
		len = (2 * idev->xch_num * idev->ych_num) + (idev->stx * 2) + (idev->srx * 2);
		len += 2 * self_key + (8 * 2) + 1 + 35;
		ctrl = DATA_FORMAT_DEBUG_CMD;
		break;
	case DATA_FORMAT_DEMO_DEBUG_INFO:
		/*only suport SPI interface now, so defult use size 1024 buffer*/
		len = P5_X_DEMO_MODE_PACKET_LEN +
			P5_X_DEMO_DEBUG_INFO_ID0_LENGTH + P5_X_INFO_HEADER_LENGTH;
		ctrl = DATA_FORMAT_DEMO_DEBUG_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_INFO:
		len = P5_X_GESTURE_INFO_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_INFO_CMD;
		break;
	case DATA_FORMAT_GESTURE_NORMAL:
		len = P5_X_GESTURE_NORMAL_LENGTH;
		ctrl = DATA_FORMAT_GESTURE_NORMAL_CMD;
		break;
	case DATA_FORMAT_GESTURE_SPECIAL_DEMO:
		if (idev->gesture_demo_ctrl == ENABLE) {
			if (idev->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				len = P5_X_GESTURE_INFO_LENGTH + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
			else
				len = P5_X_DEMO_MODE_PACKET_LEN + P5_X_INFO_HEADER_LENGTH + P5_X_INFO_CHECKSUM_LENGTH;
		} else {
			if (idev->gesture_mode == DATA_FORMAT_GESTURE_INFO)
				len = P5_X_GESTURE_INFO_LENGTH;
			else
				len = P5_X_GESTURE_NORMAL_LENGTH;
		}
		ipio_info("Gesture demo mode control = %d\n",  idev->gesture_demo_ctrl);
		ilitek_tddi_ic_func_ctrl("gesture_demo_en", idev->gesture_demo_ctrl);
		ipio_info("knock_en setting\n");
		ilitek_tddi_ic_func_ctrl("knock_en", 0x8);
		break;
	default:
		ipio_err("Unknow TP data format\n");
		return -1;
	}

	idev->tp_data_format = format;
	idev->tp_data_len = len;
	ipio_info("TP mode = %d, format = %d, len = %d\n",
		tp_mode, idev->tp_data_format, idev->tp_data_len);

	if (tp_mode == P5_X_FW_AP_MODE ||
		format == DATA_FORMAT_GESTURE_DEMO ||
		format == DATA_FORMAT_GESTURE_DEBUG) {
		cmd[0] = P5_X_MODE_CONTROL;
		cmd[1] = ctrl;
		ret = idev->write(cmd, 2);

		if (ret < 0) {
			ipio_err("switch to format %d failed\n", format);
			ilitek_tddi_switch_tp_mode(P5_X_FW_AP_MODE);
		}
	} else if (tp_mode == P5_X_FW_GESTURE_MODE) {
		ret = ilitek_tddi_ic_func_ctrl("lpwg", ctrl);
		if (ret < 0)
			ipio_err("write gesture mode failed\n");
	}

	return ret;
}

void ilitek_tddi_report_handler(void)
{
	int ret = 0, pid = 0;
	u8 checksum = 0;
	u8 *trdata = NULL;
	int rlen = 0;
	int tmp = ipio_debug_level;

	/* Just in case these stats couldn't be blocked in top half context */
	if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep)) {
		ipio_info("ignore report request\n");
		return;
	}

	if (idev->irq_after_recovery) {
		ipio_info("ignore int triggered by recovery\n");
		idev->irq_after_recovery = false;
		return;
	}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE) {
		__pm_stay_awake(idev->ws);
		/* Waiting for pm resume completed */
		ipio_info("zzt_22 idev->actual_tp_mode = %x\n",idev->actual_tp_mode);
		//mdelay(40);
		msleep(40);//up 25ms --deep sleep wait spi resueme
	}

	rlen = idev->tp_data_len;
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify start	
	//ipio_debug("Packget length = %d\n", rlen);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify end
	if (!rlen || rlen > TR_BUF_SIZE) {
		ipio_err("Length of packet (%d) is invaild\n", rlen);
		goto out;
	}

	memset(idev->tr_buf, 0x0, TR_BUF_SIZE);

	ret = idev->read(idev->tr_buf, rlen);
	if (ret < 0) {
		ipio_err("Read report packet failed, ret = %d\n", ret);
		if (ret == DO_SPI_RECOVER) {
			ilitek_tddi_ic_get_pc_counter();
			if ((idev->actual_tp_mode == P5_X_FW_GESTURE_MODE) && idev->gesture && !idev->prox_near) {
				ipio_err("Gesture failed, doing gesture recovery\n");
				if (ilitek_tddi_gesture_recovery() < 0)
					ipio_err("Failed to recover gesture\n");
				idev->irq_after_recovery = true;
			} else {
				ipio_err("SPI ACK failed, doing spi recovery\n");
				ilitek_tddi_spi_recovery();
				idev->irq_after_recovery = true;
			}
		}
		goto out;
	}

	if (ret == SPI_IS_LOCKED)
		goto out;

	if (ret > TR_BUF_SIZE) {
		ipio_err("Returned length (%d) is invaild\n", ret);
		goto out;
	}

	rlen = ret;

	ilitek_dump_data(idev->tr_buf, 8, rlen, 0, "finger report");

	checksum = ilitek_calc_packet_checksum(idev->tr_buf, rlen - 1);

	if ((checksum != idev->tr_buf[rlen-1]) && (idev->fw_uart_en == DISABLE)) {
		ipio_err("Wrong checksum, checksum = %x, buf = %x, len = %d\n", checksum, idev->tr_buf[rlen-1], rlen);
		ipio_debug_level = DEBUG_ALL;
		ilitek_dump_data(idev->tr_buf, 8, rlen, 0, "finger report with wrong");
		ipio_debug_level = tmp;
		goto out;
	}

	pid = idev->tr_buf[0];
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify start	
	//ipio_debug("Packet ID = %x\n", pid);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify end	
	trdata = idev->tr_buf;
	if (pid == P5_X_INFO_HEADER_PACKET_ID) {
		trdata = idev->tr_buf + P5_X_INFO_HEADER_LENGTH;
		pid = trdata[0];
	}

	switch (pid) {
	case P5_X_DEMO_PACKET_ID:
		ilitek_tddi_report_ap_mode(trdata, rlen);
		break;
	case P5_X_DEBUG_PACKET_ID:
		ilitek_tddi_report_debug_mode(trdata, rlen);
		break;
	case P5_X_I2CUART_PACKET_ID:
		ilitek_tddi_report_i2cuart_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_PACKET_ID:
		ilitek_tddi_report_gesture_mode(trdata, rlen);
		break;
	case P5_X_GESTURE_FAIL_ID:
		ipio_info("gesture fail reason code = 0x%02x", trdata[1]);
		break;
	case P5_X_DEMO_DEBUG_INFO_PACKET_ID:
		demo_debug_info_mode(trdata, rlen);
		break;
	default:
		ipio_err("Unknown packet id, %x\n", pid);
		break;
	}

out:
	if (idev->actual_tp_mode != P5_X_FW_GESTURE_MODE) {
		ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
		ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	}

	if (idev->actual_tp_mode == P5_X_FW_GESTURE_MODE)
		__pm_relax(idev->ws);
}

int ilitek_tddi_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&idev->tp_reset, START);

	if (mode != TP_IC_CODE_RST) {
		idev->skip_wake = false;
		ilitek_tddi_ic_check_otp_prog_mode();
	}

	switch (mode) {
	case TP_IC_CODE_RST:
		ipio_info("TP IC Code RST \n");
		ret = ilitek_tddi_ic_code_reset();
		if (ret < 0)
			ipio_err("IC Code reset failed\n");
		break;
	case TP_IC_WHOLE_RST:
		ipio_info("TP IC whole RST\n");
		ret = ilitek_tddi_ic_whole_reset();
		if (ret < 0)
			ipio_err("IC whole reset failed\n");
		break;
	case TP_HW_RST_ONLY:
		ipio_info("TP HW RST\n");
		ilitek_plat_tp_reset();
		break;
	default:
		ipio_err("Unknown reset mode, %d\n", mode);
		ret = -EINVAL;
		break;
	}

	/*
	 * Since OTP must be folloing with reset, except for code rest,
	 * the stat of ice mode should be set as 0.
	 */
	if (mode != TP_IC_CODE_RST)
		atomic_set(&idev->ice_stat, DISABLE);

	idev->fw_uart_en = DISABLE;
	idev->tp_data_format = DATA_FORMAT_DEMO;
	idev->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;
	atomic_set(&idev->tp_reset, END);
	idev->skip_wake = true;
	return ret;
}

static int ilitek_get_tp_module(void)
{
	/*
	 * TODO: users should implement this function
	 * if there are various tp modules been used in projects.
	 */

	return 0;
}

static void ilitek_update_tp_module_info(void)
{
	int module;

	module = ilitek_get_tp_module();

	switch (module) {
	case MODEL_CSOT:
		idev->md_name = "CSOT";
		idev->md_fw_filp_path = CSOT_FW_FILP_PATH;
		idev->md_fw_rq_path = CSOT_FW_REQUEST_PATH;
		idev->md_ini_path = CSOT_INI_NAME_PATH;
		idev->md_ini_rq_path = CSOT_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_CSOT;
		idev->md_fw_ili_size = sizeof(CTPM_FW_CSOT);
		break;
	case MODEL_AUO:
		idev->md_name = "AUO";
		idev->md_fw_filp_path = AUO_FW_FILP_PATH;
		idev->md_fw_rq_path = AUO_FW_REQUEST_PATH;
		idev->md_ini_path = AUO_INI_NAME_PATH;
		idev->md_ini_rq_path = AUO_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_AUO;
		idev->md_fw_ili_size = sizeof(CTPM_FW_AUO);
		break;
	case MODEL_BOE:
		idev->md_name = "BOE";
		idev->md_fw_filp_path = BOE_FW_FILP_PATH;
		idev->md_fw_rq_path = BOE_FW_REQUEST_PATH;
		idev->md_ini_path = BOE_INI_NAME_PATH;
		idev->md_ini_rq_path = BOE_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_BOE;
		idev->md_fw_ili_size = sizeof(CTPM_FW_BOE);
		break;
	case MODEL_INX:
		idev->md_name = "INX";
		idev->md_fw_filp_path = INX_FW_FILP_PATH;
		idev->md_fw_rq_path = INX_FW_REQUEST_PATH;
		idev->md_ini_path = INX_INI_NAME_PATH;
		idev->md_ini_rq_path = INX_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_INX;
		idev->md_fw_ili_size = sizeof(CTPM_FW_INX);
		break;
	case MODEL_DJ:
		idev->md_name = "DJ";
		idev->md_fw_filp_path = DJ_FW_FILP_PATH;
		idev->md_fw_rq_path = DJ_FW_REQUEST_PATH;
		idev->md_ini_path = DJ_INI_NAME_PATH;
		idev->md_ini_rq_path = DJ_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_DJ;
		idev->md_fw_ili_size = sizeof(CTPM_FW_DJ);
		break;
	case MODEL_TXD:
		idev->md_name = "TXD";
		idev->md_fw_filp_path = TXD_FW_FILP_PATH;
		idev->md_fw_rq_path = TXD_FW_REQUEST_PATH;
		idev->md_ini_path = TXD_INI_NAME_PATH;
		idev->md_ini_rq_path = TXD_FW_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_TXD;
		idev->md_fw_ili_size = sizeof(CTPM_FW_TXD);
		break;
	case MODEL_TM:
		idev->md_name = "TM";
		idev->md_fw_filp_path = TM_FW_REQUEST_PATH;
		idev->md_fw_rq_path = TM_FW_REQUEST_PATH;
		idev->md_ini_path = TM_INI_NAME_PATH;
		idev->md_ini_rq_path = TM_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_TM;
		idev->md_fw_ili_size = sizeof(CTPM_FW_TM);
		break;
	default:
		break;
	}

	if (module == 0 || idev->md_fw_ili_size < ILI_FILE_HEADER) {
		ipio_err("Couldn't find any tp modules, applying default settings\n");
		idev->md_name = "DEF";
		idev->md_fw_filp_path = DEF_FW_FILP_PATH;
		idev->md_fw_rq_path = DEF_FW_REQUEST_PATH;
		idev->md_ini_path = DEF_INI_NAME_PATH;
		idev->md_ini_rq_path = DEF_INI_REQUEST_PATH;
		idev->md_fw_ili = CTPM_FW_DEF;
		idev->md_fw_ili_size = sizeof(CTPM_FW_DEF);
	}

	ipio_info("Found %s module: ini path = %s, fw path = (%s, %s, %d)\n",
			idev->md_name,
			idev->md_ini_path,
			idev->md_fw_filp_path,
			idev->md_fw_rq_path,
			idev->md_fw_ili_size);

	idev->tp_module = module;
}

int ilitek_tddi_init(void)
{
#if BOOT_FW_UPDATE
	struct task_struct *fw_boot_th;
#endif

	ipio_info("driver version = %s\n", DRIVER_VERSION);

	mutex_init(&idev->touch_mutex);
	mutex_init(&idev->debug_mutex);
	mutex_init(&idev->debug_read_mutex);
	init_waitqueue_head(&(idev->inq));
	spin_lock_init(&idev->irq_spin);
	init_completion(&idev->esd_done);

	atomic_set(&idev->irq_stat, DISABLE);
	atomic_set(&idev->ice_stat, DISABLE);
	atomic_set(&idev->tp_reset, END);
	atomic_set(&idev->fw_stat, END);
	atomic_set(&idev->mp_stat, DISABLE);
	atomic_set(&idev->tp_sleep, END);
	atomic_set(&idev->mp_int_check, DISABLE);
	atomic_set(&idev->esd_stat, END);

	ilitek_tddi_ic_init();
	ilitek_tddi_wq_init();

	/* Must do hw reset once in first time for work normally if tp reset is avaliable */
	if (!TDDI_RST_BIND)
		if (ilitek_tddi_reset_ctrl(idev->reset) < 0)
			ipio_err("TP Reset failed during init\n");

	idev->do_otp_check = ENABLE;
	idev->fix_ice = DISABLE;
	idev->fw_uart_en = DISABLE;
	idev->force_fw_update = DISABLE;
	idev->demo_debug_info[0] = demo_debug_info_id0;
	idev->tp_data_format = DATA_FORMAT_DEMO;
	idev->boot = false;

	/*
	 * This status of ice enable will be reset until process of fw upgrade runs.
	 * it might cause unknown problems if we disable ice mode without any
	 * codes inside touch ic.
	 */
	if (ilitek_ice_mode_ctrl(ENABLE, OFF) < 0) {
		ipio_err("Not found ilitek chips\n");
		return -ENODEV;
	}

	if (ilitek_tddi_ic_get_info() < 0)
		ipio_err("Chip info is incorrect\n");

	ilitek_update_tp_module_info();

	ilitek_tddi_node_init();

	ilitek_tddi_fw_read_flash_info();

#if BOOT_FW_UPDATE
	fw_boot_th = kthread_run(ilitek_tddi_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ipio_err("Failed to create fw upgrade thread\n");
	}
#else
	if (ilitek_ice_mode_ctrl(DISABLE, OFF) < 0)
		ipio_err("Failed to disable ice mode failed during init\n");

#if (TDDI_INTERFACE == BUS_I2C)
	idev->info_from_hex = DISABLE;
#endif

	ilitek_tddi_ic_get_core_ver();
	ilitek_tddi_ic_get_protocl_ver();
	ilitek_tddi_ic_get_fw_ver();
	ilitek_tddi_ic_get_tp_info();
	ilitek_tddi_ic_get_panel_info();

#if (TDDI_INTERFACE == BUS_I2C)
	idev->info_from_hex = ENABLE;
#endif

	ipio_info("Registre touch to input subsystem\n");
	ilitek_plat_input_register();
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	idev->boot = true;
#endif

	idev->ws = wakeup_source_register(NULL,"ili_wakelock");
	if (!idev->ws)
		ipio_err("wakeup source request failed\n");

	return 0;
}

void ilitek_tddi_dev_remove(void)
{
	ipio_info("remove ilitek dev\n");

	if (!idev)
		return;

	gpio_free(idev->tp_int);
	gpio_free(idev->tp_rst);

	if (esd_wq != NULL) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq != NULL) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}

	if (idev->ws)
		wakeup_source_unregister(idev->ws);

	kfree(idev->tr_buf);
	kfree(idev->gcoord);
	ilitek_tddi_interface_dev_exit(idev);
}

int ilitek_tddi_dev_init(struct ilitek_hwif_info *hwif)
{
	ipio_info("TP Interface: %s\n", (hwif->bus_type == BUS_I2C) ? "I2C" : "SPI");
	return ilitek_tddi_interface_dev_init(hwif);
}
