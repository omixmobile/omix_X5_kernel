/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */


#include <linux/proc_fs.h>
#include <linux/seq_file.h>
//#include <linux/delay.h>	//jx:Moved to nt36xxx.h
//#include <asm/uaccess.h>	//jx:Moved to nt36xxx.h

#include "nt36xxx.h"

#if NVT_TOUCH_EXT_PROC
#define NVT_FW_VERSION "nvt_fw_version"
#define NVT_BASELINE "nvt_baseline"
#define NVT_RAW "nvt_raw"
#define NVT_DIFF "nvt_diff"
#define NVT_UPDATE "nvt_update"//[20180903,jx]Modify Testing sfor VIVO APK call MP function!
#define NVT_TRIMID "nvt_trimid"
#define NVT_CMDTEST "nvt_CmdTest"	//To Trigger Test Cmd


#define BUS_TRANSFER_LENGTH  256

#define NORMAL_MODE 0x00
#define TEST_MODE_1 0x21
#define TEST_MODE_2 0x22
#define HANDSHAKING_HOST_READY 0xBB

#define XDATA_SECTOR_SIZE   256

static uint8_t xdata_tmp[2048] = {0};
static int32_t xdata[2048] = {0};
//static int32_t xdata_i[2048] = {0};	//[20190815,jx]Remove NT36870 support.
//static int32_t xdata_q[2048] = {0};	//[20190815,jx]Remove NT36870 support.

static struct proc_dir_entry *NVT_proc_fw_version_entry;
static struct proc_dir_entry *NVT_proc_baseline_entry;
static struct proc_dir_entry *NVT_proc_raw_entry;
static struct proc_dir_entry *NVT_proc_diff_entry;
static struct proc_dir_entry *NVT_proc_fwupdate_entry;
static struct proc_dir_entry *NVT_proc_trimid_entry;
static struct proc_dir_entry *NVT_proc_cmdTest_entry;

/*******************************************************
Description:
	Novatek touchscreen change mode function.

return:
	n.a.
*******************************************************/
void nvt_change_mode(uint8_t mode)
{
	uint8_t buf[8] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	//---set mode---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = mode;
	CTP_SPI_WRITE(ts->client, buf, 2);

	if (mode == NORMAL_MODE) {
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = HANDSHAKING_HOST_READY;
		CTP_SPI_WRITE(ts->client, buf, 2);
		msleep(20);
	}
}

/*******************************************************
Description:
	Novatek touchscreen get firmware pipe function.

return:
	Executive outcomes. 0---pipe 0. 1---pipe 1.
*******************************************************/
uint8_t nvt_get_fw_pipe(void)
{
	uint8_t buf[8]= {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

	//---read fw status---
	buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
	buf[1] = 0x00;
	CTP_SPI_READ(ts->client, buf, 2);

	//NVT_LOG("FW pipe=%d, buf[1]=0x%02X\n", (buf[1]&0x01), buf[1]);

	return (buf[1] & 0x01);
}

/*******************************************************
Description:
	Novatek touchscreen read meta data function.

return:
	n.a.
*******************************************************/
void nvt_read_mdata(uint32_t xdata_addr, uint32_t xdata_btn_addr)
{
	int32_t i = 0;
	int32_t j = 0;
	int32_t k = 0;
	uint8_t buf[BUS_TRANSFER_LENGTH + 1] = {0};
	uint32_t head_addr = 0;
	int32_t dummy_len = 0;
	int32_t data_len = 0;
	int32_t residual_len = 0;

	//---set xdata sector address & length---
	head_addr = xdata_addr - (xdata_addr % XDATA_SECTOR_SIZE);
	dummy_len = xdata_addr - head_addr;
	data_len = ts->x_num * ts->y_num * 2;
	residual_len = (head_addr + dummy_len + data_len) % XDATA_SECTOR_SIZE;

	//printk("head_addr=0x%05X, dummy_len=0x%05X, data_len=0x%05X, residual_len=0x%05X\n", head_addr, dummy_len, data_len, residual_len);

	//read xdata : step 1
	for (i = 0; i < ((dummy_len + data_len) / XDATA_SECTOR_SIZE); i++) {
		//---change xdata index---
		nvt_set_page(head_addr + XDATA_SECTOR_SIZE * i);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (XDATA_SECTOR_SIZE / BUS_TRANSFER_LENGTH); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[XDATA_SECTOR_SIZE * i + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04X\n", buf[k+1], (XDATA_SECTOR_SIZE*i + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (head_addr+XDATA_SECTOR_SIZE*i));
	}

	//read xdata : step2
	if (residual_len != 0) {
		//---change xdata index---
		nvt_set_page(xdata_addr + data_len - residual_len);

		//---read xdata by BUS_TRANSFER_LENGTH
		for (j = 0; j < (residual_len / BUS_TRANSFER_LENGTH + 1); j++) {
			//---read data---
			buf[0] = BUS_TRANSFER_LENGTH * j;
			CTP_SPI_READ(ts->client, buf, BUS_TRANSFER_LENGTH + 1);

			//---copy buf to xdata_tmp---
			for (k = 0; k < BUS_TRANSFER_LENGTH; k++) {
				xdata_tmp[(dummy_len + data_len - residual_len) + BUS_TRANSFER_LENGTH * j + k] = buf[k + 1];
				//printk("0x%02X, 0x%04x\n", buf[k+1], ((dummy_len+data_len-residual_len) + BUS_TRANSFER_LENGTH*j + k));
			}
		}
		//printk("addr=0x%05X\n", (xdata_addr+data_len-residual_len));
	}

	//---remove dummy data and 2bytes-to-1data---
	for (i = 0; i < (data_len / 2); i++) {
		xdata[i] = (int16_t)(xdata_tmp[dummy_len + i * 2] + 256 * xdata_tmp[dummy_len + i * 2 + 1]);
	}

#if TOUCH_KEY_NUM > 0
	//read button xdata : step3
	//---change xdata index---
	nvt_set_page(xdata_btn_addr);

	//---read data---
	buf[0] = (xdata_btn_addr & 0xFF);
	CTP_SPI_READ(ts->client, buf, (TOUCH_KEY_NUM * 2 + 1));

	//---2bytes-to-1data---
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		xdata[ts->x_num * ts->y_num + i] = (int16_t)(buf[1 + i * 2] + 256 * buf[1 + i * 2 + 1]);
	}
#endif

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);
}

/*******************************************************
Description:
    Novatek touchscreen get meta data function.

return:
    n.a.
*******************************************************/
void nvt_get_mdata(int32_t *buf, uint8_t *m_x_num, uint8_t *m_y_num)
{
    *m_x_num = ts->x_num;
    *m_y_num = ts->y_num;
    memcpy(buf, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
}

/*******************************************************
Description:
	Novatek touchscreen firmware version show function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_fw_version_show(struct seq_file *m, void *v)
{
	seq_printf(m, "fw_ver=%d, x_num=%d, y_num=%d, button_num=%d\n", ts->fw_ver, ts->x_num, ts->y_num, ts->max_button_num);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print show
	function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t c_show(struct seq_file *m, void *v)
{
	int32_t i = 0;
	int32_t j = 0;

	for (i = 0; i < ts->y_num; i++) {
		for (j = 0; j < ts->x_num; j++) {
			seq_printf(m, "%5d, ", xdata[i * ts->x_num + j]);
		}
		seq_puts(m, "\n");
	}

#if TOUCH_KEY_NUM > 0
	for (i = 0; i < TOUCH_KEY_NUM; i++) {
		seq_printf(m, "%5d, ", xdata[ts->x_num * ts->y_num + i]);
	}
	seq_puts(m, "\n");
#endif

	seq_printf(m, "\n\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print start
	function.

return:
	Executive outcomes. 1---call next function.
	NULL---not call next function and sequence loop
	stop.
*******************************************************/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print next
	function.

return:
	Executive outcomes. NULL---no next and call sequence
	stop function.
*******************************************************/
static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

/*******************************************************
Description:
	Novatek touchscreen xdata sequence print stop
	function.

return:
	n.a.
*******************************************************/
static void c_stop(struct seq_file *m, void *v)
{
	return;
}

const struct seq_operations nvt_fw_version_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_fw_version_show
};

const struct seq_operations nvt_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_show
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_fw_version open
	function.

return:
	n.a.
*******************************************************/
static int32_t nvt_fw_version_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_fw_version_seq_ops);
}

static const struct file_operations nvt_fw_version_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fw_version_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_baseline open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_baseline_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_read_mdata(ts->mmap->BASELINE_ADDR, ts->mmap->BASELINE_BTN_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_baseline_fops = {
	.owner = THIS_MODULE,
	.open = nvt_baseline_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_raw open function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_raw_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_raw_fops = {
	.owner = THIS_MODULE,
	.open = nvt_raw_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_diff open function.

return:
	Executive outcomes. 0---succeed. negative---failed.
*******************************************************/
static int32_t nvt_diff_open(struct inode *inode, struct file *file)
{
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

	NVT_LOG("++\n");

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		return -EAGAIN;
	}

	if (nvt_get_fw_pipe() == 0)
		nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
	else
		nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);

	nvt_change_mode(NORMAL_MODE);

	mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return seq_open(file, &nvt_seq_ops);
}

static const struct file_operations nvt_diff_fops = {
	.owner = THIS_MODULE,
	.open = nvt_diff_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

#if(_CustomerFunction_)//[20180525,jx]


#define	EVENTBUFFER_STATUS_OFF 		(0x51)
#define	EVENTBUFFER_STATUS_DC		(0x52)
#define	EVENTBUFFER_STATUS_AC		(0x53)

#define FM_mode_Enter				(0x75)//[20180308,jx]Add FM Nodify
#define FM_mode_Leave				(0x76)

#define	IDLE_Enable 				(0x79)//New
#define	IDLE_Disable 				(0x7A)

#define	CMD_EDGE_REJECT_VERTICAL	(0x7C)
#define	CMD_EDGE_REJECT_LEFT_Up  	(0x7D)
#define	CMD_EDGE_REJECT_RIGHT_Up 	(0x7E)

#define	CMD_EXT7F					(0x7F)
#define	EXTCMD_ID_SMARTWAKE_SWITCH	(0x01)//[20200325,jx] SMARTWAKE New format
#define	EXTCMD_ID_GAMEMODE_SWITCH	(0x02)//[20200507,jx] GameMode

#define	USB_PLUGOUT				(0)
#define	USB_PLUGIN				(1)

#define	IDLE_SET_Disable 		(0)
#define	IDLE_SET_Enable 		(1)

#define	EDGE_REJECT_VERTICAL 	(1)
#define	EDGE_REJECT_LEFT_Up 	(2)
#define	EDGE_REJECT_RIGHT_Up	(0)

#define TOUCHSCREEN_NORMAL		(0x0A)
#define TOUCHSCREEN_GESTURE		(0x0B)
#define TOUCHSCREEN_SLEEP		(0x0C)



int8_t nvt_customizeCmd(uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	NVT_LOG("++ Cmd=0x%02X\n",u8Cmd);

  //mutex_lock(&ts->lock); 	//[20190724,jx]Old Rectify!
  
	#if (NVTFLASH_WORK_PROTECT)		
		if(u8_NVTflashWorking){
			NVT_ERR("EXECUTE FAIL!!! u8_NVTflashWorking=[%d]\n",u8_NVTflashWorking);
			NVT_LOG("--\n");
			ret = -1;
			return ret;
		}
	#endif
	mutex_lock(&ts->lock);	//[20200501,jx]New Rectify!	

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	for (retry = 0; retry < 20; retry++) {
		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_SPI_WRITE(ts->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}

	mutex_unlock(&ts->lock);//[20190724,jx]Rectify!
	NVT_LOG("--\n");	
	return ret;
}


int8_t nvt_customizeExtCmd(uint8_t u8Cmd, uint8_t u8ExtCmdID, uint8_t u8Param1, uint8_t u8Param2)
{
	uint8_t buf[8] = {0};
	uint8_t retry = 0;
	int8_t ret = 0;

	NVT_LOG("++ Cmd=0x%02X,ExtCmdID 0x%02X,0x%02X,0x%02X\n",u8Cmd, u8ExtCmdID, u8Param1, u8Param2);

  //mutex_lock(&ts->lock); 	//[20190724,jx]Old Rectify!
  
	#if (NVTFLASH_WORK_PROTECT)		
		if(u8_NVTflashWorking){
			NVT_ERR("EXECUTE FAIL!!! u8_NVTflashWorking=[%d]\n",u8_NVTflashWorking);
			NVT_LOG("--\n");
			ret = -1;
			return ret;
		}
	#endif
	mutex_lock(&ts->lock);	//[20200501,jx]New Rectify!	

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	
	for (retry = 0; retry < 20; retry++) {
		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8ExtCmdID;
		buf[3] = u8Param1;		
		buf[4] = u8Param2;		
		CTP_SPI_WRITE(ts->client, buf, 5);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry == 20)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}

	mutex_unlock(&ts->lock);//[20190724,jx]Rectify!
	NVT_LOG("--\n");	
	return ret;
}


//-----------------------------------------------------------------------------
//[20190108,jx]For VIVO NT36525_HDL( Host Down Load) 
//	Must replace single calling nvt_customizeCmd_WaitSet(,,0x13) in add follwing:
//	if (ts->wkg_info) {
//      nvt_update_gesture_firmware(BOOT_UPDATE_FIRMWARE_NAME);
//	} else {
//  	//---write spi command to enter "wakeup gesture mode"---
//      nvt_customizeCmd_WaitSet(EVENT_MAP_RESET_COMPLETE,RESET_STATE_INIT,0x13);
// 	}
//-----------------------------------------------------------------------------
int8_t nvt_customizeCmd_WaitSet(uint8_t u8WaitAddr, RST_COMPLETE_STATE check_reset_state, uint8_t u8Cmd)
{
	uint8_t buf[8] = {0};
	int8_t  ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;
	NVT_LOG("++\n");
	
	#if (0)//[20200610]Ignore APK/CmdTool Working flag in  B/C mode(Gesture/Sleep),//#if (NVTFLASH_WORK_PROTECT) 	
		if(u8_NVTflashWorking){
			NVT_ERR("EXECUTE FAIL!!! u8_NVTflashWorking=[%d]\n",u8_NVTflashWorking);
			NVT_LOG("--\n");
			ret = -1;
			return ret;
		}
	#endif
	mutex_lock(&ts->lock);	//[20200501,jx]New Rectify!	
	
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("HANDSHAKING failed, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}


	for (retry = 0; retry < retry_max; retry++) {

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		CTP_SPI_WRITE(ts->client, buf, 2);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);//[20180621]Correct

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry >= retry_max)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	
	mutex_unlock(&ts->lock);//[20190724,jx]Rectify!
	NVT_LOG("--\n");
	return ret;
}



int8_t nvt_customizeExtCmd_WaitSet(uint8_t u8WaitAddr, RST_COMPLETE_STATE check_reset_state, uint8_t u8Cmd
	, uint8_t u8ExtCmdID, uint8_t u8Param1, uint8_t u8Param2)
{
	uint8_t buf[8] = {0};
	int8_t  ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;
	
	NVT_LOG("++ Addr%02X,Wait%02X,Cmd%02X,Ext%02X,%02X,%02X\n"
		,u8WaitAddr, check_reset_state, u8Cmd, u8ExtCmdID, u8Param1, u8Param2);
	
	#if (0)//[20200610]Ignore APK/CmdTool Working flag in  B/C mode(Gesture/Sleep),//#if (NVTFLASH_WORK_PROTECT) 		
		if(u8_NVTflashWorking){
			NVT_ERR("EXECUTE FAIL!!! u8_NVTflashWorking=[%d]\n",u8_NVTflashWorking);
			NVT_LOG("--\n");
			ret = -1;
			return ret;
		}
	#endif
	mutex_lock(&ts->lock);	//[20200501,jx]New Rectify!	
	
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = u8WaitAddr;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if(unlikely(retry > retry_max)) {
			NVT_ERR("HANDSHAKING failed, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}


	for (retry = 0; retry < retry_max; retry++) {

		//---switch HOST_CMD---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = u8Cmd;
		buf[2] = u8ExtCmdID;
		buf[3] = u8Param1;		
		buf[4] = u8Param2;		
		CTP_SPI_WRITE(ts->client, buf, 5);

		msleep(35);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);//[20180621]Correct

		if (buf[1] == 0x00)
			break;
	}

	if (unlikely(retry >= retry_max)) {
		NVT_ERR("customizeCmd 0x%02X failed, buf[1]=0x%02X\n", u8Cmd, buf[1]);
		ret = -1;
	}
	
	mutex_unlock(&ts->lock);//[20190724,jx]Rectify!
	NVT_LOG("--\n");
	return ret;
}



int tpd_usb_plugin(int plugin)
{
	int ret = -1;
	//mutex_lock(&ts->lock);//[20190724,jx]Move to nvt_customizeCmd()

	switch(plugin) {
        case USB_PLUGOUT:
            NVT_LOG("usb plug [out] .\n");
            ret = nvt_customizeCmd(EVENTBUFFER_STATUS_OFF);
            if (ret < 0)
            {
                NVT_LOG("tpd_usb_plugin 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_OFF);
            }
            break;

        case USB_PLUGIN:
		default:			//default is AC
            NVT_LOG("usb plug [in ] .\n");
            ret = nvt_customizeCmd(EVENTBUFFER_STATUS_AC);
            if (ret < 0)
            {
                NVT_LOG("tpd_usb_plugin 0x%02X cmd failed.\n", EVENTBUFFER_STATUS_AC);
            }
            break;
    }
	//mutex_unlock(&ts->lock);
	return ret;
}

int bbk_xxx_set_charger_bit(int state)
{
	int32_t i32ret = 0;
	i32ret = tpd_usb_plugin(state);
	NVT_LOG("i32ret=[%d]\n",i32ret);///jx
	return	i32ret;
}

int bbk_xxx_read_charger_bit(void)
{
	uint8_t buf[8] = {0};
	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);

	buf[0] = 0x5C;//[20180525]5C_bit2 is new; 62 is Old!
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	NVT_LOG("charger_bit=[%d],bit=[%X]\n",buf[1],buf[1] & 0x04);

	if (buf[1] & 0x04)	//[20180525]5C_bit2
		return (1);
	else
		return (-1);
}

int idleEnableOrDisable(int state)
{
	int32_t i32ret = 0;
	if(state == IDLE_SET_Disable)
		i32ret = nvt_customizeCmd(IDLE_Disable);
	else if(state == IDLE_SET_Enable)
		i32ret = nvt_customizeCmd(IDLE_Enable);
	else
		i32ret = (-1);

	NVT_LOG("para=[%d],i32ret=[%d]\n",state,i32ret);///jx
	return	i32ret;
}

//[20180110]
int setEdgeRestainSwitch(int i32Switch)
{
	int32_t i32ret = 0;
	if(i32Switch == EDGE_REJECT_VERTICAL)
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_VERTICAL);
	else if(i32Switch == EDGE_REJECT_LEFT_Up)
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_LEFT_Up);
	else if(i32Switch == EDGE_REJECT_RIGHT_Up)
		i32ret = nvt_customizeCmd(CMD_EDGE_REJECT_RIGHT_Up);
	else{
		i32ret = (-1);
	}

	NVT_LOG("para=[%d],i32ret=[%d]\n",i32Switch,i32ret);///jx
	return	i32ret;
}

//[20180110]Creat
//[20200103]Modify from 5C~5F t0 5A~5F
uint8_t getFWcmdStatus(void)
{
	uint8_t buf[8] = {0};
	uint8_t u8ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HOST_CMD);
	//[20200103]Modify from 5C~5F t0 5A~5F
	buf[0] = 0x5A;
	buf[1] = 0xFF;//[5A]
	buf[2] = 0xFF;//[5B]
	buf[3] = 0xFF;//[5C]
	buf[4] = 0xFF;//[5D]
	buf[5] = 0xFF;//[5E]
	buf[6] = 0xFF;//[5F]
	CTP_SPI_READ(ts->client, buf, 7);	
	NVT_LOG("buf[5A~5F]=[0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X]\n" 
		,buf[1] ,buf[2] ,buf[3] ,buf[4], buf[5] ,buf[6]);


	u8ret=buf[1];
	return u8ret;
}



int FM_notify(int OnOff)
{
	int32_t i32ret = 0;
	NVT_LOG("++\n");

	if(OnOff == 1) {
		nvt_customizeCmd(FM_mode_Enter);
		NVT_LOG("On\n");
	}else if(OnOff == 0) {
		nvt_customizeCmd(FM_mode_Leave);
		NVT_LOG("Off\n");
	}else{
		NVT_LOG("Parameter Error:[%d]\n",OnOff);
		i32ret = (-1);
	}
	NVT_LOG("--\n");
	return i32ret;
}


//[20200325]NewVersion,Add for wei.zhou.rj@vivo.com
//example:
//	smartWake(EXTCMD_ID_SMARTWAKE_SWITCH,0xFF,0x0F);
//	getFWcmdStatus();
int smartWake(uint8_t u8ExtCmdID, uint8_t u8Switch01, uint8_t u8Switch02)
{
	int ret_i = 0;
  //ret_i = nvt_customizeExtCmd(CMD_EXT7F, u8ExtCmdID, u8Switch01, u8Switch02);

  	ret_i = nvt_customizeExtCmd_WaitSet(EVENT_MAP_RESET_COMPLETE, RESET_STATE_INIT, CMD_EXT7F 
		, u8ExtCmdID, u8Switch01, u8Switch02);
	return ret_i;
}


int game_mode_ctl(uint8_t u8gameMode)
{
	int ret_i = 0;
  	ret_i = nvt_customizeExtCmd(CMD_EXT7F, EXTCMD_ID_GAMEMODE_SWITCH, u8gameMode, 0x00);
	return ret_i;
}


#endif//(_CustomerFunction_)


/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update
	function.
return:
	n.a.
Example:
	adb shell "echo 0 > /proc/nvt_update"
	adb shell "echo 1 > /proc/nvt_update"
History:
	[20180903]Correct to use copy_from_user()
	[20180904]Parsing to use kstrtouint(),string to uint.
*******************************************************/
/*
static ssize_t nvt_fwupdate_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *f_pos)
{
	uint8_t fwtype = FWTYPE_Normal;
	uint8_t *u8str = NULL;
	uint32_t *u32str = NULL;
	size_t stLenth=1;
	int32_t ret = 0;

	NVT_LOG("++\n");//[20180902,jx]//[20180902,jx]move &debug
	if (mutex_lock_interruptible(&ts->lock)) {
		return -ERESTARTSYS;
	}

#if NVT_TOUCH_ESD_PROTECT
	nvt_esd_check_enable(false);
#endif // #if NVT_TOUCH_ESD_PROTECT //

	// allocate buffer //
	u8str = (uint8_t *)kzalloc((stLenth), GFP_KERNEL);
	if(u8str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		return -ENOMEM;
	}
	// copy_from_user //
	if (copy_from_user(u8str, buf, stLenth)) {
		NVT_ERR("copy from user error\n");
		return -EFAULT;
	}

	// allocate buffer //
	u32str = (uint32_t *)kzalloc((stLenth), GFP_KERNEL);
	if(u8str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		return -ENOMEM;
	}
	// kstrtouint,String to Uint //
	ret = kstrtouint(u8str,10,u32str);
	if(ret != 0) {
		NVT_ERR("kstrtouint failed! (%d)\n",ret);
		return -1;
	}

	fwtype = (uint8_t)u32str[0];
	NVT_LOG("fwtype is %d\n", fwtype);

	switch (fwtype) {
		case FWTYPE_Normal:
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			break;
		case FWTYPE_MP:
			nvt_update_firmware(MP_UPDATE_FIRMWARE_NAME);
			break;
		default:
			NVT_ERR("fwtype error\n");
	}

	NVT_LOG("--\n");
	mutex_unlock(&ts->lock);

	return count;
}

static const struct file_operations nvt_fwupdate_fops = {
	.owner = THIS_MODULE,
	.write = nvt_fwupdate_write,
};
*/

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update open function.
return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
static int32_t nvt_fwupdate_open(struct inode *inode, struct file *file)
{
	struct nvt_fwupdate_data *dev;

	NVT_LOG("++\n");
	dev = kmalloc(sizeof(struct nvt_fwupdate_data), GFP_KERNEL);
	if (dev == NULL) {
		NVT_ERR("Failed to allocate memory for nvt flash data\n");
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	NVT_LOG("--\n");
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update close function.
return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_fwupdate_close(struct inode *inode, struct file *file)
{
	struct nvt_fwupdate_data *dev = file->private_data;

	NVT_LOG("++\n");
	if (dev)
		kfree(dev);

	NVT_LOG("--\n");
	return 0;
}
/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_update read function.
return:
	Executive outcomes. 0---succeed.
History:
	[20180903,jx]Correct to use copy_from_user()
	[20180904,jx]Parsing to use kstrtouint(),string to uint.
	[20180904,jx]Change to read node (from write node) for SELinux
*******************************************************/
#define FWTYPE_MP     (0)
#define FWTYPE_Normal (1)
static ssize_t nvt_fwupdate_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	uint8_t *str = NULL;
	uint8_t fwtype = FWTYPE_Normal;
	int32_t ret = 0;

	NVT_LOG("++\n");

	if (count > NVT_TRANSFER_LEN) {
		NVT_ERR("invalid transfer len!\n");
		return -EFAULT;
	}

	/* allocate buffer*/
	str = (uint8_t *)kzalloc((count), GFP_KERNEL);
	if(str == NULL) {
		NVT_ERR("kzalloc for buf failed!\n");
		ret = -ENOMEM;
		goto kzalloc_failed;
	}
	if (copy_from_user(str, buff, count)) {
		NVT_ERR("copy from user error\n");
		ret = -EFAULT;
		goto out;
	}

#if NVT_TOUCH_ESD_PROTECT
	/*
	 * stop esd check work to avoid case that 0x77 report righ after here to enable esd check again
	 * finally lead to trigger esd recovery bootloader reset
	 */
	cancel_delayed_work_sync(&nvt_esd_check_work);
	nvt_esd_check_enable(false);
#endif /* #if NVT_TOUCH_ESD_PROTECT */

  //fwtype=buff[0];  //[20190907,jx]"Internal error: Accessing user space memory outside uaccess.h routines"
	fwtype = str[0]; //[20190907,jx]Correct!

	NVT_LOG("fwtype is %d\n", fwtype);


	switch (fwtype) {
		case FWTYPE_Normal:
			nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
			break;
		case FWTYPE_MP:
			nvt_update_firmware(MP_UPDATE_FIRMWARE_NAME);
			break;
		default:
			NVT_ERR("fwtype error\n");
	}


	NVT_LOG("--\n");

out:
	kfree(str);
kzalloc_failed:
	return ret;
}

static const struct file_operations nvt_fwupdateV2_fops = {
	.owner = THIS_MODULE,
	.open = nvt_fwupdate_open,
	.read = nvt_fwupdate_read,
	.release = nvt_fwupdate_close,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_trimid
	function.

return:
	n.a.
*******************************************************/
static int32_t c_trimid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%02X%02X%02X%02X%02X%02X\n",
			ts->trimid[0], ts->trimid[1], ts->trimid[2],
			ts->trimid[3], ts->trimid[4], ts->trimid[5]);

	return 0;
}

const struct seq_operations nvt_trimid_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_trimid_show
};

static int32_t nvt_trimid_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &nvt_trimid_seq_ops);
}

static const struct file_operations nvt_trimid_fops = {
	.owner = THIS_MODULE,
	.open = nvt_trimid_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/*******************************************************
Description:
	Novatek touchscreen /proc/nvt_CmdTest
	function.

return:
	n.a.
*******************************************************/
static int32_t c_cmdtest_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s,jxTest\n",__func__);

	return 0;
}

const struct seq_operations nvt_cmdtest_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_cmdtest_show
};

//[20180912,jx]VIVO new spec.
int bbk_xxx_get_rawordiff_data(int which, int *data)
{
	int32_t i32ret = 0;

//	if (mutex_lock_interruptible(&ts->lock)) {
//		//return -ERESTARTSYS;
//	}
		NVT_LOG("++\n");

	if (nvt_clear_fw_status()) {
		mutex_unlock(&ts->lock);
		//return -EAGAIN;
			NVT_ERR("EINVAL!\n");
		return -EINVAL;
	}

	nvt_change_mode(TEST_MODE_2);

	if (nvt_check_fw_status()) {
		mutex_unlock(&ts->lock);
		//return -EAGAIN;
			NVT_ERR("EINVAL!\n");
		return -EINVAL;
	}


	if (nvt_get_fw_info()) {
		mutex_unlock(&ts->lock);
		//return -EAGAIN;
			NVT_ERR("EINVAL!\n");
		return -EINVAL;
	}

	if(which==0)//0,raw
	{
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}else if(which==1){//1,diff
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->DIFF_PIPE0_ADDR, ts->mmap->DIFF_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->DIFF_PIPE1_ADDR, ts->mmap->DIFF_BTN_PIPE1_ADDR);
	}else if(which==2){//2,base//[20180912,jx]VIVO new spec.
		if (nvt_get_fw_pipe() == 0)
			nvt_read_mdata(ts->mmap->RAW_PIPE0_ADDR, ts->mmap->RAW_BTN_PIPE0_ADDR);
		else
			nvt_read_mdata(ts->mmap->RAW_PIPE1_ADDR, ts->mmap->RAW_BTN_PIPE1_ADDR);
	}else{
		NVT_ERR("EINVAL!\n");
		i32ret = (-EINVAL);
	}

	NVT_LOG("(x,y)=(%d,%d),which=[%d]\n",ts->x_num ,ts->y_num,which);	///jx

	if(i32ret!=(-EINVAL)){
		memcpy(data, xdata, ((ts->x_num * ts->y_num + TOUCH_KEY_NUM) * sizeof(int32_t)));
	}

	nvt_change_mode(NORMAL_MODE);

	//mutex_unlock(&ts->lock);

	NVT_LOG("--\n");

	return i32ret;
}
int32_t xdata_custom[2048] = {0};
static int32_t nvt_CmdTest_open(struct inode *inode, struct file *file)
{
//TestAction

	game_mode_ctl(1);
	getFWcmdStatus();
	
		//smartWake(EXTCMD_ID_SMARTWAKE_SWITCH, 0xFF, 0x0F); //New			
		//getFWcmdStatus();
		
	//bbk_xxx_get_rawordiff_data(0,xdata_custom);
	//bbk_xxx_get_rawordiff_data(2,xdata_custom);
/*
	bbk_xxx_get_rawordiff_data(1,xdata_custom);
	{
		int32_t y,x;
		printk("-----Normal(Vertical) arrange.-----------\n");
		for (y = 0; y < ts->y_num; y++) {
			for (x = 0; x < ts->x_num; x++) {
				printk("%5d, ", xdata_custom[y * ts->x_num + x]);
			}
			printk("\n");
		}
		printk("-----Horizontal arrange.-----------\n");
		for (x = 0; x < ts->x_num; x++) {
			for (y = 0; y < ts->y_num; y++) {
				printk("%5d, ", xdata_custom[y * ts->x_num + x]);
			}
			printk("\n");
		}
	}
*/
/*
	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME);
	nvt_customizeCmd_WaitSet(EVENT_MAP_RESET_COMPLETE,RESET_STATE_INIT,0x13);
*/
	//bbk_xxx_set_charger_bit(USB_PLUGOUT);
/*
		msleep(200);
		bbk_xxx_read_charger_bit();
		getFWcmdStatus();

	bbk_xxx_set_charger_bit(USB_PLUGIN);
		msleep(200);
		bbk_xxx_read_charger_bit();
		getFWcmdStatus();

	idleEnableOrDisable(2);//Err Parameter
		msleep(200);
		getFWcmdStatus();
	idleEnableOrDisable(IDLE_SET_Enable);
		msleep(200);
		getFWcmdStatus();
	idleEnableOrDisable(IDLE_SET_Disable);
		msleep(200);
		getFWcmdStatus();

	setEdgeRestainSwitch(3);//Err Parameter
		msleep(1000);
		getFWcmdStatus();
	setEdgeRestainSwitch(EDGE_REJECT_VERTICAL);
		msleep(1000);
		getFWcmdStatus();
	setEdgeRestainSwitch(EDGE_REJECT_LEFT_Up);
		msleep(1000);
		getFWcmdStatus();
	setEdgeRestainSwitch(EDGE_REJECT_RIGHT_Up);
		msleep(1000);
		getFWcmdStatus();

	FM_notify(3);//Err Parameter
		msleep(1000);
		getFWcmdStatus();
	FM_notify(1);
		msleep(1000);
		getFWcmdStatus();
	FM_notify(0);
		msleep(1000);
		getFWcmdStatus();
	*/

	//TestLog
	/*
	[ 1290.842977] [NVT-ts] tpd_usb_plugin 709: usb plug [out] .
	[ 1290.847356] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x51
	[ 1290.902305] [NVT-ts] nvt_customizeCmd 635: --
	[ 1290.905660] [NVT-ts] bbk_xxx_set_charger_bit 735: i32ret=[0]
	[ 1291.125669] [NVT-ts] bbk_xxx_read_charger_bit 749: charger_bit=[49]
	[ 1291.134719] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x31,0x00,0x01,0x00]
	[ 1291.140763] [NVT-ts] tpd_usb_plugin 719: usb plug [in ] .
	[ 1291.146044] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x53
	[ 1291.203428] [NVT-ts] nvt_customizeCmd 635: --
	[ 1291.206789] [NVT-ts] bbk_xxx_set_charger_bit 735: i32ret=[0]
	[ 1291.425899] [NVT-ts] bbk_xxx_read_charger_bit 749: charger_bit=[53]
	[ 1291.434781] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]

	[ 1291.442899] [NVT-ts] idleEnableOrDisable 767: para=[2],i32ret=[-1]
	[ 1291.655676] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.661756] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x79
	[ 1291.723288] [NVT-ts] nvt_customizeCmd 635: --
	[ 1291.726650] [NVT-ts] idleEnableOrDisable 767: para=[1],i32ret=[0]
	[ 1291.944774] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x35,0x00,0x01,0x00]
	[ 1291.950836] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7A
	[ 1292.003344] [NVT-ts] nvt_customizeCmd 635: --
	[ 1292.006706] [NVT-ts] idleEnableOrDisable 767: para=[0],i32ret=[0]
	[ 1292.226022] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1292.232083] [NVT-ts] setEdgeRestainSwitch 785: para=[3],i32ret=[-1]
	[ 1293.242293] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]

	[ 1293.248255] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7C
	[ 1293.301057] [NVT-ts] nvt_customizeCmd 635: --
	[ 1293.304394] [NVT-ts] setEdgeRestainSwitch 785: para=[1],i32ret=[0]
	[ 1294.322212] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x25,0x00,0x01,0x00]
	[ 1294.328170] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7D
	[ 1294.381043] [NVT-ts] nvt_customizeCmd 635: --
	[ 1294.384381] [NVT-ts] setEdgeRestainSwitch 785: para=[2],i32ret=[0]
	[ 1295.402157] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x45,0x00,0x01,0x00]
	[ 1295.408113] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x7E
	[ 1295.461033] [NVT-ts] nvt_customizeCmd 635: --
	[ 1295.464374] [NVT-ts] setEdgeRestainSwitch 785: para=[0],i32ret=[0]
	[ 1296.482340] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]

	[ 1296.488298] [NVT-ts] FM_notify 817: ++
	[ 1296.492118] [NVT-ts] FM_notify 826: Parameter Error:[3]
	[ 1296.497238] [NVT-ts] FM_notify 829: --
	[ 1297.512574] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]
	[ 1297.518535] [NVT-ts] FM_notify 817: ++
	[ 1297.522939] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x75
	[ 1297.571110] [NVT-ts] nvt_customizeCmd 635: --
	[ 1297.574447] [NVT-ts] FM_notify 821: On
	[ 1297.578177] [NVT-ts] FM_notify 829: --
	[ 1298.594103] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x03,0x00]
	[ 1298.600198] [NVT-ts] FM_notify 817: ++
	[ 1298.603804] [NVT-ts] nvt_customizeCmd 611: ++ Cmd=0x76
	[ 1298.662285] [NVT-ts] nvt_customizeCmd 635: --
	[ 1298.665628] [NVT-ts] FM_notify 824: Off
	[ 1298.669451] [NVT-ts] FM_notify 829: --
	[ 1299.684187] [NVT-ts] getFWcmdStatus 806: buf[1,2,3,4]=[0x65,0x00,0x01,0x00]

	*/


	return seq_open(file, &nvt_cmdtest_seq_ops);
}

static const struct file_operations nvt_CmdTest_fops = {
	.owner = THIS_MODULE,
	.open = nvt_CmdTest_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};


/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	initial function.

return:
	Executive outcomes. 0---succeed. -12---failed.
*******************************************************/
int32_t nvt_extra_proc_init(void)
{
	NVT_proc_fw_version_entry = proc_create(NVT_FW_VERSION, 0444, NULL,&nvt_fw_version_fops);
	if (NVT_proc_fw_version_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_FW_VERSION);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_FW_VERSION);
	}

	NVT_proc_baseline_entry = proc_create(NVT_BASELINE, 0444, NULL,&nvt_baseline_fops);
	if (NVT_proc_baseline_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_BASELINE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_BASELINE);
	}

	NVT_proc_raw_entry = proc_create(NVT_RAW, 0444, NULL,&nvt_raw_fops);
	if (NVT_proc_raw_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_RAW);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_RAW);
	}

	NVT_proc_diff_entry = proc_create(NVT_DIFF, 0444, NULL,&nvt_diff_fops);
	if (NVT_proc_diff_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n", NVT_DIFF);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n", NVT_DIFF);
	}



	//[20180904,jx]Change to read node (from write node) for SELinux
	NVT_proc_fwupdate_entry = proc_create(NVT_UPDATE, 0444, NULL,&nvt_fwupdateV2_fops);
	//NVT_proc_fwupdate_entry = proc_create(NVT_UPDATE, 0222, NULL,&nvt_fwupdate_fops);//no use!

	if (NVT_proc_fwupdate_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n",NVT_UPDATE);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n",NVT_UPDATE);
	}

	NVT_proc_trimid_entry = proc_create(NVT_TRIMID, 0444, NULL,&nvt_trimid_fops);
	if (NVT_proc_trimid_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n",NVT_TRIMID);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n",NVT_TRIMID);
	}
	//Novatek cmd test node
	NVT_proc_cmdTest_entry = proc_create(NVT_CMDTEST, 0444, NULL,&nvt_CmdTest_fops);
	if (NVT_proc_cmdTest_entry == NULL) {
		NVT_ERR("create proc/%s Failed!\n",NVT_CMDTEST);
		return -ENOMEM;
	} else {
		NVT_LOG("create proc/%s Succeeded!\n",NVT_CMDTEST);
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen extra function proc. file node
	deinitial function.

return:
	n.a.
History:
	[20190815,jx]Add	
*******************************************************/
void nvt_extra_proc_deinit(void)
{
	if (NVT_proc_fw_version_entry != NULL) {
		remove_proc_entry(NVT_FW_VERSION, NULL);
		NVT_proc_fw_version_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_FW_VERSION);
	}

	if (NVT_proc_baseline_entry != NULL) {
		remove_proc_entry(NVT_BASELINE, NULL);
		NVT_proc_baseline_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_BASELINE);
	}

	if (NVT_proc_raw_entry != NULL) {
		remove_proc_entry(NVT_RAW, NULL);
		NVT_proc_raw_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_RAW);
	}

	if (NVT_proc_diff_entry != NULL) {
		remove_proc_entry(NVT_DIFF, NULL);
		NVT_proc_diff_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_DIFF);
	}
	
	if (NVT_proc_fwupdate_entry != NULL) {
		remove_proc_entry(NVT_UPDATE, NULL);
		NVT_proc_fwupdate_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_UPDATE);
	}
	
	if (NVT_proc_trimid_entry != NULL) {
		remove_proc_entry(NVT_TRIMID, NULL);
		NVT_proc_trimid_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_TRIMID);
	}
	
	if (NVT_proc_cmdTest_entry != NULL) {
		remove_proc_entry(NVT_CMDTEST, NULL);
		NVT_proc_cmdTest_entry = NULL;
		NVT_LOG("Removed /proc/%s\n", NVT_CMDTEST);
	}			
		
}
#endif
