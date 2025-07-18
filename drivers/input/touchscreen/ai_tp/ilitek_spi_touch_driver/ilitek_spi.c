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

#include "ilitek.h"


struct touch_bus_info {
	struct spi_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_tddi_dev *idev;
static u8 spi_ice_buf[1*K];

#if SPI_DMA_TRANSFER_SPLIT
#define DMA_TRANSFER_MAX_CHUNK		64   // number of chunks to be transferred.
#define DMA_TRANSFER_MAX_LEN		1024 // length of a chunk.
struct spi_transfer	xfer[DMA_TRANSFER_MAX_CHUNK];

int ilitek_spi_write_then_read_split(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	int duplex_len = 0;
	u8 cmd = 0x0;
	struct spi_message	message;

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));
	memset(idev->spi_tx, 0x0, SPI_TX_BUF_SIZE);
	memset(idev->spi_rx, 0x0, SPI_RX_BUF_SIZE);

	if ((n_tx > SPI_TX_BUF_SIZE) || (n_rx > SPI_RX_BUF_SIZE)) {
		ipio_err("Tx/Rx length is over than dma buf, abort\n");
		status = -ENOMEM;
		goto out;
	}

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		if (n_tx % DMA_TRANSFER_MAX_LEN)
			xferloop = (n_tx / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = n_tx / DMA_TRANSFER_MAX_LEN;

		xferlen = n_tx;
		memcpy(idev->spi_tx, (u8 *)txbuf, xferlen);

		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = idev->spi_tx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		if (n_tx > DMA_TRANSFER_MAX_LEN) {
			ipio_err("Tx length must be lower than transfer length (%d).\n", DMA_TRANSFER_MAX_LEN);
			status = -EINVAL;
			break;
		}

		memcpy(idev->spi_tx, txbuf, n_tx);

		duplex_len = n_tx + n_rx;

		if (duplex_len % DMA_TRANSFER_MAX_LEN)
			xferloop = (duplex_len / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = duplex_len / DMA_TRANSFER_MAX_LEN;

		xferlen = duplex_len;
		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = idev->spi_tx;
			xfer[xfercnt].rx_buf = idev->spi_rx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = duplex_len - xfercnt * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		if (status == 0)
			memcpy((u8 *)rxbuf, &idev->spi_rx[1], n_rx);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

out:
	if (status != 0)
		ipio_err("spi transfer failed\n");

	return status;
}
#else
int ilitek_spi_write_then_read_direct(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1;
	int duplex_len = 0;
	u8 cmd;
	struct spi_message	message;
	struct spi_transfer	xfer;

	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		xfer.len = n_tx;
		xfer.tx_buf = txbuf;
		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		duplex_len = n_tx + n_rx;
		if ((duplex_len > SPI_TX_BUF_SIZE) ||
			(duplex_len > SPI_RX_BUF_SIZE)) {
			ipio_err("duplex_len is over than dma buf, abort\n");
			status = -ENOMEM;
			break;
		}

		memset(idev->spi_tx, 0x0, SPI_TX_BUF_SIZE);
		memset(idev->spi_rx, 0x0, SPI_RX_BUF_SIZE);

		xfer.len = duplex_len;
		memcpy(idev->spi_tx, txbuf, n_tx);
		xfer.tx_buf = idev->spi_tx;
		xfer.rx_buf = idev->spi_rx;

		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		if (status != 0)
			break;

		memcpy((u8 *)rxbuf, &idev->spi_rx[1], n_rx);
		break;
	default:
		ipio_info("Unknown command 0x%x\n", cmd);
		break;
	}

	if (status != 0)
		ipio_err("spi transfer failed\n");

	return status;
}
#endif

static int core_rx_lock_check(int *ret_size)
{
	int i, count = 1;
	u8 txbuf[5] = {0};
	u8 rxbuf[4] = {0};
	u16 status = 0, lock = 0x5AA5;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		txbuf[1] = 0x25;
		txbuf[2] = 0x94;
		txbuf[3] = 0x0;
		txbuf[4] = 0x2;
		if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x94,0x0,0x2) error\n");
			goto out;
		}

		memset(txbuf, 0, sizeof(txbuf));
		memset(rxbuf, 0, sizeof(rxbuf));
		txbuf[0] = SPI_READ;
		if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];
		*ret_size = (rxbuf[0] << 8) + rxbuf[1];
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify start	
		//ipio_debug("Rx lock = 0x%x, size = %d\n", status, *ret_size);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify end	
		if (status == lock)
			return 0;

		mdelay(1);
	}

out:
	ipio_info("Rx buf is locked, 0x%x\n", status);
	return SPI_IS_LOCKED;
}

static int core_tx_unlock_check(void)
{
	int i, count = 100;
	u8 txbuf[5] = {0};
	u8 rxbuf[4] = {0};
	u16 status = 0, unlock = 0x9881;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		txbuf[1] = 0x25;
		txbuf[2] = 0x0;
		txbuf[3] = 0x0;
		txbuf[4] = 0x2;
		if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi write (0x25,0x0,0x0,0x2) error\n");
			goto out;
		}

		memset(txbuf, 0, sizeof(txbuf));
		memset(rxbuf, 0, sizeof(rxbuf));
		txbuf[0] = SPI_READ;
		if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi read error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];

		ipio_debug("Tx unlock = 0x%x\n", status);

		if (status == unlock)
			return 0;

		mdelay(1);
	}

out:
	ipio_err("Tx buf is locked, 0x%x\n", status);
	return -EIO;
}

static int core_spi_ice_mode_unlock_read(u8 *data, int size)
{
	int ret = 0;
	u8 txbuf[64] = { 0 };

	/* set read address */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x98;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	if (idev->spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_info("spi write (0x25,0x98,0x00,0x2) error\n");
		ret = -EIO;
		return ret;
	}

	/* read data */
	memset(txbuf, 0, sizeof(txbuf));
	txbuf[0] = SPI_READ;
	if (idev->spi_write_then_read(idev->spi, txbuf, 1, data, size) < 0) {
		ret = -EIO;
		return ret;
	}

	/* write data unlock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x94;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x98;
	txbuf[8] = (char)0x81;
	if (idev->spi_write_then_read(idev->spi, txbuf, 9, txbuf, 0) < 0) {
		ipio_err("spi write unlock (0x9881) error, ret = %d\n", ret);
		ret = -EIO;
	}
	return ret;
}

static int core_spi_ice_mode_lock_write(u8 *data, int size)
{
	int ret = 0;
	int safe_size = size;
	u8 check_sum = 0, wsize = 0;

	if (size > sizeof(spi_ice_buf)) {
		ipio_err("Size(%d) is greater than spi_ice_buf, abort\n", size);
		return -EINVAL;
	}

	memset(spi_ice_buf, 0x0, sizeof(spi_ice_buf));

	/* Write data */
	spi_ice_buf[0] = SPI_WRITE;
	spi_ice_buf[1] = 0x25;
	spi_ice_buf[2] = 0x4;
	spi_ice_buf[3] = 0x0;
	spi_ice_buf[4] = 0x2;

	/* Calcuate checsum and fill it in the last byte */
	check_sum = ilitek_calc_packet_checksum(data, size);
	ipio_memcpy(spi_ice_buf + 5, data, size, safe_size + 4);
	spi_ice_buf[5 + size] = check_sum;
	size++;
	wsize = size;
	if (wsize % 4 != 0)
		wsize += 4 - (wsize % 4);

	if (idev->spi_write_then_read(idev->spi, spi_ice_buf, wsize + 5, spi_ice_buf, 0) < 0) {
		ipio_info("spi write (0x25,0x4,0x00,0x2) error\n");
		ret = -EIO;
		goto out;
	}

	/* write data lock */
	spi_ice_buf[0] = SPI_WRITE;
	spi_ice_buf[1] = 0x25;
	spi_ice_buf[2] = 0x0;
	spi_ice_buf[3] = 0x0;
	spi_ice_buf[4] = 0x2;
	spi_ice_buf[5] = (size & 0xFF00) >> 8;
	spi_ice_buf[6] = size & 0xFF;
	spi_ice_buf[7] = (char)0x5A;
	spi_ice_buf[8] = (char)0xA5;
	if (idev->spi_write_then_read(idev->spi, spi_ice_buf, 9, spi_ice_buf, 0) < 0) {
		ipio_err("spi write lock (0x5AA5) error, ret = %d\n", ret);
		ret = -EIO;
	}

out:
	return ret;
}

static int ilitek_spi_check_ack(void)
{
	u8 ack = 0;
	u8 write = SPI_WRITE;

	if (idev->spi_write_then_read(idev->spi, &write, 1, &ack, 1) < 0) {
		ipio_err("spi write 0x82 error\n");
		return -EIO;
	}
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify start	
	//ipio_debug("spi ack = %x\n", ack);
//Antaiui <AI_BSP_CTP> <hehl> <2020-10-27> modify end		
	return ack;
}

static int core_spi_ice_mode_disable(void)
{
	int ack, retry = 3;
	u8 ex_ice[5] = {0x82, 0x1B, 0x62, 0x10, 0x18};

	while (retry > 0) {
		if (idev->spi_write_then_read(idev->spi, ex_ice, sizeof(ex_ice), NULL, 0) < 0) {
			ipio_err("spi write ice mode disable failed\n");
			retry--;
			continue;
		}

		if (!idev->fix_ice)
			break;

		ack = idev->spi_ack();
		if (ack == SPI_ACK) {
			break;
		} else if (ack == SPI_WRITE) {
			ipio_err("SPI ACK error (0x%x)\n", ack);
			return DO_SPI_RECOVER;
		} else {
			retry--;
		}
	}

	if (retry <= 0) {
		ipio_err("Failed to exit ice mode with spi comm\n");
		return -EIO;
	}

	return 0;
}

static int core_spi_ice_mode_enable(void)
{
	int retry = 3, ret = 0;
	u8 en_ice[5] = {0x82, 0x1F, 0x62, 0x10, 0x18};
	u8 pid_cmd[5] = {0};
	u32 pid = 0;

	while (retry > 0) {
		if (idev->spi_write_then_read(idev->spi, en_ice, sizeof(en_ice), NULL, 0) < 0) {
			ipio_err("write ice mode cmd error\n");
			retry--;
			continue;
		}

		if (!idev->fix_ice)
			break;

		pid_cmd[0] = SPI_WRITE;
		pid_cmd[1] = 0x25;
		pid_cmd[2] = ((idev->chip->pid_addr & 0x000000FF) >> 0);
		pid_cmd[3] = ((idev->chip->pid_addr & 0x0000FF00) >> 8);
		pid_cmd[4] = ((idev->chip->pid_addr & 0x00FF0000) >> 16);

		if (idev->spi_write_then_read(idev->spi, pid_cmd, sizeof(pid_cmd), NULL, 0) < 0) {
			ipio_err("write pid cmd error\n");
			retry--;
			continue;
		}

		pid_cmd[0] = SPI_READ;
		if (idev->spi_write_then_read(idev->spi, &pid_cmd[0], sizeof(u8), &pid, sizeof(pid)) < 0) {
			ipio_err("write pid cmd error\n");
			retry--;
			continue;
		}

		ipio_debug("check pid = 0x%x\n", pid);

		if (ilitek_tddi_ic_check_support(pid, pid >> 16) == 0)
			break;

		retry--;
	}

	if (retry <= 0) {
		ipio_err("Failed to enter ice mode with spi comm\n");
		ret = -EIO;
	}

	return ret;
}

static int core_spi_ice_mode_write(u8 *data, int len)
{
	int ack = 0, ret = 0;

	ack = idev->spi_ack();
	if (ack != SPI_ACK) {
		ipio_err("SPI ACK error (0x%x)\n", ack);
		return DO_SPI_RECOVER;
	}

	ret = core_spi_ice_mode_enable();
	if (ret < 0)
		return ret;

	/* send data and change lock status to 0x5AA5. */
	ret = core_spi_ice_mode_lock_write(data, len);
	if (ret < 0)
		goto out;

	/*
	 * Check FW if they already received the data we sent.
	 * They change lock status from 0x5AA5 to 0x9881 if they did.
	 */
	ret = core_tx_unlock_check();
	if (ret < 0)
		goto out;

out:
	if (core_spi_ice_mode_disable() < 0)
		ret = -EIO;

	return ret;
}

static int core_spi_ice_mode_read(u8 *data, int len)
{
	int size = 0, ret = 0, ack = 0;

	ack = idev->spi_ack();
	if (ack != SPI_ACK) {
		ipio_err("SPI ACK error (0x%x)\n", ack);
		return DO_SPI_RECOVER;
	}

	ret = core_spi_ice_mode_enable();
	if (ret < 0)
		return ret;

	/*
	 * Check FW if they already send their data to rxbuf.
	 * They change lock status from 0x9881 to 0x5AA5 if they did.
	 */
	ret = core_rx_lock_check(&size);
	if (ret < 0 || ret == SPI_IS_LOCKED)
		goto out;

	if (len < size && (!idev->fw_uart_en && !idev->gesture_demo_ctrl)) {
		ipio_info("WARRING! size(%d) > len(%d), use len to get data\n", size, len);
		size = len;
	}

	/* receive data from rxbuf and change lock status to 0x9881. */
	ret = core_spi_ice_mode_unlock_read(data, size);
	if (ret < 0)
		goto out;

out:
	if (core_spi_ice_mode_disable() < 0)
		ret = -EIO;

	if (ret == SPI_IS_LOCKED)
		return ret;

	return (ret >= 0) ? size : ret;
}

static int core_spi_write(u8 *data, int len)
{
	int ret = 0, retry = 5;
	int safe_size = len;
	#if 0	//fix for fae 
	u8 wakeup[10] = {0x82, 0x25, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};

	/* if system is suspended, wake up our spi pll clock before communication. */
	if (idev->tp_suspend && !idev->skip_wake) {
		ipio_info("wake up spi pll clk\n");
		if (idev->spi_write_then_read(idev->spi, wakeup, sizeof(wakeup), NULL, 0) < 0) {
			ipio_err("spi write wake up cmd failed\n");
			return -EIO;
		}
	}
	#endif

	if (atomic_read(&idev->ice_stat) == DISABLE) {
		do {
			ret = core_spi_ice_mode_write(data, len);
			if (ret >= 0)
				break;
		} while (--retry > 0);
		goto out;
	}

	if (len > sizeof(spi_ice_buf)) {
		ipio_err("Size(%d) is greater than spi_ice_buf, abort\n", len);
		return -EINVAL;
	}

	memset(spi_ice_buf, 0x0, sizeof(spi_ice_buf));

	spi_ice_buf[0] = SPI_WRITE;
	ipio_memcpy(spi_ice_buf+1, data, len, safe_size + 1);

	if (idev->spi_write_then_read(idev->spi, spi_ice_buf, len+1, spi_ice_buf, 0) < 0) {
		ipio_err("spi write data error in ice mode\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static int core_spi_read(u8 *rxbuf, int len)
{
	int ret = 0;
	u8 txbuf[1] = {0};

	if (atomic_read(&idev->ice_stat) == DISABLE) {
		ret = core_spi_ice_mode_read(rxbuf, len);
		goto out;
	}

	txbuf[0] = SPI_READ;
	if (idev->spi_write_then_read(idev->spi, txbuf, 1, rxbuf, len) < 0) {
		ipio_err("spi read data error in ice mode\n");
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static int ilitek_spi_write(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi write len is invaild\n");
		return -EINVAL;
	}

	ret = core_spi_write(buf, len);
	if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi write error, ret = %d\n", ret);
	}

out:
	return ret;
}

/* If ilitek_spi_read success ,this format will return read length */
static int ilitek_spi_read(void *buf, int len)
{
	int ret = 0;

	if (!len) {
		ipio_err("spi read len is invaild\n");
		return -EINVAL;
	}

	ret = core_spi_read(buf, len);
	if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == START) {
			ret = 0;
			goto out;
		}
		ipio_err("spi read error, ret = %d\n", ret);
	}

out:
	return ret;
}

int core_spi_setup(int num)
{
#if defined(CONFIG_MACH_MT6757)
	struct mt_chip_conf *chip_config;
	uint32_t temp_pulse_width = 0;
#endif
	u32 freq[] = {
		TP_SPI_CLK_1M,
		TP_SPI_CLK_2M,
		TP_SPI_CLK_3M,
		TP_SPI_CLK_4M,
		TP_SPI_CLK_5M,
		TP_SPI_CLK_6M,
		TP_SPI_CLK_7M,
		TP_SPI_CLK_8M,
		TP_SPI_CLK_9M,
		TP_SPI_CLK_10M,
		TP_SPI_CLK_11M,
		TP_SPI_CLK_12M,
		TP_SPI_CLK_13M,
		TP_SPI_CLK_14M,
		TP_SPI_CLK_15M
	};

#if defined(CONFIG_MACH_MT6757)
	chip_config = (struct mt_chip_conf *)idev->spi->controller_data;
	if (!chip_config) {
		ipio_err("chip_config is NULL.\n");
		chip_config = kzalloc(sizeof(struct mt_chip_conf), GFP_KERNEL);
		if (!chip_config)
			return -ENOMEM;
	}
	ipio_info("setup22222\n");
	temp_pulse_width = ((112 * 1000000) / freq[num]);
	temp_pulse_width = temp_pulse_width / 2;

	chip_config->setuptime = temp_pulse_width * 2;// for CS
	chip_config->holdtime = temp_pulse_width * 2;// for CS
	chip_config->high_time = temp_pulse_width;// for CLK = 1M
	chip_config->low_time = temp_pulse_width;// for CLK= 1M
	chip_config->cs_idletime = temp_pulse_width * 2;// for CS
	chip_config->rx_mlsb = 1;
	chip_config->tx_mlsb = 1;
	chip_config->tx_endian = 0;
	chip_config->rx_endian = 0;
	chip_config->cpol = 0;
	chip_config->cpha = 0;
	chip_config->com_mod = DMA_TRANSFER;
	//chip_config->com_mod = FIFO_TRANSFER;
	chip_config->pause = 1;
	chip_config->finish_intr = 1;
	chip_config->deassert = 0;

	idev->spi->controller_data = chip_config;
#endif
	if (num > sizeof(freq)) {
		ipio_err("Invaild clk freq\n");
		return -1;
	}

	ipio_info("spi clock = %d\n", freq[num]);

	idev->spi->mode = SPI_MODE_0;
	idev->spi->bits_per_word = 8;
	idev->spi->max_speed_hz = freq[num];
	#if defined(CONFIG_MACH_MT6757)
	idev->spi->chip_select = 0;
	#else
//	idev->spi->chip_select = 0;
	#endif

	if (spi_setup(idev->spi) < 0) {
		ipio_err("Failed to setup spi device\n");
		return -ENODEV;
	}

	ipio_info("name = %s, bus_num = %d,cs = %d, mode = %d, speed = %d\n",
			idev->spi->modalias,
			idev->spi->master->bus_num,
			idev->spi->chip_select,
			idev->spi->mode,
			idev->spi->max_speed_hz);
	return 0;
}

static int ilitek_spi_probe(struct spi_device *spi)
{
	struct touch_bus_info *info =
	container_of(to_spi_driver(spi->dev.driver),
		struct touch_bus_info, bus_driver);

	ipio_info("ilitek spi probe\n");

	if (!spi) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		ipio_err("Full duplex not supported by master\n");
		return -EIO;
	}

	idev = devm_kzalloc(&spi->dev, sizeof(struct ilitek_tddi_dev), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev)) {
		ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
		return -ENOMEM;
	}

	idev->update_buf = kzalloc(MAX_HEX_FILE_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev->update_buf)) {
		ipio_err("Failed to allocate update_buf\n");
		return -ENOMEM;
	}

	/* Used for receiving touch data only, do not mix up with others. */
	idev->tr_buf = kzalloc(TR_BUF_SIZE, GFP_ATOMIC);
	if (ERR_ALLOC_MEM(idev->tr_buf)) {
		ipio_err("failed to allocate touch report buffer\n");
		return -ENOMEM;
	}

	idev->spi_tx = kzalloc(SPI_TX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev->spi_tx)) {
		ipio_err("Failed to allocate spi tx buffer\n");
		return -ENOMEM;
	}

	idev->spi_rx = kzalloc(SPI_RX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(idev->spi_rx)) {
		ipio_err("Failed to allocate spi rx buffer\n");
		return -ENOMEM;
	}

	idev->gcoord = kzalloc(sizeof(struct gesture_coordinate), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev->gcoord)) {
		ipio_err("Failed to allocate gresture coordinate buffer\n");
		return -ENOMEM;
	}

	idev->i2c = NULL;
	idev->spi = spi;
	idev->dev = &spi->dev;
	idev->hwif = info->hwif;
	idev->phys = "SPI";

	idev->write = ilitek_spi_write;
	idev->read = ilitek_spi_read;
#if SPI_DMA_TRANSFER_SPLIT
	idev->spi_write_then_read = ilitek_spi_write_then_read_split;
#else
	idev->spi_write_then_read = ilitek_spi_write_then_read_direct;
#endif

	idev->spi_speed = ilitek_tddi_ic_spi_speed_ctrl;
	idev->spi_ack = ilitek_spi_check_ack;
	idev->actual_tp_mode = P5_X_FW_AP_MODE;
	idev->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;

	if (TDDI_RST_BIND)
		idev->reset = TP_IC_WHOLE_RST;
	else
		idev->reset = TP_HW_RST_ONLY;

	idev->rst_edge_delay = 5;
	idev->fw_open = FILP_OPEN;
	idev->fw_upgrade_mode = UPGRADE_IRAM;
	idev->mp_move_code = ilitek_tddi_move_mp_code_iram;
	idev->gesture_move_code = ilitek_tddi_move_gesture_code_iram;
	idev->esd_recover = ilitek_tddi_wq_esd_spi_check;
	idev->ges_recover = ilitek_tddi_touch_esd_gesture_iram;
	idev->gesture_mode = DATA_FORMAT_GESTURE_INFO;
	idev->gesture_demo_ctrl = DISABLE;
	idev->wtd_ctrl = ON;
	idev->report = ENABLE;
	idev->netlink = DISABLE;
	idev->dnp = DISABLE;
	idev->irq_tirgger_type = IRQF_TRIGGER_FALLING;
	idev->info_from_hex = ENABLE;

#if ENABLE_GESTURE
	idev->gesture = ENABLE;
#endif

	if (core_spi_setup(SPI_CLK) < 0)
		return -EINVAL;

	return info->hwif->plat_probe();
}

static int ilitek_spi_remove(struct spi_device *spi)
{
	ipio_info();
	return 0;
}

static struct spi_device_id tp_spi_id[] = {
	{TDDI_DEV_ID, 0},
	{},
};

int ilitek_tddi_interface_dev_init(struct ilitek_hwif_info *hwif)
{
	struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ipio_err("faied to allocate spi_driver\n");
		return -ENOMEM;
	}

	if (hwif->bus_type != BUS_SPI) {
		ipio_err("Not SPI dev\n");
		return -EINVAL;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;

	info->bus_driver.probe = ilitek_spi_probe;
	info->bus_driver.remove = ilitek_spi_remove;
	info->bus_driver.id_table = tp_spi_id;

	info->hwif = hwif;
	return spi_register_driver(&info->bus_driver);
}

void ilitek_tddi_interface_dev_exit(struct ilitek_tddi_dev *idev)
{
	struct touch_bus_info *info = (struct touch_bus_info *)idev->hwif->info;

	ipio_info("remove spi dev\n");
	kfree(idev->update_buf);
	kfree(idev->spi_tx);
	kfree(idev->spi_rx);
	spi_unregister_driver(&info->bus_driver);
	ipio_kfree((void **)&info);
}
