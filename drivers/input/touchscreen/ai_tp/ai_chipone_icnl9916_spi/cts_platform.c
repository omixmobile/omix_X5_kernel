// SPDX-License-Identifier: GPL-2.0
#define LOG_TAG         "Plat"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_firmware.h"
#include "cts_sysfs.h"
#include "cts_tcs.h"

int tpd_rst_gpio_index = 0;
int tpd_int_gpio_index = 1;

//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  begin
#define KEY_DOUBLE_CLICK_WAKEUP                 KEY_F17 //187    //KEY_U
#define KEY_GESTURE_O                           195         //KEY_O
#define KEY_GESTURE_E                           KEY_F18     //KEY_E
#define KEY_GESTURE_M                           KEY_F19     //KEY_M
#define KEY_GESTURE_W                           KEY_F23     //KEY_W
#define KEY_GESTURE_V                           KEY_F13     //KEY_V

#ifdef CFG_CTS_GESTURE
int cts_wake_switch = 0;
int cts_gesture_switch = 0;

struct ges_feature_info
{
    union {
        struct{
            u16 start_x;
            u16 start_y;
            u16 end_x;
            u16 end_y;
            u16 width;
            u16 height;
            u16 mid_x;
            u16 mid_y;
            u16 top_x;
            u16 top_y;
            u16 bottom_x;
            u16 bottom_y;
            u16 left_x;
            u16 left_y;
            u16 right_x;
            u16 right_y;
        };
        u16 data[16];
    };
};

struct cts_gesture_st {
    struct ges_feature_info f_point;
};

static struct cts_gesture_st cts_gesture_data;
#endif /* CFG_CTS_GESTURE */
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  end
#ifdef CFG_CTS_FW_LOG_REDIRECT
size_t cts_plat_get_max_fw_log_size(struct cts_platform_data *pdata)
{
    return CTS_FW_LOG_BUF_LEN;
}

u8 *cts_plat_get_fw_log_buf(struct cts_platform_data *pdata, size_t size)
{
    return pdata->fw_log_buf;
}
#endif

#ifdef CONFIG_CTS_I2C_HOST
size_t cts_plat_get_max_i2c_xfer_size(struct cts_platform_data *pdata)
{
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->i2c_dma_available) {
        return CFG_CTS_MAX_I2C_XFER_SIZE;
    } else {
        return CFG_CTS_MAX_I2C_FIFO_XFER_SIZE;
    }
#else /* TPD_SUPPORT_I2C_DMA */
    return CFG_CTS_MAX_I2C_XFER_SIZE;
#endif /* TPD_SUPPORT_I2C_DMA */
}

u8 *cts_plat_get_i2c_xfer_buf(struct cts_platform_data *pdata, size_t xfer_size)
{
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->i2c_dma_available && xfer_size > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        return pdata->i2c_dma_buff_va;
    } else
#endif /* TPD_SUPPORT_I2C_DMA */
        return pdata->i2c_fifo_buf;
}

int cts_plat_i2c_write(struct cts_platform_data *pdata, u8 i2c_addr,
        const void *src, size_t len, int retry, int delay)
{
    int ret = 0, retries = 0;

#ifdef TPD_SUPPORT_I2C_DMA
    struct i2c_msg msg = {
        .addr    = i2c_addr,
        .flags    = !I2C_M_RD,
        .len    = len,
        .timing = 300,
    };

    if (pdata->i2c_dma_available && len > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        msg.ext_flag = (pdata->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
        msg.buf = (u8 *)pdata->i2c_dma_buff_pa;
        memcpy(pdata->i2c_dma_buff_va, src, len);
    } else {
        msg.buf = (u8 *)src;
    }
    msg.len  = len;
#else
    struct i2c_msg msg = {
        .flags = !I2C_M_RD,
        .addr = i2c_addr,
        .buf = (u8 *) src,
        .len = len,
    };
#endif /* TPD_SUPPORT_I2C_DMA */

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter, &msg, 1);
        if (ret != 1) {
            if (ret >= 0)
                ret = -EIO;

            if (delay)
                mdelay(delay);
            continue;
        } else
            return 0;
    } while (++retries < retry);

    return ret;
}

int cts_plat_i2c_read(struct cts_platform_data *pdata, u8 i2c_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int num_msg, ret = 0, retries = 0;

#ifdef TPD_SUPPORT_I2C_DMA
    struct i2c_msg msgs[2] = {
        {
            .addr    = i2c_addr,
            .flags    = !I2C_M_RD,
            .len    = wlen,
            .buf    = (u8 *)wbuf,
            .timing = 300,
        },
        {
            .addr      = i2c_addr,
            .flags      = I2C_M_RD,
            .len      = rlen,
            .timing   = 300,
        },
    };

    if (pdata->i2c_dma_available && rlen > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
        msgs[1].ext_flag = (pdata->i2c_client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
        msgs[1].buf  = (u8 *)pdata->i2c_dma_buff_pa;
    } else {
        msgs[1].buf = (u8 *)rbuf;
    }
#else /* TPD_SUPPORT_I2C_DMA */
    struct i2c_msg msgs[2] = {
        {
         .addr = i2c_addr,
         .flags = !I2C_M_RD,
         .buf = (u8 *) wbuf,
         .len = wlen },
        {
         .addr = i2c_addr,
         .flags = I2C_M_RD,
         .buf = (u8 *) rbuf,
         .len = rlen }
    };
#endif /* TPD_SUPPORT_I2C_DMA */

    if (wbuf == NULL || wlen == 0)
        num_msg = 1;
    else
        num_msg = 2;

    do {
        ret = i2c_transfer(pdata->i2c_client->adapter,
                msgs + ARRAY_SIZE(msgs) - num_msg, num_msg);

        if (ret != num_msg) {
            if (ret >= 0)
                ret = -EIO;

            if (delay)
                mdelay(delay);
            continue;
        } else {
#ifdef TPD_SUPPORT_I2C_DMA
        if (pdata->i2c_dma_available && rlen > CFG_CTS_MAX_I2C_FIFO_XFER_SIZE) {
            memcpy(rbuf, pdata->i2c_dma_buff_va, rlen);
        }
#endif /* TPD_SUPPORT_I2C_DMA */

            return 0;
        }
    } while (++retries < retry);

    return ret;
}

int cts_plat_is_i2c_online(struct cts_platform_data *pdata, u8 i2c_addr)
{
    u8 dummy_bytes[2] = { 0x00, 0x00 };
    int ret;

    ret = cts_plat_i2c_write(pdata, i2c_addr, dummy_bytes,
            sizeof(dummy_bytes), 5, 2);
    if (ret) {
        cts_err("!!! I2C addr 0x%02x is offline !!!", i2c_addr);
        return false;
    }

    cts_dbg("I2C addr 0x%02x is online", i2c_addr);
    return true;
}
#endif

#ifdef CONFIG_CTS_SPI_HOST
void dump_spi_common(const char *prefix, u8 *data, size_t datalen)
{
    u8 str[1024];
    int offset = 0;
    int i;

    offset += snprintf(str + offset, sizeof(str) - offset, "%s", prefix);
    for (i = 0; i < datalen; i++) {
        offset += snprintf(str + offset, sizeof(str) - offset, " %02x", data[i]);
    }
    cts_err("%s", str);
}

#ifdef CFG_MTK_LEGEND_PLATFORM
struct mt_chip_conf cts_spi_conf_mt65xx = {
    .setuptime = 15,
    .holdtime = 15,
    .high_time = 21, //for mt6582, 104000khz/(4+4) = 130000khz
    .low_time = 21,
    .cs_idletime = 20,
    .ulthgh_thrsh = 0,

    .cpol = 0,
    .cpha = 0,

    .rx_mlsb = 1,
    .tx_mlsb = 1,

    .tx_endian = 0,
    .rx_endian = 0,

    .com_mod = FIFO_TRANSFER,
    .pause = 1,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 0,
};

typedef enum {
    SPEED_500KHZ = 500,
    SPEED_1MHZ = 1000,
    SPEED_2MHZ = 2000,
    SPEED_3MHZ = 3000,
    SPEED_4MHZ = 4000,
    SPEED_6MHZ = 6000,
    SPEED_8MHZ = 8000,
    SPEED_KEEP,
    SPEED_UNSUPPORTED
} SPI_SPEED;

void cts_plat_spi_set_mode(struct spi_device *spi, SPI_SPEED speed, int flag)
{
    struct mt_chip_conf *mcc = &cts_spi_conf_mt65xx;
    int ret;

    if (flag == 0) {
        mcc->com_mod = FIFO_TRANSFER;
    } else {
        mcc->com_mod = DMA_TRANSFER;
    }

    switch (speed) {
    case SPEED_500KHZ:
        mcc->high_time = 120;
        mcc->low_time = 120;
        break;
    case SPEED_1MHZ:
        mcc->high_time = 60;
        mcc->low_time = 60;
        break;
    case SPEED_2MHZ:
        mcc->high_time = 30;
        mcc->low_time = 30;
        break;
    case SPEED_3MHZ:
        mcc->high_time = 20;
        mcc->low_time = 20;
        break;
    case SPEED_4MHZ:
        mcc->high_time = 15;
        mcc->low_time = 15;
        break;
    case SPEED_6MHZ:
        mcc->high_time = 10;
        mcc->low_time = 10;
        break;
    case SPEED_8MHZ:
        mcc->high_time = 8;
        mcc->low_time = 8;
        break;
    case SPEED_KEEP:
    case SPEED_UNSUPPORTED:
        break;
    }

    ret = spi_setup(spi);
    if (ret) {
        cts_err("Spi setup failed %d(%s)", ret, cts_strerror(ret));
    }
}

int cts_plat_spi_setup(struct cts_platform_data *pdata)
{
    pdata->spi_client->mode = SPI_MODE_0;
    pdata->spi_client->bits_per_word = 8;
    pdata->spi_client->chip_select = 0;

    cts_info(" - chip_select  :%d", pdata->spi_client->chip_select);
    cts_info(" - spi_mode     :%d", pdata->spi_client->mode);
    cts_info(" - bits_per_word:%d", pdata->spi_client->bits_per_word);

    pdata->spi_client->controller_data = (void *)&cts_spi_conf_mt65xx;
    spi_setup(pdata->spi_client);
    cts_plat_spi_set_mode(pdata->spi_client, pdata->spi_speed, 0);
    return 0;
}
#else
int cts_plat_spi_setup(struct cts_platform_data *pdata)
{
    int ret;

    pdata->spi_client->chip_select = 0;
    pdata->spi_client->mode = SPI_MODE_0;
    pdata->spi_client->bits_per_word = 8;

    cts_info("chip_select  :%d", pdata->spi_client->chip_select);
    cts_info("spi_mode     :%d", pdata->spi_client->mode);
    cts_info("bits_per_word:%d", pdata->spi_client->bits_per_word);

    ret = spi_setup(pdata->spi_client);
    if (ret)
        cts_err("spi_setup err!");
    return 0;
}
#endif

#ifdef CFG_CTS_MANUAL_CS
int cts_plat_set_cs(struct cts_platform_data *pdata, int val)
{
    if (val)
        pinctrl_select_state(pdata->pinctrl1, pdata->spi_cs_high);
    else
        pinctrl_select_state(pdata->pinctrl1, pdata->spi_cs_low);

    return 0;
}
#endif

int cts_spi_send_recv(struct cts_platform_data *pdata, size_t len,
        u8 *tx_buffer, u8 *rx_buffer)
{
    struct chipone_ts_data *cts_data;
    struct spi_message msg;
    struct spi_transfer cmd = {
        .delay_usecs = 0,
        .speed_hz = pdata->spi_speed * 1000u,
        .tx_buf = tx_buffer,
        .rx_buf = rx_buffer,
        .len = len,
        /* .tx_dma = 0, */
        /* .rx_dma = 0, */
    };
    int ret = 0;

    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

#ifdef CFG_CTS_MANUAL_CS
    cts_plat_set_cs(pdata, 0);
#endif
    spi_message_init(&msg);
    spi_message_add_tail(&cmd, &msg);
    ret = spi_sync(cts_data->spi_client, &msg);
    if (ret)
        cts_err("spi sync failed %d", ret);

    udelay(100);

#ifdef CFG_CTS_MANUAL_CS
    cts_plat_set_cs(pdata, 1);
#endif
    return ret;
}

size_t cts_plat_get_max_spi_xfer_size(struct cts_platform_data *pdata)
{
    return CFG_CTS_MAX_SPI_XFER_SIZE;
}

u8 *cts_plat_get_spi_xfer_buf(struct cts_platform_data *pdata, size_t xfer_size)
{
    return pdata->spi_cache_buf;
}

int cts_plat_spi_write(struct cts_platform_data *pdata, u8 dev_addr,
        const void *src, size_t len, int retry, int delay)
{
    int ret = 0, retries = 0;
    u16 crc16_calc;
    size_t data_len;

    if (len > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write too much data:wlen=%zu", len);
        return -EIO;
    }

    if (pdata->cts_dev->rtdata.program_mode) {
#ifdef CONFIG_CTS_ICTYPE_ICNL9922C
        data_len = len - 3;
        pdata->spi_tx_buf[0] = dev_addr;
        memcpy(&pdata->spi_tx_buf[1], src, 3);
        put_unaligned_be24(data_len, &pdata->spi_tx_buf[4]);
        crc16_calc = (u16) cts_crc16(pdata->spi_tx_buf, 7);
        put_unaligned_be16(~crc16_calc, &pdata->spi_tx_buf[7]);
        memcpy(&pdata->spi_tx_buf[13], (char *)src + 3, data_len);
        crc16_calc = (u16) cts_crc16((char *)src + 3, data_len);
        put_unaligned_be16(~crc16_calc, &pdata->spi_tx_buf[len + 10]);
        memset(pdata->spi_tx_buf + len + 12, 0, 3);

        do {
            ret = cts_spi_send_recv(pdata, len + 15, pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI write failed %d", ret);
                if (delay)
                    mdelay(delay);
            } else
                return 0;
        } while (++retries < retry);
#else
        pdata->spi_tx_buf[0] = dev_addr;
        memcpy(&pdata->spi_tx_buf[1], src, len);
        do {
            ret = cts_spi_send_recv(pdata, len + 1, pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI write failed %d", ret);
                if (delay)
                    mdelay(delay);
            } else
                return 0;
        } while (++retries < retry);
#endif
    } else {
        data_len = len - 2;
        pdata->spi_tx_buf[0] = dev_addr;
        pdata->spi_tx_buf[1] = *((u8 *) src + 1);
        pdata->spi_tx_buf[2] = *((u8 *) src);
        put_unaligned_le16(data_len, &pdata->spi_tx_buf[3]);
        crc16_calc = (u16) cts_crc32(pdata->spi_tx_buf, 5);
        put_unaligned_le16(crc16_calc, &pdata->spi_tx_buf[5]);
        memcpy(&pdata->spi_tx_buf[7], (char *)src + 2, data_len);
        crc16_calc = (u16) cts_crc32((char *)src + 2, data_len);
        put_unaligned_le16(crc16_calc, &pdata->spi_tx_buf[7 + data_len]);
        do {
            ret = cts_spi_send_recv(pdata, len + 7, pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            udelay(10 * data_len);
            if (ret) {
                cts_err("SPI write failed %d", ret);
                if (delay)
                    mdelay(delay);
            } else
                return 0;
        } while (++retries < retry);
    }
    return ret;
}

int cts_plat_spi_read(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf, size_t rlen,
        int retry, int delay)
{
    int ret = 0, retries = 0;
    u16 crc16_calc, crc16_recv;

    if (wlen > CFG_CTS_MAX_SPI_XFER_SIZE
    || rlen > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write/read too much data:wlen=%zd, rlen=%zd", wlen, rlen);
        return -EIO;
    }

    if (pdata->cts_dev->rtdata.program_mode) {
#ifdef CONFIG_CTS_ICTYPE_ICNL9922C
        memset(pdata->spi_tx_buf, 0, CFG_CTS_MAX_SPI_XFER_SIZE);
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        put_unaligned_be24(rlen, &pdata->spi_tx_buf[4]);
        crc16_calc = (u16) cts_crc16(pdata->spi_tx_buf, 7);
        put_unaligned_be16(~crc16_calc, &pdata->spi_tx_buf[7]);

        do {
            ret = cts_spi_send_recv(pdata, wlen + rlen + 15,
                pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf + wlen + 10, rlen);
            crc16_calc = (u16) cts_crc16(rbuf, rlen);
            crc16_recv = get_unaligned_be16(&pdata->spi_rx_buf[wlen + rlen + 10]);
            if (crc16_recv != (uint16_t)(~crc16_calc)) {
                cts_err("SPI RX CRC error: rx_crc %04x != %04x",
                    crc16_recv, ~crc16_calc);
                continue;
            }
            return 0;
        } while (++retries < retry);
#else
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        do {
            ret = cts_spi_send_recv(pdata, rlen + 5, pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf + 5, rlen);
            return 0;
        } while (++retries < retry);
#endif
    } else {
        do {
            if (wlen != 0) {
                pdata->spi_tx_buf[0] = dev_addr | 0x01;
                pdata->spi_tx_buf[1] = wbuf[1];
                pdata->spi_tx_buf[2] = wbuf[0];
                put_unaligned_le16(rlen, &pdata->spi_tx_buf[3]);
                crc16_calc = (u16) cts_crc32(pdata->spi_tx_buf, 5);
                put_unaligned_le16(crc16_calc, &pdata->spi_tx_buf[5]);
                ret = cts_spi_send_recv(pdata, 7, pdata->spi_tx_buf,
                        pdata->spi_rx_buf);
                if (ret) {
                    cts_err("SPI read failed %d", ret);
                    if (delay)
                        mdelay(delay);
                    continue;
                }
            }
            memset(pdata->spi_tx_buf, 0, 7);
            pdata->spi_tx_buf[0] = dev_addr | 0x01;
            udelay(100);
            ret = cts_spi_send_recv(pdata, rlen + 2,
                    pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf, rlen);
            crc16_calc = (u16) cts_crc32(pdata->spi_rx_buf, rlen);
            crc16_recv = get_unaligned_le16(&pdata->spi_rx_buf[rlen]);
            if (crc16_recv != crc16_calc) {
                cts_err("SPI RX CRC error: rx_crc %04x != %04x",
                        crc16_recv, crc16_calc);
                continue;
            }
            return 0;
        } while (++retries < retry);
    }
    if (retries >= retry)
        cts_err("SPI read too much retry");

    return -EIO;
}

int cts_plat_spi_read_delay_idle(struct cts_platform_data *pdata, u8 dev_addr,
        const u8 *wbuf, size_t wlen, void *rbuf,
        size_t rlen, int retry, int delay, int idle)
{
    int ret = 0, retries = 0;
    u16 crc;

    if (wlen > CFG_CTS_MAX_SPI_XFER_SIZE
    || rlen > CFG_CTS_MAX_SPI_XFER_SIZE) {
        cts_err("write/read too much data:wlen=%zu, rlen=%zu", wlen, rlen);
        return -E2BIG;
    }

    if (pdata->cts_dev->rtdata.program_mode) {
#ifdef CONFIG_CTS_ICTYPE_ICNL9922C
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        put_unaligned_be24(rlen, &pdata->spi_tx_buf[4]);
        crc = (u16) cts_crc32(pdata->spi_tx_buf, 7);
        put_unaligned_be16(crc, &pdata->spi_tx_buf[7]);
        do {
            ret = cts_spi_send_recv(pdata, wlen + rlen + 15,
                pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf + wlen + 10, rlen);
            crc = (u16) cts_crc32(rbuf, rlen);
            if (get_unaligned_le16(&pdata->spi_rx_buf[wlen + rlen + 10]) != crc) {
                cts_err("SPI RX CRC error: rx_crc %04x != %04x",
                    get_unaligned_le16(&pdata->spi_rx_buf[wlen + rlen + 10]), crc);
                continue;
            }
            return 0;
        } while (++retries < retry);
#else
        pdata->spi_tx_buf[0] = dev_addr | 0x01;
        memcpy(&pdata->spi_tx_buf[1], wbuf, wlen);
        do {
            ret = cts_spi_send_recv(pdata, rlen + 5, pdata->spi_tx_buf,
                    pdata->spi_rx_buf);
            if (ret) {
                cts_err("SPI read failed %d", ret);
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf + 5, rlen);
            return 0;
        } while (++retries < retry);
#endif
    } else {
        do {
            if (wlen != 0) {
                pdata->spi_tx_buf[0] = dev_addr | 0x01;
                pdata->spi_tx_buf[1] = wbuf[1];
                pdata->spi_tx_buf[2] = wbuf[0];
                put_unaligned_le16(rlen, &pdata->spi_tx_buf[3]);
                crc = (u16) cts_crc32(pdata->spi_tx_buf, 5);
                put_unaligned_le16(crc, &pdata->spi_tx_buf[5]);
                ret = cts_spi_send_recv(pdata, 7, pdata->spi_tx_buf,
                        pdata->spi_rx_buf);
                if (ret) {
                    cts_err("SPI read failed %d", ret);
                    if (delay)
                        mdelay(delay);
                    continue;
                }
            }
            memset(pdata->spi_tx_buf, 0, 7);
            pdata->spi_tx_buf[0] = dev_addr | 0x01;
            udelay(idle);
            ret = cts_spi_send_recv(pdata, rlen + 2,
                pdata->spi_tx_buf, pdata->spi_rx_buf);
            if (ret) {
                if (delay)
                    mdelay(delay);
                continue;
            }
            memcpy(rbuf, pdata->spi_rx_buf, rlen);
            crc = (u16) cts_crc32(pdata->spi_rx_buf, rlen);
            if (get_unaligned_le16(&pdata->spi_rx_buf[rlen]) != crc)
                continue;
            return 0;
        } while (++retries < retry);
    }
    if (retries >= retry)
        cts_err("cts_plat_spi_read error");

    return -EIO;
}

int cts_plat_is_normal_mode(struct cts_platform_data *pdata)
{
    struct chipone_ts_data *cts_data;
    u16 fwid;
/*
    u8 tx_buf[4] = { 0 };
    u32 addr;
*/
    int ret;

    cts_set_normal_addr(pdata->cts_dev);
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);
    ret = cts_tcs_get_fw_id(pdata->cts_dev, &fwid);
/*
        addr = CTS_DEVICE_FW_REG_CHIP_TYPE;
        put_unaligned_be16(addr, tx_buf);
        ret = cts_plat_spi_read(pdata, CTS_DEV_NORMAL_MODE_SPIADDR,
                tx_buf, 2, &fwid, 2, 3, 10);
        fwid = be16_to_cpu(fwid);
*/
    if (ret || !cts_is_fwid_valid(fwid))
        return false;

    return true;
}
#endif

static void cts_plat_handle_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_dbg("Handle IRQ");

    cts_lock_device(pdata->cts_dev);
    ret = cts_irq_handler(pdata->cts_dev);
    if (ret)
        cts_err("Device handle IRQ failed %d", ret);
    cts_unlock_device(pdata->cts_dev);
}

static irqreturn_t cts_plat_irq_handler(int irq, void *dev_id)
{
    struct cts_platform_data *pdata;
#ifndef CONFIG_GENERIC_HARDIRQS
    struct chipone_ts_data *cts_data;
#endif

    cts_dbg("IRQ handler");

    pdata = (struct cts_platform_data *)dev_id;
    if (pdata == NULL) {
        cts_err("IRQ handler with NULL dev_id");
        return IRQ_NONE;
    }
#ifdef CONFIG_GENERIC_HARDIRQS
    cts_plat_handle_irq(pdata);
#else
    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

    if (queue_work(cts_data->workqueue, &pdata->ts_irq_work)) {
        cts_dbg("IRQ queue work");
        cts_plat_disable_irq(pdata);
    } else
        cts_warn("IRQ handler queue work failed as already on the queue");
#endif /* CONFIG_GENERIC_HARDIRQS */

    return IRQ_HANDLED;
}

#ifndef CONFIG_GENERIC_HARDIRQS
static void cts_plat_touch_dev_irq_work(struct work_struct *work)
{
    struct cts_platform_data *pdata =
        container_of(work, struct cts_platform_data, ts_irq_work);

    cts_dbg("IRQ work");

    cts_plat_handle_irq(pdata);

    cts_plat_enable_irq(pdata);
}
#endif /* CONFIG_GENERIC_HARDIRQS */

#ifdef CFG_CTS_FORCE_UP
static void cts_plat_touch_event_timeout_work(struct work_struct *work)
{
    struct cts_platform_data *pdata = container_of(work,
            struct cts_platform_data, touch_event_timeout_work.work);

    cts_warn("Touch event timeout work");

    cts_plat_release_all_touch(pdata);
}
#endif

#ifdef CONFIG_CTS_SPI_HOST
static int cts_plat_init_dts(struct cts_platform_data *pdata, struct device *device)
{
#ifdef CFG_CTS_MANUAL_CS
    struct device_node *node;

    pdata->pinctrl1 = devm_pinctrl_get(device);
    node = device->of_node;
    if (node) {
        pdata->spi_cs_low = pinctrl_lookup_state(pdata->pinctrl1, "spi_cs_low");
        if (IS_ERR(pdata->spi_cs_low)) {
            cts_err("Cannot find pinctrl spi cs high!\n");
            return -ENOENT;
        }
        pdata->spi_cs_high = pinctrl_lookup_state(pdata->pinctrl1, "spi_cs_high");
        if (IS_ERR(pdata->spi_cs_high)) {
            return -ENOENT;
        }
        return 0;
    }
    return -ENOENT;
#else
    return 0;
#endif
}
#endif /* CONFIG_CTS_SPI_HOST */

#ifdef CONFIG_CTS_I2C_HOST
int cts_init_platform_data(struct cts_platform_data *pdata,
        struct i2c_client *i2c_client)
#else
int cts_init_platform_data(struct cts_platform_data *pdata,
        struct spi_device *spi)
#endif
{
    struct device_node *node = NULL;
    u32 ints[2] = { 0, 0 };

    cts_info("cts_init_platform_data Init");

#ifdef CONFIG_CTS_OF
    {
        struct device *dev;

#ifdef CONFIG_CTS_I2C_HOST
        dev = &i2c_client->dev;
#else
        dev = &spi->dev;
#endif /* CONFIG_CTS_I2C_HOST */
    }
#endif /* CONFIG_CTS_OF */

#ifdef CONFIG_CTS_I2C_HOST
    pdata->i2c_client = i2c_client;
#else
    pdata->spi_client = spi;
#endif /* CONFIG_CTS_I2C_HOST */

    pdata->ts_input_dev = tpd->dev;

    spin_lock_init(&pdata->irq_lock);
    mutex_init(&pdata->dev_lock);

#if !defined(CONFIG_GENERIC_HARDIRQS)
    INIT_WORK(&pdata->ts_irq_work, cts_plat_touch_dev_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */

    if ((node = of_find_matching_node(node, touch_of_match)) == NULL) {
        cts_err("Find touch eint node failed");
        return -ENODATA;
    }
    if (of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints)) == 0) {
        gpio_set_debounce(ints[0], ints[1]);
    } else {
        cts_info("Debounce time not found");
    }
    pdata->irq = irq_of_parse_and_map(node, 0);
    if (pdata->irq == 0) {
        cts_err("Parse irq in dts failed");
        return -ENODEV;
    }

#ifdef CONFIG_CTS_VIRTUALKEY
    pdata->vkey_num = tpd_dts_data.tpd_keycnt;
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CFG_CTS_GESTURE
    {
        u8 gesture_keymap[CFG_CTS_NUM_GESTURE][2] = CFG_CTS_GESTURE_KEYMAP;
        memcpy(pdata->gesture_keymap, gesture_keymap, sizeof(gesture_keymap));
        pdata->gesture_num = CFG_CTS_NUM_GESTURE;
    }
#endif /* CFG_CTS_GESTURE */

#ifdef TPD_SUPPORT_I2C_DMA
        tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
        pdata->i2c_dma_buff_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev,
                CFG_CTS_MAX_I2C_XFER_SIZE, &pdata->i2c_dma_buff_pa, GFP_KERNEL);
        if (pdata->i2c_dma_buff_va == NULL) {
            cts_err("Allocate I2C DMA Buffer failed!");
            //return -ENOMEM;
        } else {
            pdata->i2c_dma_available = true;
        }
#endif /* TPD_SUPPORT_I2C_DMA */

#ifdef CFG_CTS_FORCE_UP
    INIT_DELAYED_WORK(&pdata->touch_event_timeout_work,
              cts_plat_touch_event_timeout_work);
#endif

#ifdef CONFIG_CTS_SPI_HOST
    cts_plat_init_dts(pdata, &spi->dev);
    pdata->spi_speed = CFG_CTS_SPI_SPEED_KHZ;
    cts_plat_spi_setup(pdata);
#endif
    return 0;
}

int cts_plat_request_resource(struct cts_platform_data *pdata)
{
    cts_info("Request resource");

    tpd_gpio_as_int(tpd_int_gpio_index);
    tpd_gpio_output(tpd_rst_gpio_index, 1);

    return 0;
}

void cts_plat_free_resource(struct cts_platform_data *pdata)
{
    cts_info("Free resource");

    /**
     * Note:
     *    If resource request without managed, should free all resource
     *    requested in cts_plat_request_resource().
     */
#ifdef TPD_SUPPORT_I2C_DMA
    if (pdata->i2c_dma_buff_va) {
        dma_free_coherent(&tpd->dev->dev, CFG_CTS_MAX_I2C_XFER_SIZE,
        pdata->i2c_dma_buff_va, pdata->i2c_dma_buff_pa);
        pdata->i2c_dma_buff_va = NULL;
        pdata->i2c_dma_buff_pa = 0;
    }
#endif /* TPD_SUPPORT_I2C_DMA */
}

int cts_plat_request_irq(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Request IRQ");

#ifdef CONFIG_GENERIC_HARDIRQS
    ret = request_threaded_irq(pdata->irq, NULL, cts_plat_irq_handler,
            IRQF_TRIGGER_FALLING | IRQF_ONESHOT, CFG_CTS_DRIVER_NAME, pdata);
#else /* CONFIG_GENERIC_HARDIRQS */
    ret = request_irq(pdata->irq, cts_plat_irq_handler,
            IRQF_TRIGGER_FALLING | IRQF_ONESHOT, CFG_CTS_DRIVER_NAME, pdata);
#endif /* CONFIG_GENERIC_HARDIRQS */
    if (ret) {
        cts_err("Request IRQ failed %d", ret);
        return ret;
    }

    cts_plat_disable_irq(pdata);

    return 0;
}

void cts_plat_free_irq(struct cts_platform_data *pdata)
{
    free_irq(pdata->irq, pdata);
}

int cts_plat_enable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Enable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (pdata->irq_is_disable) {    /* && !cts_is_device_suspended(pdata->chip)) */
            cts_dbg("Real enable IRQ");
            enable_irq(pdata->irq);
            pdata->irq_is_disable = false;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

int cts_plat_disable_irq(struct cts_platform_data *pdata)
{
    unsigned long irqflags;

    cts_dbg("Disable IRQ");

    if (pdata->irq > 0) {
        spin_lock_irqsave(&pdata->irq_lock, irqflags);
        if (!pdata->irq_is_disable) {
            cts_dbg("Real disable IRQ");
            disable_irq_nosync(pdata->irq);
            pdata->irq_is_disable = true;
        }
        spin_unlock_irqrestore(&pdata->irq_lock, irqflags);

        return 0;
    }

    return -ENODEV;
}

#ifdef CFG_CTS_HAS_RESET_PIN
int cts_plat_reset_device(struct cts_platform_data *pdata)
{
    /* !!!can not be modified */
    /* !!!can not be modified */
    /* !!!can not be modified */
    cts_info("Reset device");

    tpd_gpio_output(tpd_rst_gpio_index, 0);
    mdelay(1);
    tpd_gpio_output(tpd_rst_gpio_index, 1);
    mdelay(50);

    return 0;
}

int cts_plat_set_reset(struct cts_platform_data *pdata, int val)
{
    cts_info("Set Reset to %s", val ? "HIGH" : "LOW");
    if (val)
        tpd_gpio_output(tpd_rst_gpio_index, 1);
    else
        tpd_gpio_output(tpd_rst_gpio_index, 0);

    return 0;
}
#endif /* CFG_CTS_HAS_RESET_PIN */

int cts_plat_get_int_pin(struct cts_platform_data *pdata)
{
    /* MTK platform can not get INT pin value */
    return -ENOTSUPP;
}

int cts_plat_power_up_device(struct cts_platform_data *pdata)
{
    cts_info("Power up device");

    return 0;
}

int cts_plat_power_down_device(struct cts_platform_data *pdata)
{
    cts_info("Power down device");

    return 0;
}

int cts_plat_init_touch_device(struct cts_platform_data *pdata)
{
    cts_info("Init touch device");

    return input_mt_init_slots(pdata->ts_input_dev,
        tpd_dts_data.touch_max_num, INPUT_MT_DIRECT);

    return 0;
}

void cts_plat_deinit_touch_device(struct cts_platform_data *pdata)
{
    cts_info("De-init touch device");

#ifndef CONFIG_GENERIC_HARDIRQS
    if (work_pending(&pdata->ts_irq_work))
        cancel_work_sync(&pdata->ts_irq_work);
#endif /* CONFIG_GENERIC_HARDIRQS */
}

#ifdef CFG_CTS_PALM_DETECT
void cts_report_palm_event(struct cts_platform_data *pdata)
{
    input_report_key(pdata->ts_input_dev, CFG_CTS_PALM_EVENT, 1);
    input_sync(pdata->ts_input_dev);
    msleep(100);
    input_report_key(pdata->ts_input_dev, CFG_CTS_PALM_EVENT, 0);
    input_sync(pdata->ts_input_dev);
}
#endif

static int tpd_history_x, tpd_history_y;
int cts_plat_process_touch_msg(struct cts_platform_data *pdata,
        struct cts_device_touch_msg *msgs, int num)
{
    struct chipone_ts_data *cts_data;
    struct input_dev *input_dev = pdata->ts_input_dev;
    int i;
    int contact = 0;
#ifdef CONFIG_CTS_SLOTPROTOCOL
    static unsigned char finger_last[CFG_CTS_MAX_TOUCH_NUM] = { 0 };
    unsigned char finger_current[CFG_CTS_MAX_TOUCH_NUM] = { 0 };
#endif
    //Antai <AI_BSP_TP> <chenht> <2023-04-27> add debug info begin
    cts_info("Process touch %d msgs", num);
    //Antai <AI_BSP_TP> <chenht> <2023-04-27> add debug info end

    cts_data = container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

    if (num == 0 || num > CFG_CTS_MAX_TOUCH_NUM)
        return 0;

    for (i = 0; i < num; i++) {
        u16 x, y;

        x = le16_to_cpu(msgs[i].x);
        y = le16_to_cpu(msgs[i].y);

#ifdef CFG_CTS_SWAP_XY
        swap(x, y);
#endif /* CFG_CTS_SWAP_XY */
#ifdef CFG_CTS_WRAP_X
        x = wrap(TPD_RES_X, x);
#endif /* CFG_CTS_WRAP_X */
#ifdef CFG_CTS_WRAP_Y
        y = wrap(TPD_RES_Y, y);
#endif /* CFG_CTS_WRAP_Y */
        //Antai <AI_BSP_TP> <chenht> <2023-04-27> add debug info begin
        cts_info("Process touch msg[%d]: id[%u] ev=%u x=%u y=%u p=%u",
            i, msgs[i].id, msgs[i].event, x, y, msgs[i].pressure);
        //Antai <AI_BSP_TP> <chenht> <2023-04-27> add debug info end

#ifdef CONFIG_CTS_SLOTPROTOCOL
        if (msgs[i].event == CTS_DEVICE_TOUCH_EVENT_DOWN ||
            msgs[i].event == CTS_DEVICE_TOUCH_EVENT_MOVE ||
            msgs[i].event == CTS_DEVICE_TOUCH_EVENT_STAY) {
            if (msgs[i].id < CFG_CTS_MAX_TOUCH_NUM)
                finger_current[msgs[i].id] = 1;
        }
        input_mt_slot(input_dev, msgs[i].id);

        switch (msgs[i].event) {
        case CTS_DEVICE_TOUCH_EVENT_DOWN:
            TPD_DEBUG_SET_TIME;
            TPD_EM_PRINT(x, y, x, y, msgs[i].id, 1);
            tpd_history_x = x;
            tpd_history_y = y;
#ifdef CONFIG_MTK_BOOT
            if (tpd_dts_data.use_tpd_button) {
                if (FACTORY_BOOT == get_boot_mode() ||
                    RECOVERY_BOOT == get_boot_mode())
                    tpd_button(x, y, 1);
            }
#endif /* CONFIG_MTK_BOOT */
        case CTS_DEVICE_TOUCH_EVENT_MOVE:
        case CTS_DEVICE_TOUCH_EVENT_STAY:
            contact++;
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
            input_report_abs(input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, msgs[i].pressure);
            input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
            break;

        case CTS_DEVICE_TOUCH_EVENT_UP:
            TPD_DEBUG_SET_TIME;
            TPD_EM_PRINT(tpd_history_x, tpd_history_y, tpd_history_x, tpd_history_y, msgs[i].id, 0);
            tpd_history_x = 0;
            tpd_history_y = 0;
#ifdef CONFIG_MTK_BOOT
            if (tpd_dts_data.use_tpd_button) {
                if (FACTORY_BOOT == get_boot_mode() ||
                    RECOVERY_BOOT == get_boot_mode())
                    tpd_button(0, 0, 0);
            }
#endif /* CONFIG_MTK_BOOT */
            //input_report_key(input_dev, BTN_TOUCH, 0);
            //input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
            break;

        default:
            cts_warn
                ("Process touch msg with unknwon event %u id %u",
                 msgs[i].event, msgs[i].id);
            break;
        }
#else /* CONFIG_CTS_SLOTPROTOCOL */
    /**
     * If the driver reports one of BTN_TOUCH or ABS_PRESSURE
     * in addition to the ABS_MT events, the last SYN_MT_REPORT event
     * may be omitted. Otherwise, the last SYN_REPORT will be dropped
     * by the input core, resulting in no zero-contact event
     * reaching userland.
     */
        switch (msgs[i].event) {
        case CTS_DEVICE_TOUCH_EVENT_DOWN:
        case CTS_DEVICE_TOUCH_EVENT_MOVE:
        case CTS_DEVICE_TOUCH_EVENT_STAY:
            contact++;
            input_report_abs(input_dev, ABS_MT_PRESSURE, msgs[i].pressure);
            input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, msgs[i].pressure);
            input_report_key(input_dev, BTN_TOUCH, 1);
            input_report_abs(input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
            input_mt_sync(input_dev);
            break;

        case CTS_DEVICE_TOUCH_EVENT_UP:
            break;
        default:
            cts_warn("Process touch msg with unknwon event %u id %u",
                 msgs[i].event, msgs[i].id);
            break;
        }
#endif /* CONFIG_CTS_SLOTPROTOCOL */
    }

#ifdef CONFIG_CTS_SLOTPROTOCOL
    for (i = 0; i < CFG_CTS_MAX_TOUCH_NUM; i++) {
        if (finger_last[i] != 0 && finger_current[i] == 0) {
            input_mt_slot(input_dev, i);
            input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
        }
        finger_last[i] = finger_current[i];
    }
    input_report_key(input_dev, BTN_TOUCH, contact > 0);
#else
    if (contact == 0) {
        input_report_key(input_dev, BTN_TOUCH, 0);
        input_mt_sync(input_dev);
    }
#endif
    input_sync(input_dev);

#ifdef CFG_CTS_FORCE_UP
    if (contact) {
        if (delayed_work_pending(&pdata->touch_event_timeout_work)) {
            mod_delayed_work(cts_data->workqueue,
                    &pdata->touch_event_timeout_work, msecs_to_jiffies(100));
        } else {
            queue_delayed_work(cts_data->workqueue,
                    &pdata->touch_event_timeout_work, msecs_to_jiffies(100));
        }
    } else {
        cancel_delayed_work_sync(&pdata->touch_event_timeout_work);
    }
#endif

#ifdef CFG_CTS_HEARTBEAT_MECHANISM
    if (contact) {
        if (delayed_work_pending(&cts_data->heart_work)) {
            mod_delayed_work(cts_data->heart_workqueue,
                    &cts_data->heart_work, msecs_to_jiffies(2000));
        } else {
            queue_delayed_work(cts_data->heart_workqueue,
                    &cts_data->heart_work, msecs_to_jiffies(2000));
        }
    }
#endif

    return 0;
}

int cts_plat_release_all_touch(struct cts_platform_data *pdata)
{
    struct input_dev *input_dev = pdata->ts_input_dev;

#if defined(CONFIG_CTS_SLOTPROTOCOL)
    int id;
#endif /* CONFIG_CTS_SLOTPROTOCOL */

    cts_info("Release all touch");

#ifdef CONFIG_CTS_SLOTPROTOCOL
    for (id = 0; id < CFG_CTS_MAX_TOUCH_NUM; id++) {
        input_mt_slot(input_dev, id);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
    }
    input_report_key(input_dev, BTN_TOUCH, 0);
#else
    input_report_key(input_dev, BTN_TOUCH, 0);
    input_mt_sync(input_dev);
#endif /* CONFIG_CTS_SLOTPROTOCOL */
    input_sync(input_dev);

    return 0;
}

#ifdef CONFIG_CTS_VIRTUALKEY
int cts_plat_init_vkey_device(struct cts_platform_data *pdata)
{
    pdata->vkey_state = 0;

    cts_info("Init VKey");

    if (tpd_dts_data.use_tpd_button) {
        cts_info("Init vkey");

        pdata->vkey_state = 0;
        tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
                tpd_dts_data.tpd_key_dim_local);
    }

    return 0;
}

void cts_plat_deinit_vkey_device(struct cts_platform_data *pdata)
{
    cts_info("De-init VKey");

    pdata->vkey_state = 0;
}

int cts_plat_process_vkey(struct cts_platform_data *pdata, u8 vkey_state)
{
    u8 event;
    int i;

    event = pdata->vkey_state ^ vkey_state;

    cts_dbg("Process vkey state=0x%02x, event=0x%02x", vkey_state, event);

    for (i = 0; i < pdata->vkey_num; i++) {
        if (event & BIT(i)) {
            tpd_button(x, y, vkey_state & BIT(i));

            /* MTK fobidon more than one key pressed in the same time */
            break;
        }
    }

    pdata->vkey_state = vkey_state;

    return 0;
}

int cts_plat_release_all_vkey(struct cts_platform_data *pdata)
{
    int i;

    cts_info("Release all vkeys");

    for (i = 0; i < pdata->vkey_num; i++) {
        if (pdata->vkey_state & BIT(i)) {
            tpd_button(x, y, 0);
        }
    }

    pdata->vkey_state = 0;

    return 0;
}
#endif /* CONFIG_CTS_VIRTUALKEY */

#ifdef CFG_CTS_GESTURE
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  begin
ssize_t cts_double_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", cts_wake_switch);
}

ssize_t cts_double_wake_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rt;
	unsigned long val;

    
    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        cts_dbg("invalid value\n");
        return rt;
    }
	
	cts_wake_switch = val;

	cts_dbg("cts_double_wake_store value : %d\n", __func__, cts_wake_switch);


    return count;
}


ssize_t cts_gesture_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", cts_gesture_switch);
}

ssize_t cts_gesture_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    
	int rt;
	unsigned long val;

    rt = kstrtoul(buf, 10, &val);
    if(rt != 0){
        cts_dbg("invalid value\n");
        return rt;
    }
	
	cts_gesture_switch = val;

    
	cts_dbg("cts_gesture_store value : %d\n", __func__, cts_gesture_switch);


    return count;
}

void cts_set_location_area_point()
{
		cts_gesture_data.f_point.start_x = 60;
		cts_gesture_data.f_point.start_y = 500;
		cts_gesture_data.f_point.end_x = 660;
		cts_gesture_data.f_point.end_y = 1100;
		cts_gesture_data.f_point.width = 600;
		cts_gesture_data.f_point.height = 600;
		cts_gesture_data.f_point.mid_x = 360;
		cts_gesture_data.f_point.mid_y = 800;
		cts_gesture_data.f_point.top_x = 360;
		cts_gesture_data.f_point.top_y = 500;
		cts_gesture_data.f_point.bottom_x = 360;
		cts_gesture_data.f_point.bottom_y = 1100;
		cts_gesture_data.f_point.left_x = 60;
		cts_gesture_data.f_point.left_y = 800;
		cts_gesture_data.f_point.right_x = 660;
		cts_gesture_data.f_point.right_y = 800;
}

ssize_t cts_gesture_buf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count;
    int i = 0;
    int len = 0;
    cts_set_location_area_point();
    count = sizeof(cts_gesture_data.f_point)/sizeof(cts_gesture_data.f_point.data[0]);

    for (i = 0; i < count; i++)
    {
        if(i==count-1)
            len += sprintf(buf+len,"%d",cts_gesture_data.f_point.data[i]);
        else
            len += sprintf(buf+len,"%d,",cts_gesture_data.f_point.data[i]);
    }
    

    return len;
}

ssize_t cts_gesture_buf_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return -EPERM;
}
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  end

int cts_plat_enable_irq_wake(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Enable IRQ wake");

    if (pdata->irq > 0) {
        ret = enable_irq_wake(pdata->irq);
        if (ret < 0) {
            cts_err("Enable irq wake failed");
            return -EINVAL;
        }
        pdata->irq_wake_enabled = true;
        return 0;
    }

    cts_warn("Enable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_disable_irq_wake(struct cts_platform_data *pdata)
{
    int ret;

    cts_info("Disable IRQ wake");

    if (pdata->irq > 0) {
        ret = disable_irq_wake(pdata->irq);
        if (ret < 0) {
            cts_warn("Disable irq wake while already disabled");
            return -EINVAL;
        }
        pdata->irq_wake_enabled = false;
        return 0;
    }

    cts_warn("Disable irq wake while irq invalid %d", pdata->irq);
    return -ENODEV;
}

int cts_plat_init_gesture(struct cts_platform_data *pdata)
{

    cts_info("Init gesture");

    /* TODO: If system will issure enable/disable command, comment following line. */
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  begin
    cts_enable_gesture_wakeup(pdata->cts_dev);


	set_bit(KEY_DOUBLE_CLICK_WAKEUP, pdata->ts_input_dev->keybit);
	set_bit(KEY_GESTURE_V, pdata->ts_input_dev->keybit);
	set_bit(KEY_GESTURE_O, pdata->ts_input_dev->keybit);
	set_bit(KEY_GESTURE_E, pdata->ts_input_dev->keybit);
	set_bit(KEY_GESTURE_M, pdata->ts_input_dev->keybit);
	set_bit(KEY_GESTURE_W, pdata->ts_input_dev->keybit);
	
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_DOUBLE_CLICK_WAKEUP);
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_GESTURE_M);
	input_set_capability(pdata->ts_input_dev, EV_KEY, KEY_GESTURE_W);
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  end

    return 0;
}

void cts_plat_deinit_gesture(struct cts_platform_data *pdata)
{
    cts_info("De-init gesture");
}

int cts_plat_process_gesture_info(struct cts_platform_data *pdata,
        struct cts_device_gesture_info *gesture_info)
{
    int gesture;

    cts_info("Process gesture, id=0x%02x", gesture_info->gesture_id);

//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  begin
#if defined(CFG_CTS_GESTURE_REPORT_KEY)
    
    if (gesture_info->gesture_id == CTS_GESTURE_D_TAP && cts_wake_switch == 1 ) {
        input_report_key(pdata->ts_input_dev, KEY_DOUBLE_CLICK_WAKEUP, 1);
		input_sync(pdata->ts_input_dev);
		input_report_key(pdata->ts_input_dev, KEY_DOUBLE_CLICK_WAKEUP, 0);
		input_sync(pdata->ts_input_dev);
        
        return 0;
    }

    if (cts_gesture_switch == 1) {
        switch (gesture_info->gesture_id) {
            case CTS_GESTURE_V :
                gesture = KEY_GESTURE_V;
                break;
            case CTS_GESTURE_O :
                gesture = KEY_GESTURE_O;
                break;
            case CTS_GESTURE_E :
                gesture = KEY_GESTURE_E;
                break;
            case CTS_GESTURE_M :
                gesture = KEY_GESTURE_M;
                break;
            case CTS_GESTURE_W:
                gesture = KEY_GESTURE_W;
                break;
            default:
                gesture = -1;
                break;
        }
    }

    /* report event key */
    if (gesture != -1) {
        cts_info("Report key = %d", gesture);
        input_report_key(pdata->ts_input_dev, gesture, 1);
        input_sync(pdata->ts_input_dev);
        input_report_key(pdata->ts_input_dev, gesture, 0);
        input_sync(pdata->ts_input_dev);

        return 0;
    }
//Antai <AI_BSP_TP> <chenht> <2023-02-16> add gesture mode for 2206  end
#endif /* CFG_CTS_GESTURE_REPORT_KEY */

    cts_warn("Process unrecognized gesture id=%u", gesture_info->gesture_id);

    return -EINVAL;
}

#endif /* CFG_CTS_GESTURE */
