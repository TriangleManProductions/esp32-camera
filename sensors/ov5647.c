///
///
///
///
///

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "xclk.h"
#include "OV5647.h"
#include "OV5647_regs.h"
#include "OV5647_settings.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "OV5647";
#endif

/// @brief Single byte read.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits)
/// @return Byte value read
static uint8_t ov5647_read_reg(uint8_t dev_addr, uint16_t reg_addr)
{
    return SCCB_Read16(dev_addr, reg_addr);
}

/// @brief
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits), typically from ov5647_reg_t
/// @param mask
/// @return
static int ov5647_check_reg_mask(uint8_t dev_addr, uint16_t reg_addr, uint8_t mask)
{
    return (ov5647_read_reg(dev_addr, reg_addr) & mask) == mask;
}

/// @brief Two byte read.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits), typically from ov5647_reg_t
/// @return 2 byte value read
static uint16_t ov5647_read_reg16(uint8_t dev_addr, uint16_t reg_addr)
{
    int ret = 0, ret2 = 0;
    ret = ov5647_read_reg(dev_addr, reg_addr);
    if (ret >= 0)
    {
        ret = (ret & 0xFF) << 8;
        ret2 = ov5647_read_reg(dev_addr, reg_addr + 1);
        if (ret2 < 0)
        {
            ret = ret2;
        }
        else
        {
            ret |= ret2 & 0xFF;
        }
    }
    return ret;
}

/// @brief Write 8-bit value to register.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits), typically from ov5647_reg_t
/// @param value Byte to write
/// @return 0 on success.
static int ov5647_write_reg(uint8_t dev_addr, uint16_t reg_addr, uint8_t value)
{
    return SCCB_Write16(dev_addr, reg_addr, value);
}

/// @brief Write partial values to register.
/// This overlays bitmasked values onto the current register value.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits), typically from ov5647_reg_t
/// @param offset Bit offset
/// @param mask Bitmask
/// @param value
/// @return 0 on success.
static int ov5647_set_reg_bits(uint8_t dev_addr, uint16_t reg_addr, uint8_t offset, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t c_value, new_value;
    ret = ov5647_read_reg(dev_addr, reg_addr);
    if (ret < 0)
    {
        return ret;
    }
    c_value = ret;
    new_value = (c_value & ~(mask << offset)) | ((value & mask) << offset);
    ret = ov5647_write_reg(dev_addr, reg_addr, new_value);
    return ret;
}

/// @brief Bulk register write.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param regs Array of register-value pairs, terminated with OV5647_REG_TERM
/// @return 0 on success.
static int ov5647_write_regs(uint8_t dev_addr, ov5647_regval_t *regs, int array_size)
{
    int ret = 0;
    for (int i = 0; i < array_size; i++)
    {
        if (regs[i]->addr == OV5647_REG_TERM)
        {
            break;
        }
        if (regs[i]->addr == OV5647_REG_DLY)
        {
            vTaskDelay(regs[i]->data / portTICK_PERIOD_MS);
            continue;
        }
        ret = ov5647_write_reg(dev_addr, regs[i]->addr, regs[i]->data);
    }
    return ret;
}

/// @brief 2 byte (16-bit) write.
/// @param dev_addr Device address (== 0x36 for OV5647)
/// @param reg_addr Register address (2-bits), typically from ov5647_reg_t
/// The write starts at this value and passes into the subsequent one.
/// @param value 16-bit value to write
/// @return 0 on success
static int ov5647_write_reg16(uint8_t dev_addr, uint16_t reg_addr, uint16_t value)
{
    if (ov5647_write_reg(dev_addr, reg_addr, value >> 8) || ov5647_write_reg(dev_addr, reg_addr + 1, value))
    {
        return -1;
    }
    return 0;
}

/// @brief
/// @param dev_addr
/// @param reg_addr
/// @param x_value
/// @param y_value
/// @return
static int ov5647_write_addr_reg(uint8_t dev_addr, uint16_t reg_addr, uint16_t x_value, uint16_t y_value)
{
    if (ov5647_write_reg16(dev_addr, reg_addr, x_value) || ov5647_write_reg16(dev_addr, reg_addr + 2, y_value))
    {
        return -1;
    }
    return 0;
}

/// @brief
#define ov5647_write_reg_bits(dev_addr, reg_addr, mask, enable) ov5647_set_reg_bits(dev_addr, reg_addr, 0, mask, (enable) ? (mask) : 0)

/// @brief
/// @param xclk
/// @param pll_bypass
/// @param pll_multiplier
/// @param pll_sys_div
/// @param pre_div
/// @param root_2x
/// @param pclk_root_div
/// @param pclk_manual
/// @param pclk_div
/// @return
static int calc_sysclk(int xclk, bool pll_bypass, int pll_multiplier, int pll_sys_div, int pre_div, bool root_2x, int pclk_root_div, bool pclk_manual, int pclk_div)
{
    const float pll_pre_div2x_map[] = {1, 1, 2, 3, 4, 1.5, 6, 2.5, 8};
    const int pll_pclk_root_div_map[] = {1, 2, 4, 8};

    if (!pll_sys_div)
    {
        pll_sys_div = 1;
    }

    float pll_pre_div = pll_pre_div2x_map[pre_div];
    unsigned int root_2x_div = root_2x ? 2 : 1;
    unsigned int pll_pclk_root_div = pll_pclk_root_div_map[pclk_root_div];

    unsigned int REFIN = xclk / pll_pre_div;

    unsigned int VCO = REFIN * pll_multiplier / root_2x_div;

    unsigned int PLL_CLK = pll_bypass ? (xclk) : (VCO / pll_sys_div * 2 / 5); // 5 here is 10bit mode / 2, for 8bit it should be 4 (reg_addr 0x3034)

    unsigned int PCLK = PLL_CLK / pll_pclk_root_div / ((pclk_manual && pclk_div) ? pclk_div : 2);

    unsigned int SYSCLK = PLL_CLK / 4;

    ESP_LOGI(TAG, "Calculated XVCLK: %d Hz, REFIN: %u Hz, VCO: %u Hz, PLL_CLK: %u Hz, SYSCLK: %u Hz, PCLK: %u Hz", xclk, REFIN, VCO, PLL_CLK, SYSCLK, PCLK);
    return SYSCLK;
}

/// @brief
/// @param sensor
/// @param bypass
/// @param multiplier
/// @param sys_div
/// @param pre_div
/// @param root_2x
/// @param pclk_root_div
/// @param pclk_manual
/// @param pclk_div
/// @return
static int ov5647_set_pll(sensor_t *sensor, bool bypass, uint8_t multiplier, uint8_t sys_div, uint8_t pre_div, bool root_2x, uint8_t pclk_root_div, bool pclk_manual, uint8_t pclk_div)
{
    int ret = 0;
    if (multiplier > 252 || multiplier < 4 || sys_div > 15 || pre_div > 8 || pclk_div > 31 || pclk_root_div > 3)
    {
        ESP_LOGE(TAG, "Invalid arguments");
        return -1;
    }
    if (multiplier > 127)
    {
        multiplier &= 0xFE; // only even integers above 127
    }
    ESP_LOGI(TAG, "Set PLL: bypass: %u, multiplier: %u, sys_div: %u, pre_div: %u, root_2x: %u, pclk_root_div: %u, pclk_manual: %u, pclk_div: %u", bypass, multiplier, sys_div, pre_div, root_2x, pclk_root_div, pclk_manual, pclk_div);

    calc_sysclk(sensor->xclk_freq_hz, bypass, multiplier, sys_div, pre_div, root_2x, pclk_root_div, pclk_manual, pclk_div);

    ret = ov5647_write_reg(sensor->dev_addr, 0x3039, bypass ? 0x80 : 0x00);
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3034, 0x1A); // 10bit mode
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3035, 0x01 | ((sys_div & 0x0f) << 4));
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3036, multiplier & 0xff);
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3037, (pre_div & 0xf) | (root_2x ? 0x10 : 0x00));
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3108, (pclk_root_div & 0x3) << 4 | 0x06);
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3824, pclk_div & 0x1f);
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x460C, pclk_manual ? 0x22 : 0x20);
    }
    if (ret == 0)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x3103, 0x13); // system clock from pll, bit[1]
    }
    if (ret)
    {
        ESP_LOGE(TAG, "set_sensor_pll FAILED!");
    }
    return ret;
}

static int ov5647_set_ae_level(sensor_t *sensor, int level);

static int ov5647_reset(sensor_t *sensor)
{
    // dump_regs(sensor);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    int ret = 0;
    // Software Reset: clear all registers and reset them to their default values
    ret = ov5647_write_reg(sensor->dev_addr, SYSTEM_CTROL0, 0x82);
    if (ret)
    {
        ESP_LOGE(TAG, "Software Reset FAILED!");
        return ret;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ret = ov5647_write_regs(sensor->dev_addr, sensor_default_regs);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Camera defaults loaded");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // write_regs(sensor->dev_addr, sensor_regs_awb0);
        // write_regs(sensor->dev_addr, sensor_regs_gamma1);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param pixformat
/// @return
static int ov5647_set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    int ret = 0;
    ov5647_regval_t *regs;

    switch (pixformat)
    {
    case PIXFORMAT_YUV422:
        regs = ov5647_sensor_fmt_yuv422;
        break;

    case PIXFORMAT_GRAYSCALE:
        regs = ov5647_sensor_fmt_grayscale;
        break;

    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        regs = ov5647_sensor_fmt_rgb565;
        break;

    case PIXFORMAT_JPEG:
        regs = ov5647_sensor_fmt_jpeg;
        break;

    case PIXFORMAT_RAW:
        regs = ov5647_sensor_fmt_raw;
        break;

    default:
        ESP_LOGE(TAG, "Unsupported pixformat: %u", pixformat);
        return -1;
    }

    ret = ov5647_write_regs(sensor->dev_addr, regs);
    if (ret == 0)
    {
        sensor->pixformat = pixformat;
        ESP_LOGD(TAG, "Set pixformat to: %u", pixformat);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @return
static int ov5647_set_image_options(sensor_t *sensor)
{
    int ret = 0;
    uint8_t reg20 = 0;
    uint8_t reg21 = 0;
    uint8_t reg4514 = 0;
    uint8_t reg4514_test = 0;

    // compression
    if (sensor->pixformat == PIXFORMAT_JPEG)
    {
        reg21 |= 0x20;
    }

    // binning
    if (!sensor->status.binning)
    {
        reg20 |= 0x40;
    }
    else
    {
        reg20 |= 0x01;
        reg21 |= 0x01;
        reg4514_test |= 4;
    }

    // V-Flip
    if (sensor->status.vflip)
    {
        reg20 |= 0x06;
        reg4514_test |= 1;
    }

    // H-Mirror
    if (sensor->status.hmirror)
    {
        reg21 |= 0x06;
        reg4514_test |= 2;
    }

    switch (reg4514_test)
    {
    // no binning
    case 0:
        reg4514 = 0x88;
        break; // normal
    case 1:
        reg4514 = 0x00;
        break; // v-flip
    case 2:
        reg4514 = 0xbb;
        break; // h-mirror
    case 3:
        reg4514 = 0x00;
        break; // v-flip+h-mirror
    // binning
    case 4:
        reg4514 = 0xaa;
        break; // normal
    case 5:
        reg4514 = 0xbb;
        break; // v-flip
    case 6:
        reg4514 = 0xbb;
        break; // h-mirror
    case 7:
        reg4514 = 0xaa;
        break; // v-flip+h-mirror
    }

    if (write_reg(sensor->dev_addr, TIMING_TC_REG20, reg20) || ov5647_write_reg(sensor->dev_addr, TIMING_TC_REG21, reg21) || ov5647_write_reg(sensor->dev_addr, 0x4514, reg4514))
    {
        ESP_LOGE(TAG, "Setting Image Options Failed");
        return -1;
    }

    if (!sensor->status.binning)
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x4520, 0x10) || ov5647_write_reg(sensor->dev_addr, X_INCREMENT, 0x11) // odd:1, even: 1
              || ov5647_write_reg(sensor->dev_addr, Y_INCREMENT, 0x11);                                                 // odd:1, even: 1
    }
    else
    {
        ret = ov5647_write_reg(sensor->dev_addr, 0x4520, 0x0b) || ov5647_write_reg(sensor->dev_addr, X_INCREMENT, 0x31) // odd:3, even: 1
              || ov5647_write_reg(sensor->dev_addr, Y_INCREMENT, 0x31);                                                 // odd:3, even: 1
    }

    ESP_LOGD(TAG, "Set Image Options: Compression: %u, Binning: %u, V-Flip: %u, H-Mirror: %u, Reg-4514: 0x%02x",
             sensor->pixformat == PIXFORMAT_JPEG, sensor->status.binning, sensor->status.vflip, sensor->status.hmirror, reg4514);
    return ret;
}

/// @brief
/// @param sensor
/// @param framesize
/// @return
static int ov5647_set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret = 0;
    framesize_t old_framesize = sensor->status.framesize;
    sensor->status.framesize = framesize;

    if (framesize > FRAMESIZE_QSXGA)
    {
        ESP_LOGE(TAG, "Invalid framesize: %u", framesize);
        return -1;
    }
    uint16_t w = resolution[framesize].width;
    uint16_t h = resolution[framesize].height;
    aspect_ratio_t ratio = resolution[framesize].aspect_ratio;
    ratio_settings_t settings = ratio_table[ratio];

    sensor->status.binning = (w <= (settings.max_width / 2) && h <= (settings.max_height / 2));
    sensor->status.scale = !((w == settings.max_width && h == settings.max_height) || (w == (settings.max_width / 2) && h == (settings.max_height / 2)));

    ret = ov5647_write_addr_reg(sensor->dev_addr, X_ADDR_ST_H, settings.start_x, settings.start_y) || ov5647_write_addr_reg(sensor->dev_addr, X_ADDR_END_H, settings.end_x, settings.end_y) || ov5647_write_addr_reg(sensor->dev_addr, X_OUTPUT_SIZE_H, w, h);

    if (ret)
    {
        goto fail;
    }

    if (!sensor->status.binning)
    {
        ret = ov5647_write_addr_reg(sensor->dev_addr, X_TOTAL_SIZE_H, settings.total_x, settings.total_y) || ov5647_write_addr_reg(sensor->dev_addr, X_OFFSET_H, settings.offset_x, settings.offset_y);
    }
    else
    {
        if (w > 920)
        {
            ret = ov5647_write_addr_reg(sensor->dev_addr, X_TOTAL_SIZE_H, settings.total_x - 200, settings.total_y / 2);
        }
        else
        {
            ret = ov5647_write_addr_reg(sensor->dev_addr, X_TOTAL_SIZE_H, 2060, settings.total_y / 2);
        }
        if (ret == 0)
        {
            ret = ov5647_write_addr_reg(sensor->dev_addr, X_OFFSET_H, settings.offset_x / 2, settings.offset_y / 2);
        }
    }

    if (ret == 0)
    {
        ret = ov5647_write_reg_bits(sensor->dev_addr, ISP_CONTROL_01, 0x20, sensor->status.scale);
    }

    if (ret == 0)
    {
        ret = ov5647_set_image_options(sensor);
    }

    if (ret)
    {
        goto fail;
    }

    if (sensor->pixformat == PIXFORMAT_JPEG)
    {
        // 10MHz PCLK
        uint8_t sys_mul = 200;
        if (framesize < FRAMESIZE_QVGA || sensor->xclk_freq_hz == 16000000)
        {
            sys_mul = 160;
        }
        else if (framesize < FRAMESIZE_XGA)
        {
            sys_mul = 180;
        }
        ret = ov5647_set_pll(sensor, false, sys_mul, 4, 2, false, 2, true, 4);
        // Set PLL: bypass: 0, multiplier: sys_mul, sys_div: 4, pre_div: 2, root_2x: 0, pclk_root_div: 2, pclk_manual: 1, pclk_div: 4
    }
    else
    {
        // ret = ov5647_set_pll(sensor, false, 8, 1, 1, false, 1, true, 4);
        if (framesize > FRAMESIZE_HVGA)
        {
            ret = ov5647_set_pll(sensor, false, 10, 1, 2, false, 1, true, 2);
        }
        else if (framesize >= FRAMESIZE_QVGA)
        {
            ret = ov5647_set_pll(sensor, false, 8, 1, 1, false, 1, true, 4);
        }
        else
        {
            ret = ov5647_set_pll(sensor, false, 20, 1, 1, false, 1, true, 8);
        }
    }

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set framesize to: %ux%u", w, h);
    }
    return ret;

fail:
    sensor->status.framesize = old_framesize;
    ESP_LOGE(TAG, "Setting framesize to: %ux%u failed", w, h);
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_hmirror(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.hmirror = enable;
    ret = ov5647_set_image_options(sensor);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set h-mirror to: %d", enable);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_vflip(sensor_t *sensor, int enable)
{
    int ret = 0;
    sensor->status.vflip = enable;
    ret = ov5647_set_image_options(sensor);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set v-flip to: %d", enable);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param qs
/// @return
static int ov5647_set_quality(sensor_t *sensor, int qs)
{
    int ret = 0;
    ret = ov5647_write_reg(sensor->dev_addr, COMPRESSION_CTRL07, qs & 0x3f);
    if (ret == 0)
    {
        sensor->status.quality = qs;
        ESP_LOGD(TAG, "Set quality to: %d", qs);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_colorbar(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR, enable);
    if (ret == 0)
    {
        sensor->status.colorbar = enable;
        ESP_LOGD(TAG, "Set colorbar to: %d", enable);
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_gain_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN, !enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set gain_ctrl to: %d", enable);
        sensor->status.agc = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_exposure_ctrl(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN, !enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set exposure_ctrl to: %d", enable);
        sensor->status.aec = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_whitebal(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, ISP_CONTROL_01, 0x01, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set awb to: %d", enable);
        sensor->status.awb = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_dcw_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5183, 0x80, !enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set dcw to: %d", enable);
        sensor->status.dcw = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_aec2(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x3a00, 0x04, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set aec2 to: %d", enable);
        sensor->status.aec2 = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_bpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5000, 0x04, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set bpc to: %d", enable);
        sensor->status.bpc = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_wpc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5000, 0x02, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set wpc to: %d", enable);
        sensor->status.wpc = enable;
    }
    return ret;
}

/// @brief
/// @param sensor
/// @param enable
/// @return
static int ov5647_set_raw_gma_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5000, 0x20, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set raw_gma to: %d", enable);
        sensor->status.raw_gma = enable;
    }
    return ret;
}

static int ov5647_set_lenc_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5000, 0x80, enable);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set lenc to: %d", enable);
        sensor->status.lenc = enable;
    }
    return ret;
}

static int get_agc_gain(sensor_t *sensor)
{
    int ra = ov5647_read_reg(sensor->dev_addr, 0x350a);
    if (ra < 0)
    {
        return 0;
    }
    int rb = ov5647_read_reg(sensor->dev_addr, 0x350b);
    if (rb < 0)
    {
        return 0;
    }
    int res = (rb & 0xF0) >> 4 | (ra & 0x03) << 4;
    if (rb & 0x0F)
    {
        res += 1;
    }
    return res;
}

// real gain
static int ov5647_set_agc_gain(sensor_t *sensor, int gain)
{
    int ret = 0;
    if (gain < 0)
    {
        gain = 0;
    }
    else if (gain > 64)
    {
        gain = 64;
    }

    // gain value is 6.4 bits float
    // in order to use the max range, we deduct 1/16
    int gainv = gain << 4;
    if (gainv)
    {
        gainv -= 1;
    }

    ret = ov5647_write_reg(sensor->dev_addr, 0x350a, gainv >> 8) || ov5647_write_reg(sensor->dev_addr, 0x350b, gainv & 0xff);
    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set agc_gain to: %d", gain);
        sensor->status.agc_gain = gain;
    }
    return ret;
}

static int get_aec_value(sensor_t *sensor)
{
    int ra = ov5647_read_reg(sensor->dev_addr, 0x3500);
    if (ra < 0)
    {
        return 0;
    }
    int rb = ov5647_read_reg(sensor->dev_addr, 0x3501);
    if (rb < 0)
    {
        return 0;
    }
    int rc = ov5647_read_reg(sensor->dev_addr, 0x3502);
    if (rc < 0)
    {
        return 0;
    }
    int res = (ra & 0x0F) << 12 | (rb & 0xFF) << 4 | (rc & 0xF0) >> 4;
    return res;
}

static int ov5647_set_aec_value(sensor_t *sensor, int value)
{
    int ret = 0, max_val = 0;
    max_val = ov5647_read_reg16(sensor->dev_addr, 0x380e);
    if (max_val < 0)
    {
        ESP_LOGE(TAG, "Could not read max aec_value");
        return -1;
    }
    if (value > max_val)
    {
        value = max_val;
    }

    ret = ov5647_write_reg(sensor->dev_addr, 0x3500, (value >> 12) & 0x0F) || ov5647_write_reg(sensor->dev_addr, 0x3501, (value >> 4) & 0xFF) || ov5647_write_reg(sensor->dev_addr, 0x3502, (value << 4) & 0xF0);

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set aec_value to: %d / %d", value, max_val);
        sensor->status.aec_value = value;
    }
    return ret;
}

static int ov5647_set_ae_level(sensor_t *sensor, int level)
{
    int ret = 0;
    if (level < -5 || level > 5)
    {
        return -1;
    }
    // good targets are between 5 and 115
    int target_level = ((level + 5) * 10) + 5;

    int level_high, level_low;
    int fast_high, fast_low;

    level_low = target_level * 23 / 25;  // 0.92 (0.46)
    level_high = target_level * 27 / 25; // 1.08 (2.08)

    fast_low = level_low >> 1;
    fast_high = level_high << 1;

    if (fast_high > 255)
    {
        fast_high = 255;
    }

    ret = ov5647_write_reg(sensor->dev_addr, 0x3a0f, level_high) || ov5647_write_reg(sensor->dev_addr, 0x3a10, level_low) || ov5647_write_reg(sensor->dev_addr, 0x3a1b, level_high) || ov5647_write_reg(sensor->dev_addr, 0x3a1e, level_low) || ov5647_write_reg(sensor->dev_addr, 0x3a11, fast_high) || ov5647_write_reg(sensor->dev_addr, 0x3a1f, fast_low);

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set ae_level to: %d", level);
        sensor->status.ae_level = level;
    }
    return ret;
}

static int ov5647_set_wb_mode(sensor_t *sensor, int mode)
{
    int ret = 0;
    if (mode < 0 || mode > 4)
    {
        return -1;
    }

    ret = ov5647_write_reg(sensor->dev_addr, 0x3406, (mode != 0));
    if (ret)
    {
        return ret;
    }
    switch (mode)
    {
    case 1:                                                           // Sunny
        ret = ov5647_write_reg16(sensor->dev_addr, 0x3400, 0x5e0)     // AWB R GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3402, 0x410)  // AWB G GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3404, 0x540); // AWB B GAIN
        break;
    case 2:                                                           // Cloudy
        ret = ov5647_write_reg16(sensor->dev_addr, 0x3400, 0x650)     // AWB R GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3402, 0x410)  // AWB G GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3404, 0x4f0); // AWB B GAIN
        break;
    case 3:                                                           // Office
        ret = ov5647_write_reg16(sensor->dev_addr, 0x3400, 0x520)     // AWB R GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3402, 0x410)  // AWB G GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3404, 0x660); // AWB B GAIN
        break;
    case 4:                                                           // HOME
        ret = ov5647_write_reg16(sensor->dev_addr, 0x3400, 0x420)     // AWB R GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3402, 0x3f0)  // AWB G GAIN
              || ov5647_write_reg16(sensor->dev_addr, 0x3404, 0x710); // AWB B GAIN
        break;
    default: // AUTO
        break;
    }

    if (ret == 0)
    {
        ESP_LOGD(TAG, "Set wb_mode to: %d", mode);
        sensor->status.wb_mode = mode;
    }
    return ret;
}

static int ov5647_set_awb_gain_dsp(sensor_t *sensor, int enable)
{
    int ret = 0;
    int old_mode = sensor->status.wb_mode;
    int mode = enable ? old_mode : 0;

    ret = ov5647_set_wb_mode(sensor, mode);

    if (ret == 0)
    {
        sensor->status.wb_mode = old_mode;
        ESP_LOGD(TAG, "Set awb_gain to: %d", enable);
        sensor->status.awb_gain = enable;
    }
    return ret;
}

static int ov5647_set_special_effect(sensor_t *sensor, int effect)
{
    int ret = 0;
    // if (effect < 0 || effect > 6)
    // {
    //     return -1;
    // }

    // uint8_t *regs = (uint8_t *)sensor_special_effects[effect];
    // ret = ov5647_write_reg(sensor->dev_addr, 0x5580, regs[0]) || ov5647_write_reg(sensor->dev_addr, 0x5583, regs[1]) || ov5647_write_reg(sensor->dev_addr, 0x5584, regs[2]) || ov5647_write_reg(sensor->dev_addr, 0x5003, regs[3]);

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set special_effect to: %d", effect);
    //     sensor->status.special_effect = effect;
    // }
    return ret;
}

static int ov5647_set_brightness(sensor_t *sensor, int level)
{
    int ret = 0;
    // uint8_t value = 0;
    // bool negative = false;

    // switch (level)
    // {
    // case 3:
    //     value = 0x30;
    //     break;
    // case 2:
    //     value = 0x20;
    //     break;
    // case 1:
    //     value = 0x10;
    //     break;
    // case -1:
    //     value = 0x10;
    //     negative = true;
    //     break;
    // case -2:
    //     value = 0x20;
    //     negative = true;
    //     break;
    // case -3:
    //     value = 0x30;
    //     negative = true;
    //     break;
    // default: // 0
    //     break;
    // }

    // ret = ov5647_write_reg(sensor->dev_addr, 0x5587, value);
    // if (ret == 0)
    // {
    //     ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5588, 0x08, negative);
    // }

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set brightness to: %d", level);
    //     sensor->status.brightness = level;
    // }
    return ret;
}

static int ov5647_set_contrast(sensor_t *sensor, int level)
{
    int ret = 0;
    // if (level > 3 || level < -3)
    // {
    //     return -1;
    // }
    // ret = ov5647_write_reg(sensor->dev_addr, 0x5586, (level + 4) << 3);

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set contrast to: %d", level);
    //     sensor->status.contrast = level;
    // }
    return ret;
}

static int ov5647_set_saturation(sensor_t *sensor, int level)
{
    int ret = 0;
    // if (level > 4 || level < -4)
    // {
    //     return -1;
    // }

    // uint8_t *regs = (uint8_t *)sensor_saturation_levels[level + 4];
    // for (int i = 0; i < 11; i++)
    // {
    //     ret = ov5647_write_reg(sensor->dev_addr, 0x5381 + i, regs[i]);
    //     if (ret)
    //     {
    //         break;
    //     }
    // }

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set saturation to: %d", level);
    //     sensor->status.saturation = level;
    // }
    return ret;
}

static int ov5647_set_sharpness(sensor_t *sensor, int level)
{
    int ret = 0;
    // if (level > 3 || level < -3)
    // {
    //     return -1;
    // }

    // uint8_t mt_offset_2 = (level + 3) * 8;
    // uint8_t mt_offset_1 = mt_offset_2 + 1;

    // ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5308, 0x40, false) // 0x40 means auto
    //       || ov5647_write_reg(sensor->dev_addr, 0x5300, 0x10) || ov5647_write_reg(sensor->dev_addr, 0x5301, 0x10) || ov5647_write_reg(sensor->dev_addr, 0x5302, mt_offset_1) || ov5647_write_reg(sensor->dev_addr, 0x5303, mt_offset_2) || ov5647_write_reg(sensor->dev_addr, 0x5309, 0x10) || ov5647_write_reg(sensor->dev_addr, 0x530a, 0x10) || ov5647_write_reg(sensor->dev_addr, 0x530b, 0x04) || ov5647_write_reg(sensor->dev_addr, 0x530c, 0x06);

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set sharpness to: %d", level);
    //     sensor->status.sharpness = level;
    // }
    return ret;
}

static int ov5647_set_gainceiling(sensor_t *sensor, gainceiling_t level)
{
    int ret = 0, l = (int)level;

    // ret = ov5647_write_reg(sensor->dev_addr, 0x3A18, (l >> 8) & 3) || ov5647_write_reg(sensor->dev_addr, 0x3A19, l & 0xFF);

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set gainceiling to: %d", l);
    //     sensor->status.gainceiling = l;
    // }
    return ret;
}

static int get_denoise(sensor_t *sensor)
{
    // if (!ov5647_check_reg_mask(sensor->dev_addr, 0x5308, 0x10))
    // {
    //     return 0;
    // }
    // return (read_reg(sensor->dev_addr, 0x5306) / 4) + 1;
    return 0;
}

static int ov5647_set_denoise(sensor_t *sensor, int level)
{
    int ret = 0;
    // if (level < 0 || level > 8)
    // {
    //     return -1;
    // }

    // ret = ov5647_write_reg_bits(sensor->dev_addr, 0x5308, 0x10, level > 0);
    // if (ret == 0 && level > 0)
    // {
    //     ret = ov5647_write_reg(sensor->dev_addr, 0x5306, (level - 1) * 4);
    // }

    // if (ret == 0)
    // {
    //     ESP_LOGD(TAG, "Set denoise to: %d", level);
    //     sensor->status.denoise = level;
    // }
    return ret;
}

static int ov5647_get_reg(sensor_t *sensor, int reg_addr, int mask)
{
    int ret = 0, ret2 = 0;
    // if (mask > 0xFF)
    // {
    //     ret = ov5647_read_reg16(sensor->dev_addr, reg_addr);
    //     if (ret >= 0 && mask > 0xFFFF)
    //     {
    //         ret2 = ov5647_read_reg(sensor->dev_addr, reg_addr + 2);
    //         if (ret2 >= 0)
    //         {
    //             ret = (ret << 8) | ret2;
    //         }
    //         else
    //         {
    //             ret = ret2;
    //         }
    //     }
    // }
    // else
    // {
    //     ret = ov5647_read_reg(sensor->dev_addr, reg_addr);
    // }
    // if (ret > 0)
    // {
    //     ret &= mask;
    // }
    return ret;
}

static int ov5647_set_reg(sensor_t *sensor, int reg_addr, int mask, int value)
{
    int ret = 0, ret2 = 0;
    // if (mask > 0xFF)
    // {
    //     ret = ov5647_read_reg16(sensor->dev_addr, reg_addr);
    //     if (ret >= 0 && mask > 0xFFFF)
    //     {
    //         ret2 = ov5647_read_reg(sensor->dev_addr, reg_addr + 2);
    //         if (ret2 >= 0)
    //         {
    //             ret = (ret << 8) | ret2;
    //         }
    //         else
    //         {
    //             ret = ret2;
    //         }
    //     }
    // }
    // else
    // {
    //     ret = ov5647_read_reg(sensor->dev_addr, reg_addr);
    // }
    // if (ret < 0)
    // {
    //     return ret;
    // }
    // value = (ret & ~mask) | (value & mask);
    // if (mask > 0xFFFF)
    // {
    //     ret = ov5647_write_reg16(sensor->dev_addr, reg_addr, value >> 8);
    //     if (ret >= 0)
    //     {
    //         ret = ov5647_write_reg(sensor->dev_addr, reg_addr + 2, value & 0xFF);
    //     }
    // }
    // else if (mask > 0xFF)
    // {
    //     ret = ov5647_write_reg16(sensor->dev_addr, reg_addr, value);
    // }
    // else
    // {
    //     ret = ov5647_write_reg(sensor->dev_addr, reg_addr, value);
    // }
    return ret;
}

static int ov5647_set_res_raw(sensor_t *sensor, int startX, int startY, int endX, int endY, int offsetX, int offsetY, int totalX, int totalY, int outputX, int outputY, bool scale, bool binning)
{
    int ret = 0;
    // ret = ov5647_write_addr_reg(sensor->dev_addr, X_ADDR_ST_H, startX, startY) || ov5647_write_addr_reg(sensor->dev_addr, X_ADDR_END_H, endX, endY) || ov5647_write_addr_reg(sensor->dev_addr, X_OFFSET_H, offsetX, offsetY) || ov5647_write_addr_reg(sensor->dev_addr, X_TOTAL_SIZE_H, totalX, totalY) || ov5647_write_addr_reg(sensor->dev_addr, X_OUTPUT_SIZE_H, outputX, outputY) || ov5647_write_reg_bits(sensor->dev_addr, ISP_CONTROL_01, 0x20, scale);
    // if (!ret)
    // {
    //     sensor->status.scale = scale;
    //     sensor->status.binning = binning;
    //     ret = ov5647_set_image_options(sensor);
    // }
    return ret;
}

static int _ov5647_set_pll(sensor_t *sensor, int bypass, int multiplier, int sys_div, int root_2x, int pre_div, int seld5, int pclk_manual, int pclk_div)
{
    int ret = 0;
    // ret = ov5647_set_pll(sensor, bypass > 0, multiplier, sys_div, pre_div, root_2x > 0, seld5, pclk_manual > 0, pclk_div);
    return ret;
}

static int ov5647_set_xclk(sensor_t *sensor, int timer, int xclk)
{
    int ret = 0;
    // sensor->xclk_freq_hz = xclk * 1000000U;
    // ret = xclk_timer_conf(timer, sensor->xclk_freq_hz);
    return ret;
}

static int init_status(sensor_t *sensor)
{
    // sensor->status.brightness = 0;
    // sensor->status.contrast = 0;
    // sensor->status.saturation = 0;
    // sensor->status.sharpness = (read_reg(sensor->dev_addr, 0x5303) / 8) - 3;
    // sensor->status.denoise = get_denoise(sensor);
    // sensor->status.ae_level = 0;
    // sensor->status.gainceiling = ov5647_read_reg16(sensor->dev_addr, 0x3A18) & 0x3FF;
    // sensor->status.awb = ov5647_check_reg_mask(sensor->dev_addr, ISP_CONTROL_01, 0x01);
    // sensor->status.dcw = !ov5647_check_reg_mask(sensor->dev_addr, 0x5183, 0x80);
    // sensor->status.agc = !ov5647_check_reg_mask(sensor->dev_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AGC_MANUALEN);
    // sensor->status.aec = !ov5647_check_reg_mask(sensor->dev_addr, AEC_PK_MANUAL, AEC_PK_MANUAL_AEC_MANUALEN);
    // sensor->status.hmirror = ov5647_check_reg_mask(sensor->dev_addr, TIMING_TC_REG21, TIMING_TC_REG21_HMIRROR);
    // sensor->status.vflip = ov5647_check_reg_mask(sensor->dev_addr, TIMING_TC_REG20, TIMING_TC_REG20_VFLIP);
    // sensor->status.colorbar = ov5647_check_reg_mask(sensor->dev_addr, PRE_ISP_TEST_SETTING_1, TEST_COLOR_BAR);
    // sensor->status.bpc = ov5647_check_reg_mask(sensor->dev_addr, 0x5000, 0x04);
    // sensor->status.wpc = ov5647_check_reg_mask(sensor->dev_addr, 0x5000, 0x02);
    // sensor->status.raw_gma = ov5647_check_reg_mask(sensor->dev_addr, 0x5000, 0x20);
    // sensor->status.lenc = ov5647_check_reg_mask(sensor->dev_addr, 0x5000, 0x80);
    // sensor->status.quality = ov5647_read_reg(sensor->dev_addr, COMPRESSION_CTRL07) & 0x3f;
    // sensor->status.special_effect = 0;
    // sensor->status.wb_mode = 0;
    // sensor->status.awb_gain = ov5647_check_reg_mask(sensor->dev_addr, 0x3406, 0x01);
    // sensor->status.agc_gain = get_agc_gain(sensor);
    // sensor->status.aec_value = get_aec_value(sensor);
    // sensor->status.aec2 = ov5647_check_reg_mask(sensor->dev_addr, 0x3a00, 0x04);
    return 0;
}

int OV5647_detect(int dev_addr, sensor_id_t *id)
{
    return 0;
}

int OV5647_init(sensor_t *sensor)
{
    sensor->reset = ov5647_reset;
    sensor->set_pixformat = ov5647_set_pixformat;
    sensor->set_framesize = ov5647_set_framesize;
    sensor->set_contrast = ov5647_set_contrast;
    sensor->set_brightness = ov5647_set_brightness;
    sensor->set_saturation = ov5647_set_saturation;
    sensor->set_sharpness = ov5647_set_sharpness;
    sensor->set_gainceiling = ov5647_set_gainceiling;
    sensor->set_quality = ov5647_set_quality;
    sensor->set_colorbar = ov5647_set_colorbar;
    sensor->set_gain_ctrl = ov5647_set_gain_ctrl;
    sensor->set_exposure_ctrl = ov5647_set_exposure_ctrl;
    sensor->set_whitebal = ov5647_set_whitebal;
    sensor->set_hmirror = ov5647_set_hmirror;
    sensor->set_vflip = ov5647_set_vflip;
    sensor->init_status = init_status;
    sensor->set_aec2 = ov5647_set_aec2;
    sensor->set_aec_value = ov5647_set_aec_value;
    sensor->set_special_effect = ov5647_set_special_effect;
    sensor->set_wb_mode = ov5647_set_wb_mode;
    sensor->set_ae_level = ov5647_set_ae_level;
    sensor->set_dcw = ov5647_set_dcw_dsp;
    sensor->set_bpc = ov5647_set_bpc_dsp;
    sensor->set_wpc = ov5647_set_wpc_dsp;
    sensor->set_awb_gain = ov5647_set_awb_gain_dsp;
    sensor->set_agc_gain = ov5647_set_agc_gain;
    sensor->set_raw_gma = ov5647_set_raw_gma_dsp;
    sensor->set_lenc = ov5647_set_lenc_dsp;
    sensor->set_denoise = ov5647_set_denoise;
    sensor->get_reg = ov5647_get_reg;
    sensor->set_reg = ov5647_set_reg;
    sensor->set_res_raw = ov5647_set_res_raw;
    sensor->set_pll = _ov5647_set_pll;
    sensor->set_xclk = ov5647_set_xclk;
    return 0;
}
