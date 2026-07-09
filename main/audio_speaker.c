/*
 * Audio speaker driver for ESP32-P4-WIFI6-DEV-KIT-A
 *
 * Hardware: ES8311 mono codec + NS4150B power amplifier.
 * I2S0  : GPIO13(MCLK), GPIO12(BCLK), GPIO10(WS), GPIO9(DOUT)
 * I2C   : GPIO7(SDA), GPIO8(SCL)  (shared with camera SCCB)
 * PA_EN : GPIO53
 */

#include "audio_speaker.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

static const char *TAG = "audio_speaker";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static i2s_chan_handle_t s_tx_chan = NULL;
static esp_codec_dev_handle_t s_codec_dev = NULL;
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static bool s_i2c_bus_created_by_us = false;

/*
 * Extra digital gain applied to all playback data.
 * 1.0 = 0 dB (no change), 2.0 = +6 dB, 3.0 = +10 dB, 4.0 = +12 dB.
 * Use this when the codec/PA hardware volume is already near maximum
 * but more loudness is still needed (e.g. outdoor environments).
 */
#define AUDIO_SPEAKER_DIGITAL_GAIN  (1.0f)

static inline int16_t speaker_apply_gain(int16_t sample, float gain)
{
    float v = (float)sample * gain;
    if (v > 32767.0f) {
        return 32767;
    }
    if (v < -32768.0f) {
        return -32768;
    }
    return (int16_t)v;
}

static esp_err_t speaker_i2c_bus_get_or_create(void)
{
    if (s_i2c_bus != NULL) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_master_get_bus_handle(CONFIG_AUDIO_SPEAKER_I2C_PORT, &s_i2c_bus);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Reusing existing I2C bus on port %d", CONFIG_AUDIO_SPEAKER_I2C_PORT);
        s_i2c_bus_created_by_us = false;
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CONFIG_AUDIO_SPEAKER_I2C_PORT,
        .sda_io_num = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        s_i2c_bus = NULL;
        return ret;
    }

    s_i2c_bus_created_by_us = true;
    ESP_LOGI(TAG, "Created I2C bus on port %d", CONFIG_AUDIO_SPEAKER_I2C_PORT);
    return ESP_OK;
}

static esp_err_t speaker_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S TX channel: %s", esp_err_to_name(ret));
        s_tx_chan = NULL;
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_AUDIO_SPEAKER_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2S_MCLK_GPIO,
            .bclk = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2S_BCLK_GPIO,
            .ws = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2S_WS_GPIO,
            .dout = (gpio_num_t)CONFIG_AUDIO_SPEAKER_I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = CONFIG_AUDIO_SPEAKER_MCLK_MULTIPLE;

    ret = i2s_channel_init_std_mode(s_tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S standard mode: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return ret;
    }

    ret = i2s_channel_enable(s_tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel: %s", esp_err_to_name(ret));
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "I2S0 initialized: MCLK=%d BCLK=%d WS=%d DOUT=%d sample_rate=%d",
             CONFIG_AUDIO_SPEAKER_I2S_MCLK_GPIO,
             CONFIG_AUDIO_SPEAKER_I2S_BCLK_GPIO,
             CONFIG_AUDIO_SPEAKER_I2S_WS_GPIO,
             CONFIG_AUDIO_SPEAKER_I2S_DOUT_GPIO,
             CONFIG_AUDIO_SPEAKER_SAMPLE_RATE);
    return ESP_OK;
}

static esp_err_t speaker_codec_init(void)
{
    ESP_RETURN_ON_ERROR(speaker_i2c_bus_get_or_create(), TAG, "I2C bus unavailable");

    /* I2C control interface for ES8311 */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = CONFIG_AUDIO_SPEAKER_I2C_PORT,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (ctrl_if == NULL) {
        ESP_LOGE(TAG, "Failed to create codec I2C control interface");
        return ESP_FAIL;
    }

    /* GPIO interface (used for PA control) */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (gpio_if == NULL) {
        ESP_LOGE(TAG, "Failed to create codec GPIO interface");
        return ESP_FAIL;
    }

    /* ES8311 codec interface */
    es8311_codec_cfg_t es8311_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = CONFIG_AUDIO_SPEAKER_PA_CTRL_GPIO,
        .use_mclk = true,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    if (codec_if == NULL) {
        ESP_LOGE(TAG, "Failed to create ES8311 codec interface");
        return ESP_FAIL;
    }

    /* I2S data interface */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .tx_handle = s_tx_chan,
        .rx_handle = NULL,
        .clk_src = 0,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (data_if == NULL) {
        ESP_LOGE(TAG, "Failed to create I2S data interface");
        return ESP_FAIL;
    }

    /* Codec device instance */
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = codec_if,
        .data_if = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    s_codec_dev = esp_codec_dev_new(&dev_cfg);
    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create codec device");
        return ESP_FAIL;
    }

    /* Open codec with mono 16-bit PCM */
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = CONFIG_AUDIO_SPEAKER_SAMPLE_RATE,
        .channel = 1,
        .bits_per_sample = 16,
        .channel_mask = 0,
        .mclk_multiple = CONFIG_AUDIO_SPEAKER_MCLK_MULTIPLE,
    };
    int cd_ret = esp_codec_dev_open(s_codec_dev, &fs);
    if (cd_ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to open codec device: %d", cd_ret);
        esp_codec_dev_delete(s_codec_dev);
        s_codec_dev = NULL;
        return ESP_FAIL;
    }

    cd_ret = esp_codec_dev_set_out_vol(s_codec_dev, CONFIG_AUDIO_SPEAKER_DEFAULT_VOLUME);
    if (cd_ret != ESP_CODEC_DEV_OK) {
        ESP_LOGW(TAG, "Failed to set default volume: %d", cd_ret);
    }

    ESP_LOGI(TAG, "ES8311 codec initialized, volume=%d", CONFIG_AUDIO_SPEAKER_DEFAULT_VOLUME);
    return ESP_OK;
}

esp_err_t audio_speaker_init(void)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    ESP_LOGW(TAG, "Speaker is disabled in menuconfig");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_codec_dev != NULL) {
        ESP_LOGW(TAG, "Speaker already initialized");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(speaker_i2s_init(), TAG, "I2S init failed");
    esp_err_t ret = speaker_codec_init();
    if (ret != ESP_OK) {
        /* Clean up I2S on codec failure */
        if (s_tx_chan != NULL) {
            i2s_channel_disable(s_tx_chan);
            i2s_del_channel(s_tx_chan);
            s_tx_chan = NULL;
        }
        return ret;
    }

    ESP_LOGI(TAG, "Speaker initialized successfully");
    return ESP_OK;
}

esp_err_t audio_speaker_beep_test(void)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "Speaker not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    const int freq_hz = 1000;
    const int duration_ms = CONFIG_AUDIO_SPEAKER_BEEP_DURATION_MS;
    const int num_samples = (CONFIG_AUDIO_SPEAKER_SAMPLE_RATE * duration_ms) / 1000;
    const size_t buf_size = num_samples * sizeof(int16_t);

    int16_t *beep_buf = (int16_t *)calloc(1, buf_size);
    if (beep_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate beep buffer");
        return ESP_ERR_NO_MEM;
    }

    const float amplitude = 32767.0f;
    for (int i = 0; i < num_samples; i++) {
        beep_buf[i] = speaker_apply_gain(
            (int16_t)(amplitude * sinf(2.0f * (float)M_PI * freq_hz * i / CONFIG_AUDIO_SPEAKER_SAMPLE_RATE)),
            AUDIO_SPEAKER_DIGITAL_GAIN);
    }

    ESP_LOGI(TAG, "Playing %d ms beep at %d Hz", duration_ms, freq_hz);
    int cd_ret = esp_codec_dev_write(s_codec_dev, beep_buf, (int)buf_size);
    if (cd_ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Beep playback failed: %d", cd_ret);
    }

    free(beep_buf);
    return (cd_ret == ESP_CODEC_DEV_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t audio_speaker_play(const uint8_t *pcm, size_t len, TickType_t timeout)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "Speaker not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (pcm == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    (void)timeout; /* esp_codec_dev_write does not accept a timeout directly */

    /* Apply extra digital gain in-place on the PCM buffer before sending to codec.
     * The caller (audio_playback_task / beep test) owns the buffer, so in-place
     * modification avoids allocating another large buffer and risking heap exhaustion. */
    int num_samples = len / sizeof(int16_t);
    int16_t *samples = (int16_t *)pcm;
    for (int i = 0; i < num_samples; i++) {
        samples[i] = speaker_apply_gain(samples[i], AUDIO_SPEAKER_DIGITAL_GAIN);
    }

    /* Write in chunks: the ES8311/I2S path can fail with very large single writes. */
    const size_t CHUNK_BYTES = 16 * 1024;
    int cd_ret = ESP_CODEC_DEV_OK;
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > CHUNK_BYTES) {
            chunk = CHUNK_BYTES;
        }
        cd_ret = esp_codec_dev_write(s_codec_dev, (uint8_t *)samples + offset, (int)chunk);
        if (cd_ret != ESP_CODEC_DEV_OK) {
            ESP_LOGE(TAG, "audio_speaker_play: write failed at offset %u: %d", (unsigned)offset, cd_ret);
            break;
        }
        offset += chunk;
    }

    return (cd_ret == ESP_CODEC_DEV_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t audio_speaker_set_volume(uint8_t vol)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "Speaker not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    if (vol > 100) {
        vol = 100;
    }
    int cd_ret = esp_codec_dev_set_out_vol(s_codec_dev, (int)vol);
    return (cd_ret == ESP_CODEC_DEV_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t audio_speaker_mute(bool mute)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#endif

    if (s_codec_dev == NULL) {
        ESP_LOGE(TAG, "Speaker not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    int cd_ret = esp_codec_dev_set_out_mute(s_codec_dev, mute);
    return (cd_ret == ESP_CODEC_DEV_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t audio_speaker_deinit(void)
{
#if !CONFIG_AUDIO_SPEAKER_ENABLE
    return ESP_OK;
#endif

    if (s_codec_dev != NULL) {
        esp_codec_dev_close(s_codec_dev);
        esp_codec_dev_delete(s_codec_dev);
        s_codec_dev = NULL;
    }

    if (s_tx_chan != NULL) {
        i2s_channel_disable(s_tx_chan);
        i2s_del_channel(s_tx_chan);
        s_tx_chan = NULL;
    }

    /* Do NOT delete the shared I2C bus; the camera may still need it. */
    s_i2c_bus = NULL;
    s_i2c_bus_created_by_us = false;

    ESP_LOGI(TAG, "Speaker deinitialized");
    return ESP_OK;
}
