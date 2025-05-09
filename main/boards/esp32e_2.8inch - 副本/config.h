#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// 音频配置
#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
#define AUDIO_INPUT_REFERENCE    true

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_38
#define AUDIO_I2S_GPIO_WS GPIO_NUM_13
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_12
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_45

#define AUDIO_CODEC_USE_PCA9557
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_1
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_2
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_CODEC_ES7210_ADDR  0x82

// LED和按钮定义
#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

#define GREEN_LED_GPIO          GPIO_NUM_16
#define BLUE_LED_GPIO           GPIO_NUM_17
#define RED_LED_GPIO            GPIO_NUM_22
#define BATTERY_PIN_GPIO        GPIO_NUM_34

// 显示配置
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY true

#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_42
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT true

// TFT 显示屏配置
#define TFT_MISO GPIO_NUM_12
#define TFT_MOSI GPIO_NUM_13
#define TFT_SCLK GPIO_NUM_14
#define TFT_CS   GPIO_NUM_15
#define TFT_DC   GPIO_NUM_2
#define TFT_RST  -1
#define TFT_BL   GPIO_NUM_42

// 触摸屏配置
#define TOUCH_CS GPIO_NUM_33
#define TOUCH_IRQ_PIN        GPIO_NUM_36
#define TOUCH_MOSI_PIN       GPIO_NUM_32
#define TOUCH_MISO_PIN       GPIO_NUM_39
#define TOUCH_CLK_PIN        GPIO_NUM_25

// 触摸校准参数
#define TOUCH_MIN_X 200
#define TOUCH_MAX_X 3700
#define TOUCH_MIN_Y 300
#define TOUCH_MAX_Y 3800

// SPI 配置
#define SPI_FREQUENCY  55000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

// 字体加载配置
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

// 启用平滑字体
#define SMOOTH_FONT

#endif // _BOARD_CONFIG_H_