#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

// Movecall Moji configuration

#include <driver/gpio.h>
#include <driver/touch_sensor.h>

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000
// Currently the INPUT_REFERENCE prevent device from capturing audio correctly, so disable it for now
#define AUDIO_INPUT_REFERENCE    false

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_6
#define AUDIO_I2S_GPIO_WS GPIO_NUM_12
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_14
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_13
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_11

#define AUDIO_CODEC_PA_PIN       GPIO_NUM_9
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_5
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_4
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define AUDIO_CODEC_ES7210_ADDR  ES7210_CODEC_DEFAULT_ADDR

#define BUILTIN_LED_GPIO        GPIO_NUM_8
#define BOOT_BUTTON_GPIO        GPIO_NUM_0

#define DISPLAY_WIDTH   466
#define DISPLAY_HEIGHT  466
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false

#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

// VCI_EN控制引脚 - 用于显示屏电源控制
#define VCI_EN_GPIO             GPIO_NUM_1

#define DISPLAY_SPI_SCLK_PIN    GPIO_NUM_16
#define DISPLAY_SPI_MOSI_PIN    GPIO_NUM_17
#define DISPLAY_SPI_CS_PIN      GPIO_NUM_15
#define DISPLAY_SPI_DC_PIN      GPIO_NUM_7
#define DISPLAY_SPI_RESET_PIN   GPIO_NUM_18

#define DISPLAY_SPI_SCLK_HZ     (80 * 1000 * 1000)

// TOUCH PAD配置
#define TOUCH_PAD_GPIO           GPIO_NUM_3
#define TOUCH_PAD_CHANNEL        TOUCH_PAD_NUM3
#define TOUCH_PAD_THRESHOLD      500    // 触摸阈值
#define TOUCH_PAD_DEBOUNCE_MS    50     // 防抖时间(毫秒)

#define BATTERY_ADC_CHANNEL      ADC_CHANNEL_9 // 电池ADC通道
#define BATTERY_CHRG_STAT_GPIO   GPIO_NUM_40 // 充电状态GPIO

// BMI270陀螺仪配置
#define BMI270_I2C_ADDR          0x68    // BMI270 I2C地址
#define BMI270_I2C_SDA_PIN       GPIO_NUM_5  // 复用音频编解码器的I2C引脚
#define BMI270_I2C_SCL_PIN       GPIO_NUM_4
#define BMI270_I2C_PORT          I2C_NUM_0
#define BMI270_I2C_FREQ_HZ       400000  // 400kHz
#define BMI270_INT_PIN           GPIO_NUM_21 // BMI270中断管脚

// BMI270寄存器定义
#define BMI270_ERR_REG           0x02    // Error register
#define BMI270_STATUS            0x1E    // Status register
#define BMI270_AUX_DATA0         0x04    // Auxiliary data register
#define BMI270_ACC_DATA0         0x0C    // Accelerometer data register
#define BMI270_GYR_DATA0         0x12    // Gyroscope data register
#define BMI270_ACC_CONF          0x40    // Accelerometer configuration
#define BMI270_ACC_RANGE         0x41    // Accelerometer range
#define BMI270_GYR_CONF          0x42    // Gyroscope configuration
#define BMI270_GYR_RANGE         0x43    // Gyroscope range
#define BMI270_AUX_CONF          0x44    // Auxiliary configuration
#define BMI270_FIFO_CONFIG_0     0x46    // FIFO configuration 0
#define BMI270_FIFO_CONFIG_1     0x47    // FIFO configuration 1
#define BMI270_FIFO_DATA         0x24    // FIFO data register
#define BMI270_FIFO_LENGTH_0     0x25    // FIFO length 0
#define BMI270_FIFO_LENGTH_1     0x26    // FIFO length 1
#define BMI270_FIFO_WM_0         0x30    // FIFO watermark 0
#define BMI270_FIFO_WM_1         0x31    // FIFO watermark 1
#define BMI270_FIFO_STATUS       0x2C    // FIFO status
#define BMI270_PWR_CONF          0x7C    // Power configuration
#define BMI270_PWR_CTRL          0x7D    // Power control
#define BMI270_CMD               0x7E    // Command register

// BMI270中断相关寄存器
#define BMI270_INT1_IO_CTRL      0x53    // INT1引脚控制寄存器
#define BMI270_INT2_IO_CTRL      0x54    // INT2引脚控制寄存器
#define BMI270_INT1_MAP_FEAT     0x55    // INT1功能映射寄存器
#define BMI270_INT2_MAP_FEAT     0x56    // INT2功能映射寄存器
#define BMI270_INT_MAP_DATA      0x58    // 数据中断映射寄存器
#define BMI270_INT_LOW_HIGH      0x5A    // 中断电平配置寄存器
#define BMI270_INT_MOTION        0x5F    // 运动中断配置寄存器
#define BMI270_INT_TAP           0x63    // 点击中断配置寄存器
#define BMI270_INT_ORIENT        0x67    // 方向中断配置寄存器
#define BMI270_INT_FLAT          0x6B    // 平放中断配置寄存器
#define BMI270_INT_FIFO_WM       0x6F    // FIFO水位中断配置寄存器
#define BMI270_INT_FIFO_FULL     0x73    // FIFO满中断配置寄存器
#define BMI270_INT_DRDY          0x77    // 数据就绪中断配置寄存器
#define BMI270_INT_FFULL         0x7B    // FIFO满中断配置寄存器



// UART for external motor controller (set TX/RX to actual pins if available)
#include <driver/uart.h>
#ifndef MOJI_UART_PORT_NUM
#define MOJI_UART_PORT_NUM      UART_NUM_0
#endif
#ifndef MOJI_UART_BAUD_RATE
#define MOJI_UART_BAUD_RATE     (115200)
#endif
#ifndef MOJI_UART_TXD
#define MOJI_UART_TXD           GPIO_NUM_NC
#endif
#ifndef MOJI_UART_RXD
#define MOJI_UART_RXD           GPIO_NUM_NC
#endif
#ifndef MOJI_UART_RTS
#define MOJI_UART_RTS           (-1)
#endif
#ifndef MOJI_UART_CTS
#define MOJI_UART_CTS           (-1)
#endif


#endif // _BOARD_CONFIG_H_
