#include <stdio.h>
#include <stdbool.h>
#include <fsl_device_registers.h>

#define I2C_OLED_BASE                  I2C4
#define I2C_OLED_IRQN                  I2C4_IRQn
#define I2C_OLED_CLK_ROOT              kCLOCK_RootI2c4
#define I2C_OLED_ROOT_MUX_SYS_PLL_DIV  kCLOCK_I2cRootmuxSysPll1Div5
#define I2C_OLED_CLK_FREQ                                                                      \
		(CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(I2C_OLED_CLK_ROOT)) /  \
		(CLOCK_GetRootPostDivider(I2C_OLED_CLK_ROOT)) / 5)                           /* SYSTEM PLL1 DIV5 */

#define I2C_OLED ((I2C_Type *)I2C_OLED_BASE)

#define I2C_OLED_SLAVE_ADDR_7BIT   0x3e     /* SX1509 */
#define I2C_BAUDRATE               400000   /* 400 KHz */
#define I2C_BUFFOR_SIZE            256      /* minimum 128 bytes for 256 OLED width (4 bit per color) */

#define master_task_PRIORITY (configMAX_PRIORITIES - 2)

#define I2C_TASK_TIMER_IN_MS  50
#define I2C_TASK_TIMER_OUT_MS 100

volatile bool i2c_gpio_sx1509_input_irq;

extern void i2c_task_initialize(void);
extern void i2c_task_task(void *pvParameters);
extern bool i2c_task_read_data(uint8_t *send, size_t send_size, uint8_t *received, size_t received_size);
extern bool i2c_task_write_data(uint8_t *data, size_t data_size);
