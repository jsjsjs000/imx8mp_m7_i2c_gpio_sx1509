#include <stdio.h>

#define I2C_OLED_BASE                  I2C4
#define I2C_OLED_IRQN                  I2C4_IRQn
#define I2C_OLED_CLK_ROOT              kCLOCK_RootI2c4
#define I2C_OLED_ROOT_MUX_SYS_PLL_DIV  kCLOCK_I2cRootmuxSysPll1Div5
#define I2C_OLED_CLK_FREQ                                                                      \
		(CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(I2C_OLED_CLK_ROOT)) /  \
		(CLOCK_GetRootPostDivider(I2C_OLED_CLK_ROOT)) / 5)                           /* SYSTEM PLL1 DIV5 */

#define I2C_OLED ((I2C_Type *)I2C_OLED_BASE)

#define I2C_OLED_SLAVE_ADDR_7BIT   0x3cU    /* SSD1362 */
#define I2C_BAUDRATE               400000   /* 400 KHz */
#define I2C_BUFFOR_SIZE            256      /* minimum 128 bytes for 256 OLED width (4 bit per color) */

#define master_task_PRIORITY (configMAX_PRIORITIES - 2)

extern void i2c_task_initialize(void);
extern void i2c_task_task(void *pvParameters);
extern bool i2c_task_write_data(uint8_t *data, size_t data_size);
