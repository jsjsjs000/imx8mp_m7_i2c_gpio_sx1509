	/* Freescale includes. */
#include <fsl_device_registers.h>
#include <fsl_debug_console.h>
#include <fsl_gpio.h>
#include <fsl_i2c.h>
#include <fsl_i2c_freertos.h>

	/* FreeRTOS kernel includes. */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>

	/* Board config */
#include "board_cfg/board.h"
#include "board_cfg/clock_config.h"
#include "board_cfg/pin_mux.h"

#include "main.h"
#include "common.h"
#include "i2c_task.h"
#include "i2c_gpio_sx1509.h"

volatile bool i2c_gpio_sx1509_irq = false;

static uint8_t i2c_buffor[I2C_BUFFOR_SIZE];
static i2c_master_handle_t *i2c_master_handle;

static i2c_rtos_handle_t master_rtos_handle;
static i2c_master_config_t masterConfig;
static i2c_master_transfer_t masterXfer;
static uint32_t sourceClock;

void i2c_task_initialize(void)
{
	CLOCK_SetRootMux(I2C_OLED_CLK_ROOT, I2C_OLED_ROOT_MUX_SYS_PLL_DIV); /* Set I2C source to SysPLL1 Div5 160MHZ */
	CLOCK_SetRootDivider(I2C_OLED_CLK_ROOT, 1U, 10U);                   /* Set root clock to 160MHZ / 10 = 16MHZ */
	NVIC_SetPriority(I2C_OLED_IRQN, 3);

	/*
		masterConfig.baudRate_Bps = 100000U;
		masterConfig.enableStopHold = false;
		masterConfig.glitchFilterWidth = 0U;
		masterConfig.enableMaster = true;
	*/
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = I2C_BAUDRATE;
	sourceClock = I2C_OLED_CLK_FREQ;

	status_t status = I2C_RTOS_Init(&master_rtos_handle, I2C_OLED, &masterConfig, sourceClock);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during init, 0x%x\r\n", status);
	}

	i2c_master_handle = &master_rtos_handle.drv_handle;

	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress   = I2C_OLED_SLAVE_ADDR_7BIT;
	masterXfer.direction      = kI2C_Write;
	masterXfer.data           = i2c_buffor;
	masterXfer.dataSize       = 0;
	masterXfer.flags          = kI2C_TransferDefaultFlag;
}

void i2c_task_task(void *pvParameters)
{
	PRINTF("I2C task started.\r\n");

	i2c_gpio_sx1509_reset();
	vTaskDelay(10);
	i2c_gpio_sx1509_set_high_speed(0, true);
	i2c_gpio_sx1509_set_high_speed(1, true);
	i2c_gpio_sx1509_set_direction(0, false);
	i2c_gpio_sx1509_set_direction(1, false);

	i2c_gpio_sx1509_set_high_speed(8, true);
	i2c_gpio_sx1509_set_high_speed(9, true);
	i2c_gpio_sx1509_set_pull_up(8, true);
	i2c_gpio_sx1509_set_pull_up(9, true);
	i2c_gpio_sx1509_set_direction(8, true);
	i2c_gpio_sx1509_set_direction(9, true);

#ifdef I2C_TASK_SCAN_INPUT
	i2c_gpio_sx1509_set_pin_interrupt(8, false);
	i2c_gpio_sx1509_set_pin_interrupt(9, false);
#endif
#ifdef I2C_TASK_IRQ_INPUT
	i2c_gpio_sx1509_set_sense_interrupt(8, sense_interrupt_type_falling);
	i2c_gpio_sx1509_set_sense_interrupt(9, sense_interrupt_type_falling);
	i2c_gpio_sx1509_set_pin_interrupt(8, true);
	i2c_gpio_sx1509_set_pin_interrupt(9, true);
#endif

#ifdef I2C_TASK_SCAN_INPUT
	TickType_t timer_out_100ms = xTaskGetTickCount();
	TickType_t timer_int_50ms = xTaskGetTickCount();
	bool last_out = false;
	bool last_in8 = true;
	bool last_in9 = true;
#endif

#ifdef I2C_TASK_IRQ_INPUT
	uint16_t in_;
	i2c_gpio_sx1509_get_interrupt_source(&in_);
#endif


	while (true)
	{
#ifdef I2C_TASK_SCAN_INPUT
		if (xTaskGetTickCount() - timer_out_100ms >= I2C_TASK_TIMER_OUT_MS)
		{
			timer_out_100ms = xTaskGetTickCount();

			i2c_gpio_sx1509_set_data(last_out, false);
			i2c_gpio_sx1509_set_data(!last_out, true);
			last_out = !last_out;
		}

		if (xTaskGetTickCount() - timer_int_50ms >= I2C_TASK_TIMER_IN_MS)
		{
			timer_int_50ms = xTaskGetTickCount();

			bool in8, in9;
			i2c_gpio_sx1509_get_data(8, &in8);
			i2c_gpio_sx1509_get_data(9, &in9);
			if (in8 != last_in8)
			{
				PRINTF("in8 = %d\r\n", in8);
				last_in8 = in8;
			}
			if (in9 != last_in9)
			{
				PRINTF("in9 = %d\r\n", in9);
				last_in9 = in9;
			}
		}
#endif

#ifdef I2C_TASK_IRQ_INPUT
		if (i2c_gpio_sx1509_irq || GPIO_PinRead(I2C_GPIO_SX1509_IRQ_PORT, I2C_GPIO_SX1509_IRQ_PIN) == 0)
		{
			i2c_gpio_sx1509_irq = false;

			uint16_t in;
			if (i2c_gpio_sx1509_get_interrupt_source(&in))
			{
				bool in8 = GET_BIT(in, 8) != 0;
				bool in9 = GET_BIT(in, 9) != 0;
				if (in8)
					PRINTF("in8 fall down\r\n", in8);
				if (in9)
					PRINTF("in9 fall down\r\n", in9);
			}
		}
#endif

		vTaskDelay(pdMS_TO_TICKS(1));
	}

	vTaskSuspend(NULL);
}

bool i2c_task_read_data(uint8_t *send, size_t send_size, uint8_t *received, size_t received_size)
{
		/* write to I2C */
	memcpy(i2c_buffor, send, send_size);
	masterXfer.direction = kI2C_Write;
	masterXfer.dataSize = 1;

	status_t status = I2C_RTOS_Transfer(&master_rtos_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during write transaction, 0x%x\r\n", status);
		return false;
	}

		/* read from I2C */
	masterXfer.direction = kI2C_Read;
	masterXfer.dataSize = received_size;

	status = I2C_RTOS_Transfer(&master_rtos_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during read transaction, 0x%x\r\n", status);
		return false;
	}
	*received = i2c_buffor[0];

	return true;
}

bool i2c_task_write_data(uint8_t *data, size_t data_size)
{
		/* write to I2C */
	memcpy(i2c_buffor, data, data_size);
	masterXfer.direction = kI2C_Write;
	masterXfer.dataSize = data_size;

	status_t status = I2C_RTOS_Transfer(&master_rtos_handle, &masterXfer);
	if (status != kStatus_Success)
	{
		PRINTF("I2C master: error during write transaction, 0x%x\r\n", status);
		return false;
	}

	return true;
}
