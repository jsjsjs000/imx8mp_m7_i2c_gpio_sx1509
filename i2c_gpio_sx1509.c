#include <fsl_common.h>
#include <fsl_device_registers.h>
#include <fsl_debug_console.h>
#include "i2c_task.h"
#include "i2c_gpio_sx1509.h"

static bool i2c_gpio_sx1509_read_register(uint8_t reg, uint8_t *received)
{
	uint8_t data[] = { reg };
	return i2c_task_read_data(data, sizeof(data), received, 1);
}

static bool i2c_gpio_sx1509_write_register(uint8_t reg, uint8_t value)
{
	uint8_t data[] = { reg, value };
	return i2c_task_write_data(data, sizeof(data));
}

bool i2c_gpio_sx1509_reset(void)
{
	if (!i2c_gpio_sx1509_write_register(REG_LONG_SLEW_A, 0x12))
		return false;

	return i2c_gpio_sx1509_write_register(REG_LONG_SLEW_A, 0x34);
}

bool i2c_gpio_sx1509_set_high_speed(uint8_t pin, bool high_speed)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_LONG_SLEW_A : REG_LONG_SLEW_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	if (high_speed)
		value |= (1 << (pin % 8));
	else
		value &= ~(1 << (pin % 8));
	return i2c_gpio_sx1509_write_register(register_, value);
}

bool i2c_gpio_sx1509_set_direction(uint8_t pin, bool input)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_DIR_A : REG_DIR_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	if (input)
		value |= (1 << (pin % 8));
	else
		value &= ~(1 << (pin % 8));
	return i2c_gpio_sx1509_write_register(register_, value);
}

bool i2c_gpio_sx1509_set_pull_up(uint8_t pin, bool pull_up)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_PULL_UP_A : REG_PULL_UP_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	if (pull_up)
		value |= (1 << (pin % 8));
	else
		value &= ~(1 << (pin % 8));
	return i2c_gpio_sx1509_write_register(register_, value);
}

bool i2c_gpio_sx1509_set_data(uint8_t pin, bool output)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_DATA_A : REG_DATA_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	if (output)
		value |= (1 << (pin % 8));
	else
		value &= ~(1 << (pin % 8));
	return i2c_gpio_sx1509_write_register(register_, value);
}

bool i2c_gpio_sx1509_get_data(uint8_t pin, bool *input)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_DATA_A : REG_DATA_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	*input = (value >> (pin % 8)) & 0x01;
	return true;
}

bool i2c_gpio_sx1509_set_pin_interrupt(uint8_t pin, bool interrupt)
{
	uint8_t value;
	uint8_t register_ = (pin < 8) ? REG_INTERRUPT_MASK_A : REG_INTERRUPT_MASK_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	if (interrupt)
		value |= (1 << (pin % 8));
	else
		value &= ~(1 << (pin % 8));
	return i2c_gpio_sx1509_write_register(register_, value);
}

bool i2c_gpio_sx1509_set_sense_interrupt(uint8_t pin, sense_interrupt_type_t sense_interrupt_type)
{
	uint8_t value;
	uint8_t register_;
	if (pin >= 0 && pin < 4)
		register_ = REG_SENSE_LOW_A;
	else if (pin >= 4 && pin < 8)
		register_ = REG_SENSE_HIGH_A;
	else if (pin >= 8 && pin < 12)
		register_ = REG_SENSE_LOW_B;
	else if (pin >= 12 && pin < 16)
		register_ = REG_SENSE_HIGH_B;
	if (!i2c_gpio_sx1509_read_register(register_, &value))
		return false;

	value &= ~(3 << (pin % 4));
	value |= (sense_interrupt_type << ((pin % 4) * 2));
	return i2c_gpio_sx1509_write_register(register_, value);
}
