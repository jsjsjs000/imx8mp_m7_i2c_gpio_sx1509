#define REG_LONG_SLEW_B         0x02
#define REG_LONG_SLEW_A         0x03
#define REG_PULL_UP_B           0x06
#define REG_PULL_UP_A           0x07
#define REG_PULL_DOWN_B         0x08
#define REG_PULL_DOWN_A         0x09
#define REG_OPEN_DRAIN_B        0x0a
#define REG_OPEN_DRAIN_A        0x0b
#define REG_DIR_B               0x0e
#define REG_DIR_A               0x0f
#define REG_DATA_B              0x10
#define REG_DATA_A              0x11
#define REG_INTERRUPT_MASK_B    0x12
#define REG_INTERRUPT_MASK_A    0x13
#define REG_SENSE_HIGH_B        0x14
#define REG_SENSE_LOW_B         0x15
#define REG_SENSE_HIGH_A        0x16
#define REG_SENSE_LOW_A         0x17
#define REG_INTERRUPT_SOURCE_B  0x18
#define REG_INTERRUPT_SOURCE_A  0x19
#define REG_EVENT_STATUS_B      0x1a
#define REG_EVENT_STATUS_A      0x1b
#define REG_LEVEL_SHIFTER_1     0x1c
#define REG_LEVEL_SHIFTER_2     0x1d
#define REG_RESET               0x7d

typedef enum
{
	sense_interrupt_type_none = 0,
	sense_interrupt_type_rising = 1,
	sense_interrupt_type_falling = 2,
	sense_interrupt_type_both = 3,
} sense_interrupt_type_t;

extern bool i2c_gpio_sx1509_reset(void);
extern bool i2c_gpio_sx1509_set_high_speed(uint8_t pin, bool high_speed);
extern bool i2c_gpio_sx1509_set_direction(uint8_t pin, bool input);
extern bool i2c_gpio_sx1509_set_pull_up(uint8_t pin, bool pull_up);
extern bool i2c_gpio_sx1509_set_data(uint8_t pin, bool output);
extern bool i2c_gpio_sx1509_get_data(uint8_t pin, bool *input);
extern bool i2c_gpio_sx1509_set_pin_interrupt(uint8_t pin, bool interrupt);
extern bool i2c_gpio_sx1509_set_sense_interrupt(uint8_t pin, sense_interrupt_type_t sense_interrupt_type);
extern bool i2c_gpio_sx1509_get_interrupt_source(uint16_t *value);
