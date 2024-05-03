#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define CHIP_I2C_ADDR		0b0100000
#define I2C_NODE			DT_NODELABEL(i2c0)

//IOCON.BANK=0
#define IODIRA_ADDR			0x00
#define OLATA_ADDR			0x14
#define GPIOA_ADDR			0x12
#define IODIRB_ADDR			0x01
#define OLATB_ADDR			0x15
#define GPIOB_ADDR			0x13

static const struct device *i2c0_device = DEVICE_DT_GET(I2C_NODE);

static uint8_t i2c_register_buffer[2];
static uint8_t i2c_data_buffer[2];

static uint8_t i2c_buffer[2];

uint8_t set_gpios_to_output();

int main(void)
{
	int err = 0;
	
	if (!device_is_ready(i2c0_device)){
		printk("i2c device not ready\n\r");
		return -1;
	}

	unsigned int combined = 0x1F;  // 111110 in binary for Register A to start with five '1's
    const unsigned int maskA = 0x003F; // Mask for the lower 6 bits (0000000000111111)
    const unsigned int maskB = 0x0FC0; // Mask for the next 6 bits (0000111111000000)
    const unsigned int lastBitBMask = 0x0800; // 12th bit from LSB, the last bit of B

	unsigned int regA = 0, regB = 0, lastBitB = 0;

	while(1){	
		set_gpios_to_output();

		//Update GPIO registers
		//bank a			
		i2c_buffer[0] = GPIOA_ADDR;
		i2c_buffer[1] = regA;
		err = i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
		if (err != 0) continue;
		
		//bank b			
		i2c_buffer[0] = GPIOB_ADDR;
		i2c_buffer[1] = regB;
		err = i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
		if (err != 0) continue;


		lastBitB = (combined & lastBitBMask) ? 1 : 0; // Check the last bit of B before shift
        combined <<= 1;
        // Wrap the last bit of B to the start of A if set
        if (lastBitB) {
            combined |= 0x01; // Set the first bit of A
        }
        // Extract A and B
        regA = combined & maskA;
        regB = (combined & maskB) >> 6; // Shift down to make it 6-bit        

		k_msleep(SLEEP_TIME_MS);
	}

	return err;
}

uint8_t set_gpios_to_output(){
	i2c_buffer[0] = IODIRA_ADDR;
	i2c_buffer[1] = 0x00;
	i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);

	i2c_buffer[0] = IODIRB_ADDR;
	i2c_buffer[1] = 0x00;			
	i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
}
