#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#define CHIP_I2C_ADDR		0b0100000
#define I2C_NODE			DT_NODELABEL(i2c0)

//assuming IOCON.BANK=0
/*
#define IODIRA_ADDR			0x00
#define OLATA_ADDR			0x14
#define GPIOA_ADDR			0x12

#define IODIRB_ADDR			0x01
#define OLATB_ADDR			0x15
#define GPIOB_ADDR			0x13
*/

//assuming IOCON.BANK=1

#define IODIRA_ADDR			0x00
#define OLATA_ADDR			0x0A
#define GPIOA_ADDR			0x09

//assuming IOCON.BANK=1
#define IODIRB_ADDR			0x10
#define OLATB_ADDR			0x1A
#define GPIOB_ADDR			0x19



static const struct device *i2c0_device = DEVICE_DT_GET(I2C_NODE);

static uint8_t i2c_register_buffer[2];
static uint8_t i2c_data_buffer[2];

static uint8_t i2c_buffer[2];

uint8_t read_from_register();
uint8_t write_to_register();

int main(void)
{
	int err, i;

	if (!device_is_ready(i2c0_device)){
		printk("i2c device not ready\n\r");
		return;
	}

		

	while(1){
		do{
			//bank a
			
			//read from direction and state registers
			i2c_register_buffer[0] = IODIRA_ADDR; // IODIRA: pin bank A I/O mode
			read_from_register();
			printk("data from register %d: %d \n\r", i2c_register_buffer[0], i2c_data_buffer[0]);

			i2c_register_buffer[0] = OLATA_ADDR; // OLATA: pin bank A latch register
			read_from_register();
			printk("data from register %d: %d \n\r", i2c_register_buffer[0], i2c_data_buffer[0]);

			i2c_register_buffer[0] = GPIOA_ADDR; // GPIOA: pin bank A GPIO register
			read_from_register();
			printk("data from register %d: %d \n\n\r", i2c_register_buffer[0], i2c_data_buffer[0]);
			
			//set direction to output (low) and pin states to high			
			i2c_buffer[0] = IODIRA_ADDR;
			i = i%2; 
			if (i) i2c_buffer[1] = 0x00;
			else i2c_buffer[1] = 0xFF;
			i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);

			i2c_buffer[0] = OLATA_ADDR;
			i = i%2; 
			if (!i) i2c_buffer[1] = 0x00;
			else i2c_buffer[1] = 0xFF;
			i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);

			i2c_buffer[0] = GPIOA_ADDR;
			i = i%2; 
			if (!i) i2c_buffer[1] = 0x00;
			else i2c_buffer[1] = 0xFF;
			err = i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
			
			//bank b
			
			//read from direction and state registers
			
			i2c_register_buffer[0] = IODIRB_ADDR; // IODIRA: pin bank B I/O mode
			read_from_register();
			printk("data from register %d: %d \n\r", i2c_register_buffer[0], i2c_data_buffer[0]);

			i2c_register_buffer[0] = OLATB_ADDR; // OLATA: pin bank b latch register
			read_from_register();
			printk("data from register %d: %d \n\r", i2c_register_buffer[0], i2c_data_buffer[0]);

			i2c_register_buffer[0] = GPIOB_ADDR; // GPIOA: pin bank B GPIO register
			read_from_register();
			printk("data from register %d: %d \n\r", i2c_register_buffer[0], i2c_data_buffer[0]);
			
			//set direction to output (low) and pin states to high
			
			i2c_buffer[0] = IODIRB_ADDR;
			i2c_buffer[1] = 0xFF;			
			i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);

			i2c_buffer[0] = OLATB_ADDR;
			i = i%2; 
			if (!i) i2c_buffer[1] = 0x00;
			else i2c_buffer[1] = 0xFF;
			i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
			
			i2c_buffer[0] = GPIOB_ADDR;
			i = i%2; 
			if (!i) i2c_buffer[1] = 0x00;
			else i2c_buffer[1] = 0xFF;


			i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);

		i++;

		}while(false);

		k_msleep(SLEEP_TIME_MS);
	}
}

uint8_t read_from_register(){
	int err = 0;
	err = i2c_write(i2c0_device, i2c_register_buffer, 1, CHIP_I2C_ADDR);
	if (err<0){printk("failed: %n \n\r", &err);}

	err = i2c_read(i2c0_device, i2c_data_buffer, 1, CHIP_I2C_ADDR);
	if (err<0){printk("failed: %n \n\r", &err);}

	return err;
}

uint8_t write_to_register(){	
	int err = 0;
	err = i2c_write(i2c0_device, i2c_register_buffer, 1, CHIP_I2C_ADDR);
	if (err<0){printk("failed: %n \n\r", &err); return err;}

	err = i2c_write(i2c0_device, i2c_data_buffer, 1, CHIP_I2C_ADDR);
	if (err<0){printk("failed: %n \n\r", &err); return err;}

	return err;	
}

/*

6 LSB bank 12 i 13 


*/