#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>



/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

#define CHIP_I2C_ADDR		0b0100000
#define I2C_NODE			DT_NODELABEL(i2c0)

//IOCON.BANK=0
#define IODIRA_ADDR			0x00
#define OLATA_ADDR			0x14
#define GPIOA_ADDR			0x12
#define IODIRB_ADDR			0x01
#define OLATB_ADDR			0x15
#define GPIOB_ADDR			0x13

/* change this to any other UART peripheral if desired */
//#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;


static const struct device *i2c0_device = DEVICE_DT_GET(I2C_NODE);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static uint8_t i2c_register_buffer[2];
static uint8_t i2c_data_buffer[2];

static uint8_t i2c_buffer[2];

uint8_t set_gpios_to_output();
void print_uart(char *buf);
void serial_cb(const struct device *dev, void *user_data);
uint8_t update_gpio_registers();
void rotate_next();
void go_to_position();


// global vars - todo change to proper local
unsigned int combined = 0x1F;  // 111110 in binary for Register A to start with five '1's
const unsigned int maskA = 0x003F; // Mask for the lower 6 bits (0000000000111111)
const unsigned int maskB = 0x0FC0; // Mask for the next 6 bits (0000111111000000)
const unsigned int lastBitBMask = 0x0800; // 12th bit from LSB, the last bit of B
unsigned int regA = 0, regB = 0, lastBitB = 0;
// current position - middle director
uint8_t current_pos = 8; // when combined starts as 0x1F
uint8_t dest_pos = 8;



int main(void)
{
	int err = 0;
	char tx_buf[MSG_SIZE];

	printk("entering main\n");
	
	// init i2c device
	if (!device_is_ready(i2c0_device)){
		printk("i2c device not ready\n\r");
		return -1;
	}	

	// init uart device
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	printk("past uart enable\n");

	rotate_next();
	update_gpio_registers();


	// while(k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0){	
	while(0){		
		printk(tx_buf);
		printk("\r\n");
		
		set_gpios_to_output();
		update_gpio_registers();

		if (strstr(tx_buf, "rotate") != NULL) {
			printk("rotate X detected\r\n");			    
			// Extract the value of X
			const char *prefix = "rotate ";
			const char *number_part = tx_buf + strlen(prefix);
			dest_pos = (uint8_t)atoi(number_part);
			printk("rotate to %d\r\n", dest_pos);
			if (current_pos != dest_pos && dest_pos > 0 && dest_pos <= 12)
				go_to_position();    
		}
		else if (strstr(tx_buf, "hold") != NULL){
			printk("hold detected\r\n");
			continue;
		}

		// output the current position
		printk("current pos: %d", current_pos);
	}
	
	dest_pos = 5;
	go_to_position();

	while (1){
		set_gpios_to_output();
		update_gpio_registers();		
		
		// sleep
	}
	
	
	return err;
}

uint8_t update_gpio_registers(){
	uint8_t err = 0;
	//bank a			
	i2c_buffer[0] = GPIOA_ADDR;
	i2c_buffer[1] = regA;
	err = i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
	if (err != 0) return err;	
	//bank b			
	i2c_buffer[0] = GPIOB_ADDR;
	i2c_buffer[1] = regB;
	err = i2c_write(i2c0_device, i2c_buffer, 2, CHIP_I2C_ADDR);
	
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

void rotate_next(){
	if (current_pos<=11) current_pos++;
	else current_pos = 1;
	printk("rotating\r\n\n\n");	

	lastBitB = (combined & lastBitBMask) ? 1 : 0; // Check the last bit of B before shift
	combined <<= 1;
	// Wrap the last bit of B to the start of A if set
	if (lastBitB) {
		combined |= 0x01; // Set the first bit of A
	}
	// Extract A and B
	regA = combined & maskA;
	regB = (combined & maskB) >> 6; // Shift down to make it 6-bit
}

void go_to_position(){
	// set to initial position
	// rotate next X times :)))
	combined = 0x1F;  // 111110 in binary for Register A to start with five '1's
	regA = 0; regB = 0; lastBitB = 0;
	current_pos = 9;
	rotate_next();

	while(current_pos != dest_pos){
		printk("in the while loop\r\n");
		printk("current pos: %d, dest pos: %d\r\n", current_pos, dest_pos); 
		rotate_next();
	}

	update_gpio_registers();
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
	//printk("serial event happened\n");
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}