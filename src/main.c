/***************************************************************************************************
 *
 ***************************************************************************************************/
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/printk.h>


/***************************************************************************************************
 * Defines
 ***************************************************************************************************/
#define STACK_SIZE_CONSUMER_THREAD 512
#define PRIORITY_CONSUMER_THREAD 5
#define MY_RING_BUF_BYTES 1024

#define GREEN_LED_NODE DT_ALIAS(green_led)

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 32


/***************************************************************************************************
 * Types
 ***************************************************************************************************/
struct printk_data_t
{
    void    *fifo_reserved; /* 1st word reserved for use by fifo */
    uint32_t led;
    uint32_t cnt;
};
struct led
{
    struct gpio_dt_spec spec;
    uint8_t             num;
};

/***************************************************************************************************
 * Forward declaration
 ***************************************************************************************************/
void blink_led0(void *, void *, void *);
void blink(const struct led *led, uint32_t sleep_ms, uint32_t id);
void serial_cb(const struct device *dev, void *user_data);
void print_uart(char *buf);

/***************************************************************************************************
 * Globals
 ***************************************************************************************************/
K_THREAD_STACK_DEFINE(consumer_thread_stack, STACK_SIZE_CONSUMER_THREAD);
static const struct led green_led = {
    .spec = GPIO_DT_SPEC_GET_OR(GREEN_LED_NODE, gpios, {0}),
    .num  = 0,
};

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char                       rx_buf[MSG_SIZE];
static int                        rx_buf_pos;


/***************************************************************************************************
 * Main
 ***************************************************************************************************/
int main(int argc, char **argv)
{
    if (!device_is_ready(uart_dev))
    {
        printk("UART device not found!");
        return 0;
    }
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    static struct k_thread consumer_thread_data;
    k_thread_create(&consumer_thread_data, consumer_thread_stack,
                    K_THREAD_STACK_SIZEOF(consumer_thread_stack), &blink_led0, NULL, NULL, NULL,
                    PRIORITY_CONSUMER_THREAD, 0, K_FOREVER);
    k_thread_name_set(&consumer_thread_data, "consumer_thread");
    k_thread_start(&consumer_thread_data);

    print_uart("Hello! I'm your echo bot.\r\n");
    print_uart("Tell me something and press enter:\r\n");

    char tx_buf[MSG_SIZE];
    while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0)
    {
        print_uart("Echo: ");
        print_uart(tx_buf);
        print_uart("\r\n");
    }

    return 0;
}


/***************************************************************************************************
 * Function bodies
 ***************************************************************************************************/
void blink_led0(void *, void *, void *)
{
    blink(&green_led, 50, 0);
}

void blink(const struct led *led, uint32_t sleep_ms, uint32_t id)
{
    const struct gpio_dt_spec *spec = &led->spec;
    int                        cnt  = 0;
    int                        ret;

    if (!device_is_ready(spec->port))
    {
        printk("Error: %s device is not ready\n", spec->port->name);
        return;
    }

    ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
    if (ret != 0)
    {
        printk("Error %d: failed to configure pin %d (LED '%d')\n", ret, spec->pin, led->num);
        return;
    }

    while (1)
    {
        gpio_pin_set(spec->port, spec->pin, cnt % 2);
        k_msleep(sleep_ms);
        cnt++;
    }
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev))
    {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev))
    {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1)
    {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0)
        {
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        }
        else if (rx_buf_pos < (sizeof(rx_buf) - 1))
        {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(uart_dev, buf[i]);
    }
}