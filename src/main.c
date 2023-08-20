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
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include "proto/sin_wave.pb.h"

/***************************************************************************************************
 * Defines
 ***************************************************************************************************/
// Thread related
#define CONSUMER_THREAD_STACK_SIZE 2048
#define CONSUMER_THREAD_PRIO 1
#define LED_THREAD_STACK_SIZE 256
#define LED_THREAD_PRIO 2

// LED related
#define GREEN_LED_NODE DT_NODELABEL(green_led_4)

// UART related
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 32

// Ringbuffer related
#define RB_INCOMING_SIZE 1024


/***************************************************************************************************
 * Types
 ***************************************************************************************************/
struct led
{
    struct gpio_dt_spec spec;
    uint8_t             num;
};

struct printk_data_t
{
    void    *fifo_reserved;
    uint32_t led;
    uint32_t cnt;
};

/***************************************************************************************************
 * Functions forward declaration
 ***************************************************************************************************/
void consume_bytes(void *p1, void *p2, void *p3);
void blink_led0(void *, void *, void *);
void blink(const struct led *led, uint32_t sleep_ms, uint32_t id);
void serial_cb(const struct device *dev, void *user_data);

/***************************************************************************************************
 * Globals
 ***************************************************************************************************/
K_THREAD_STACK_DEFINE(consumer_thread_stack, CONSUMER_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct led green_led = {.spec = GPIO_DT_SPEC_GET_OR(GREEN_LED_NODE, gpios, {0}),
                                     .num  = 0};

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


uint8_t         rb_incoming_buffer[RB_INCOMING_SIZE];
struct ring_buf rb_incoming;

/***************************************************************************************************
 * Main
 ***************************************************************************************************/
int main(int argc, char **argv)
{
    ring_buf_init(&rb_incoming, sizeof(rb_incoming_buffer), rb_incoming_buffer);

    if (!device_is_ready(uart_dev))
    {
        printk("UART device not found!");
        return 0;
    }
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    static struct k_thread consumer_thread_id;
    k_thread_create(&consumer_thread_id, consumer_thread_stack,
                    K_THREAD_STACK_SIZEOF(consumer_thread_stack), &consume_bytes, &rb_incoming,
                    NULL, NULL, CONSUMER_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&consumer_thread_id, "consumer_thread");

    static struct k_thread led_thread_id;
    k_thread_create(&led_thread_id, led_thread_stack, K_THREAD_STACK_SIZEOF(led_thread_stack),
                    &blink_led0, NULL, NULL, NULL, LED_THREAD_PRIO, 0, K_FOREVER);
    k_thread_name_set(&led_thread_id, "led_thread");


    k_thread_start(&consumer_thread_id);
    k_thread_start(&led_thread_id);


    while (1)
    {
        k_sleep(K_MSEC(100));
    }

    return 0;
}


/***************************************************************************************************
 * Function bodies
 ***************************************************************************************************/
void consume_bytes(void *p1, void *p2, void *p3)
{
    if (p1 == NULL)
    {
        return;
    }

    struct ring_buf *rb = (struct ring_buf *) p1;

    uint8_t buffer[64];
    int     rb_len;

    while (1)
    {
        k_sleep(K_MSEC(100));

        rb_len = ring_buf_get(rb, buffer, sizeof(buffer));
        if (rb_len)
        {
        }
    }
}

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

void serial_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (uart_irq_rx_ready(dev))
        {
            int     recv_len, rb_len;
            uint8_t buffer[125];
            size_t  len = MIN(ring_buf_space_get(&rb_incoming), sizeof(buffer));

            recv_len = uart_fifo_read(dev, buffer, len);
            if (recv_len < 0)
            {
                // LOG_ERR("Failed to read UART FIFO");
                recv_len = 0;
            };

            rb_len = ring_buf_put(&rb_incoming, buffer, recv_len);
            if (rb_len < recv_len)
            {
                // LOG_ERR("Drop %u bytes", recv_len - rb_len);
            }
        }

        if (uart_irq_tx_ready(dev))
        {
        }
    }
}