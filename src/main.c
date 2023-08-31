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

#include "external/yahdlc.h"

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
#define BLUE_LED_NODE DT_NODELABEL(blue_led_6)

// UART related
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define UART_RX_BUFFER_SIZE 512
#define HDLC_FRAME_BUFFER_SIZE (UART_RX_BUFFER_SIZE + 32)

/***************************************************************************************************
 * Types
 ***************************************************************************************************/
struct led
{
    struct gpio_dt_spec spec;
    uint8_t             num;
};

/***************************************************************************************************
 * Functions forward declaration
 ***************************************************************************************************/
void consume_bytes(void *p1, void *p2, void *p3);
void blink_led0(void *, void *, void *);
void blink(const struct led *led, uint32_t sleep_ms, uint32_t id);
void serial_cb_single_byte(const struct device *dev, void *user_data);
void serial_cb_multi_byte(const struct device *dev, void *user_data);

/***************************************************************************************************
 * Globals
 ***************************************************************************************************/
K_THREAD_STACK_DEFINE(consumer_thread_stack, CONSUMER_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);

static const struct led green_led = {.spec = GPIO_DT_SPEC_GET_OR(GREEN_LED_NODE, gpios, {0}),
                                     .num  = 0};
static const struct led blue_led  = {.spec = GPIO_DT_SPEC_GET_OR(BLUE_LED_NODE, gpios, {0}),
                                     .num  = 0};
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

// Buffer to receive UART ISR
static uint8_t      uart_isr_buffer[HDLC_FRAME_BUFFER_SIZE];
static unsigned int uart_isr_buffer_idx;

// Buffer to place the recovered protobuf binary payload from the HDLC frame
static uint8_t      binary_payload_recovered_from_hdlc_buffer[UART_RX_BUFFER_SIZE];
static unsigned int binary_payload_recovered_len;

/***************************************************************************************************
 * Main
 ***************************************************************************************************/
int main(int argc, char **argv)
{
    /* UART related */
    if (!device_is_ready(uart_dev))
    {
        printk("UART device not found!");
        return 0;
    }
    uart_irq_callback_user_data_set(uart_dev, serial_cb_single_byte, NULL);
    uart_irq_rx_enable(uart_dev);

    /* Thread related */
    static struct k_thread consumer_thread_id;
    static struct k_thread led_thread_id;

    k_thread_create(&consumer_thread_id, consumer_thread_stack,
                    K_THREAD_STACK_SIZEOF(consumer_thread_stack), &consume_bytes, NULL, NULL, NULL,
                    CONSUMER_THREAD_PRIO, 0, K_FOREVER);
    k_thread_create(&led_thread_id, led_thread_stack, K_THREAD_STACK_SIZEOF(led_thread_stack),
                    &blink_led0, NULL, NULL, NULL, LED_THREAD_PRIO, 0, K_FOREVER);

    k_thread_name_set(&consumer_thread_id, "consumer_thread");
    k_thread_name_set(&led_thread_id, "led_thread");

    k_thread_start(&consumer_thread_id);
    k_thread_start(&led_thread_id);

    while (1)
    {
        k_msleep(100);
    }

    return 0;
}


/***************************************************************************************************
 * Function bodies
 ***************************************************************************************************/
void consume_bytes(void *p1, void *p2, void *p3)
{
    yahdlc_control_t control_recv;
    int              rc = 0;
    while (1)
    {
        // Get the data from the frame
        rc = yahdlc_get_data(&control_recv, uart_isr_buffer, uart_isr_buffer_idx,
                             binary_payload_recovered_from_hdlc_buffer,
                             &binary_payload_recovered_len);

        // Success -> complete HDLC frame found -> will decode the binary payload now
        if (rc >= 0)
        {
            // DECODE PROTOBUF

            // ACT

            // CLEAN
            rc                  = 0;
            uart_isr_buffer_idx = 0;
            memset(uart_isr_buffer, 0, HDLC_FRAME_BUFFER_SIZE);
            yahdlc_get_data_reset();
        }
        k_msleep(100);
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

void serial_cb_single_byte(const struct device *dev, void *user_data)
{
    if (!uart_irq_update(uart_dev))
    {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev))
    {
        return;
    }

    if (uart_isr_buffer_idx >= HDLC_FRAME_BUFFER_SIZE)
    {
        return;
    }

    uint8_t rx_byte;

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &rx_byte, 1) == 1)
    {
        uart_isr_buffer[uart_isr_buffer_idx++] = rx_byte;

        if (uart_isr_buffer_idx >= HDLC_FRAME_BUFFER_SIZE)
        {
            return;
        }
    }
}
