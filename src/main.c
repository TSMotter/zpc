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
int  init_led(const struct led *pled);
void blink_led0(void *, void *, void *);
void blink(const struct led *pled, uint32_t sleep_ms, uint32_t id);
void serial_cb_single_byte(const struct device *dev, void *user_data);
void serial_cb_multi_byte(const struct device *dev, void *user_data);
void timer_callback(struct k_timer *dummy);

/***************************************************************************************************
 * Globals
 ***************************************************************************************************/
K_THREAD_STACK_DEFINE(consumer_thread_stack, CONSUMER_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
K_TIMER_DEFINE(tim_uart_isr_inactivity, timer_callback, NULL);

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

static struct k_sem sem_uart_isr_inactivity;


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
    uart_irq_callback_user_data_set(uart_dev, serial_cb_multi_byte, NULL);
    uart_irq_rx_enable(uart_dev);

    k_sem_init(&sem_uart_isr_inactivity, 0, 1);

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
int init_led(const struct led *pled)
{
    const struct gpio_dt_spec *spec = &pled->spec;
    int                        ret;

    if (!device_is_ready(spec->port))
    {
        printk("Error: %s device is not ready\n", spec->port->name);
        return -1;
    }

    ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
    if (ret != 0)
    {
        printk("Error %d: failed to configure pin %d\n", ret, spec->pin);
        return ret;
    }
    return 0;
}

void consume_bytes(void *p1, void *p2, void *p3)
{
    yahdlc_control_t control_recv;
    int              rc  = 0;
    int              cnt = 0;

    const struct led          *pled = &blue_led;
    const struct gpio_dt_spec *spec = &pled->spec;

    if (init_led(pled))
    {
        printk("Error while configuring LED\n");
    }

    while (1)
    {
        k_sem_take(&sem_uart_isr_inactivity, K_FOREVER);

        // Get the data from the frame
        rc = yahdlc_get_data(&control_recv, uart_isr_buffer, uart_isr_buffer_idx,
                             binary_payload_recovered_from_hdlc_buffer,
                             &binary_payload_recovered_len);

        // Success -> complete HDLC frame found -> will decode the binary payload now
        if (rc >= 0)
        {
            // DECODE PROTOBUF

            // ACT
            gpio_pin_set(spec->port, spec->pin, cnt % 2);
            cnt++;
        }

        // CLEAN
        rc                  = 0;
        uart_isr_buffer_idx = 0;
        memset(uart_isr_buffer, 0, HDLC_FRAME_BUFFER_SIZE);
        yahdlc_get_data_reset();
    }
}

void blink_led0(void *, void *, void *)
{
    blink(&green_led, 50, 0);
}

void blink(const struct led *pled, uint32_t sleep_ms, uint32_t id)
{
    const struct gpio_dt_spec *spec = &pled->spec;
    int                        cnt  = 0;

    if (init_led(pled))
    {
        printk("Error while configuring LED\n");
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

void serial_cb_multi_byte(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev))
    {
        if (uart_irq_rx_ready(dev))
        {
            int    actual_length_read;
            size_t space_available = HDLC_FRAME_BUFFER_SIZE - uart_isr_buffer_idx;

            actual_length_read =
                uart_fifo_read(dev, &uart_isr_buffer[uart_isr_buffer_idx], space_available);
            if (actual_length_read < 0)
            {
                // LOG_ERR("Failed to read UART FIFO");
                actual_length_read = 0;
            }
            uart_isr_buffer_idx += actual_length_read;

            /* (re)start a one shot timer that'll expire indicating USART inactivity to let
            the received data to be processed */
            k_timer_start(&tim_uart_isr_inactivity, K_MSEC(10), K_NO_WAIT);
        }

        if (uart_irq_tx_ready(dev))
        {
        }
    }
}

void timer_callback(struct k_timer *dummy)
{
    k_sem_give(&sem_uart_isr_inactivity);
}