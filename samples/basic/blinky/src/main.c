/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <sys/util.h>
#include <drivers/i2c.h>

#include <nrfx.h>
#include <nrfx_twis.h>
#include <irq.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


typedef nrfx_twis_t nrf_drv_twis_t;
static const nrfx_twis_t m_twis = NRFX_TWIS_INSTANCE(1);
static uint8_t rx_buf[100];
static uint8_t tx_buf[100] = {0x20,0x05,3,4,5};


nrfx_twis_evt_type_t evts;

static void twis_event_handler(nrfx_twis_evt_t const * const p_event)
{
    switch (p_event->type)
    {
    case NRFX_TWIS_EVT_READ_REQ:
        nrfx_twis_tx_prepare(&m_twis,
                             &tx_buf,
                             sizeof(tx_buf));
        break;
    case NRFX_TWIS_EVT_READ_DONE:
        break;
    case NRFX_TWIS_EVT_WRITE_REQ:
        nrfx_twis_rx_prepare(&m_twis,
                             &rx_buf,
                             sizeof(rx_buf));
        break;
    case NRFX_TWIS_EVT_WRITE_DONE:
        break;

    case NRFX_TWIS_EVT_READ_ERROR:
    case NRFX_TWIS_EVT_WRITE_ERROR:
    case NRFX_TWIS_EVT_GENERAL_ERROR:
    default:
        break;
    }
    evts |= p_event->type;
}

void setup_i2c_slave(void)
{
  IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c1)),
        DT_IRQ(DT_NODELABEL(i2c0), priority),
        nrfx_isr, nrfx_twis_1_irq_handler, 0);

  nrfx_twis_config_t config;
  config.addr[0] = 0x55;
  config.addr[1] = 0x0B;
  config.scl = 34; //PIN_SCL;
  config.scl_pull = NRF_GPIO_PIN_PULLUP;
  config.sda = 35; //PIN_SDA;
  config.sda_pull = NRF_GPIO_PIN_PULLUP;
  config.interrupt_priority = 2;
  const int ret = nrfx_twis_init(&m_twis, &config, twis_event_handler);
  __ASSERT(ret == 0, "Init failed");
  nrfx_twis_enable(&m_twis);
}

void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	setup_i2c_slave();
  nrfx_twis_tx_prepare(&m_twis,
                        &tx_buf,
                        sizeof(tx_buf));
  nrfx_twis_rx_prepare(&m_twis,
                        &rx_buf,
                        sizeof(rx_buf));
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);

    if(evts & NRFX_TWIS_EVT_READ_REQ)
    {
      printk("Read REQ\n");
    }
    if(evts & NRFX_TWIS_EVT_READ_DONE)
    {
        printk("Read done\n");
    }
    if(evts & NRFX_TWIS_EVT_WRITE_REQ)
    {
        printk("Write rq\n");
    }
    if(evts & NRFX_TWIS_EVT_WRITE_DONE)
    {
        printk("Write done\n");
        //for(int i = 0; i < 10; i++)
        //{
        //  printk("%x\t", rx_buf[i]);
        //}
        //printk("\n");
    }
    if(evts & NRFX_TWIS_EVT_READ_ERROR ||
       evts & NRFX_TWIS_EVT_WRITE_ERROR ||
       evts & NRFX_TWIS_EVT_GENERAL_ERROR)
    {
        printk("err %d\n", evts);
    }
    evts = 0;

		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);
	}
}
