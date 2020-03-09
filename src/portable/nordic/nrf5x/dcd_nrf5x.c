/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if TUSB_OPT_DEVICE_ENABLED && CFG_TUSB_MCU == OPT_MCU_NRF5X

#include "nrf.h"
#include "nrfx_usbd_errata.h"
#include "nrfx_usbd.h"
#include <nrfx_clock.h>
#include <nrfx_power.h>
#ifdef SOFTDEVICE_PRESENT
// For enable/disable hfclk with SoftDevice
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#include "device/dcd.h"

// TODO remove later
#include "device/usbd.h"
#include "device/usbd_pvt.h" // to use defer function helper

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/
enum
{
  // Max allowed by USB specs
  MAX_PACKET_SIZE   = 64,

  // Mask of all END event (IN & OUT) for all endpoints. ENDEPIN0-7, ENDEPOUT0-7, ENDISOIN, ENDISOOUT
  EDPT_END_ALL_MASK = (0xff << USBD_INTEN_ENDEPIN0_Pos) | (0xff << USBD_INTEN_ENDEPOUT0_Pos) |
                      USBD_INTENCLR_ENDISOIN_Msk | USBD_INTEN_ENDISOOUT_Msk
};
/// NRFX HANDLER ========================================
/**
 * @brief USB configured flag
 *
 * The flag that is used to mark the fact that USB is configured and ready
 * to transmit data
 */
static volatile bool m_usbd_configured = false;

/**
 * @brief Mark the fact if remote wake up is enabled
 *
 * The internal flag that marks if host enabled the remote wake up functionality in this device.
 */
static volatile bool m_usbd_rwu_enabled = false;


static volatile bool m_usbd_suspend_state_req = false;

static volatile bool ep_usbd_hid_class_ep_position_busy_check = false;

static volatile size_t last_size = 0;
static volatile bool read_size = true;

nrfx_usbd_transfer_t edpt[2][9];
static void usbd_event_handler(nrfx_usbd_evt_t const *const p_event)
{

    switch (p_event->type)
    {
        case NRFX_USBD_EVT_SUSPEND:
            //NRFX_LOG_INFO("SUSPEND state detected");
            m_usbd_suspend_state_req = true;
            dcd_event_bus_signal(0, DCD_EVENT_SUSPEND, true);
            break;
        case NRFX_USBD_EVT_RESUME:
            //NRFX_LOG_INFO("RESUMING from suspend");
            m_usbd_suspend_state_req = false;
            dcd_event_bus_signal(0, DCD_EVENT_RESUME , true);
            break;
        case NRFX_USBD_EVT_WUREQ:
            //NRFX_LOG_INFO("RemoteWU initiated");
            m_usbd_suspend_state_req = false;
            break;
        case NRFX_USBD_EVT_RESET:
        {
            m_usbd_suspend_state_req = false;
            nrfx_usbd_ep_default_config();
            dcd_event_bus_signal(0, DCD_EVENT_BUS_RESET, true);
            break;
        }
        case NRFX_USBD_EVT_SOF:
        {
            dcd_event_bus_signal(0, DCD_EVENT_SOF, true);
            break;
        }
        case NRFX_USBD_EVT_EPTRANSFER:
        {
        	nrfx_usbd_ep_t ep = p_event->data.eptransfer.ep;
        	if (NRF_USBD_EPIN_CHECK(ep)) {
//                read_size = true;
            dcd_event_xfer_complete(0, ep, edpt[tu_edpt_dir(ep)][tu_edpt_number(ep)].size, XFER_RESULT_SUCCESS, true);
        	} else {
        		TU_LOG2("EPT: %d %d\r\n", ep, nrfx_usbd_epout_size_get(ep));
            dcd_event_xfer_complete(0, ep, nrfx_usbd_epout_size_get(ep), XFER_RESULT_SUCCESS, true);

        	}
            break;
        }
        case NRFX_USBD_EVT_SETUP:
        {
            nrfx_usbd_setup_t setup;
            nrfx_usbd_setup_get(&setup);
            switch (setup.bRequest)
            {
                case 0x05: // SetAddress
                    //nothing to do, handled by hardware; but don't STALL
                    break;
                default:
					dcd_event_setup_received(0, (uint8_t *) &setup, true);
                    return;
            }
            break;
        }
        default:
            break;
    }
}

void usbd_power_event_handler(nrfx_power_usb_evt_t event)
{
    switch (event)
    {
        case NRFX_POWER_USB_EVT_DETECTED:
            if (!nrfx_usbd_is_enabled())
            {
                nrfx_usbd_enable();
            }
            break;
        case NRFX_POWER_USB_EVT_REMOVED:
            m_usbd_configured = false;

            if (nrfx_usbd_is_started())
            {
                nrfx_usbd_stop();
            }
            if (nrfx_usbd_is_enabled())
            {
                nrfx_usbd_disable();
            }
            break;
        case NRFX_POWER_USB_EVT_READY:
            if (!nrfx_usbd_is_started())
            {
                nrfx_usbd_start(true);
                dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, true);
            }
            break;
        default:
            NRFX_ASSERT(false);
    }
}
//--------------------------------------------------------------------+
// Controller API
//--------------------------------------------------------------------+
static volatile bool hfclk_is_stable;

static void clock_event_handler(nrfx_clock_evt_type_t event)
{
    if (event == NRFX_CLOCK_EVT_HFCLK_STARTED)
    {
        hfclk_is_stable = true;
    }
}


static void init_power_clock(void)
{
	static const nrfx_power_config_t config =
	{
		.dcdcen = true,
	};
    /* Initializing power and clock */
    nrfx_clock_init(clock_event_handler);
    nrfx_power_init(&config);
    nrfx_clock_hfclk_start();
    while (!hfclk_is_stable)
    {
        /* Just waiting */
    }
}
void dcd_init (uint8_t rhport)
{
	(void) rhport;
	init_power_clock();
	nrfx_usbd_init(usbd_event_handler);
	/* Configure USB events in POWER peripheral when power detection is enabled. */
	static const nrfx_power_usbevt_config_t config =
	{
			.handler = usbd_power_event_handler
	};
	nrfx_power_usbevt_init(&config);
	nrfx_power_usbevt_enable();
}

void dcd_int_enable(uint8_t rhport)
{
  (void) rhport;
//  NVIC_EnableIRQ(USBD_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
  (void) rhport;
//  NVIC_DisableIRQ(USBD_IRQn);
}

void dcd_set_address (uint8_t rhport, uint8_t dev_addr)
{
  (void) rhport;
  (void) dev_addr;
}

void dcd_set_config (uint8_t rhport, uint8_t config_num)
{
	(void) rhport;
  (void) config_num;
  nrfx_usbd_setup_clear();
}

void dcd_remote_wakeup(uint8_t rhport)
{
  (void) rhport;

}


//--------------------------------------------------------------------+
// Endpoint API
//--------------------------------------------------------------------+

static nrfx_usbd_ep_t ep_to_nrf(uint8_t ep_addr)
{
	uint8_t const epnum = tu_edpt_number(ep_addr);
	uint8_t const dir   = tu_edpt_dir(ep_addr);
	return dir == TUSB_DIR_OUT ? NRFX_USBD_EPOUT(epnum) : NRFX_USBD_EPIN(epnum);
}

bool dcd_edpt_open (uint8_t rhport, tusb_desc_endpoint_t const * desc_edpt)
{
  (void) rhport;

  nrfx_usbd_ep_enable(ep_to_nrf(desc_edpt->bEndpointAddress));

  return true;
}

bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
  (void) rhport;
  nrfx_err_t err = NRFX_SUCCESS;
  nrfx_usbd_ep_t  ep = ep_to_nrf(ep_addr);
  TU_LOG2("EPA: %x, %d\r\n", ep_addr, total_bytes);
  nrfx_usbd_transfer_t * transfer = &edpt[tu_edpt_dir(ep)][tu_edpt_number(ep)];
	  transfer->size = total_bytes;
  if (tu_edpt_number(ep_addr) == 0 && total_bytes == 0){
	  nrfx_usbd_setup_clear();
	  // The nRF doesn't interrupt on status transmit so we queue up a success response.
	  dcd_event_xfer_complete(0, ep_addr, 0, XFER_RESULT_SUCCESS, false);
  } else if (ep == NRFX_USBD_EPOUT(0) && total_bytes > 0) {
	  transfer->p_data.rx = buffer;
	  err = nrfx_usbd_ep_transfer(NRFX_USBD_EPOUT(0), transfer);
	  nrfx_usbd_setup_data_clear();
  } else if (NRF_USBD_EPIN_CHECK(ep)) {
	  transfer->p_data.tx = buffer;
	  err = nrfx_usbd_ep_transfer(ep, transfer);
  } else {
	  transfer->p_data.rx = buffer;
	  err = nrfx_usbd_ep_transfer(ep, transfer);
  }
  
  if (err != NRFX_SUCCESS) {
    return false;
  }

  return true;
}

void dcd_edpt_stall (uint8_t rhport, uint8_t ep_addr)
{
	(void) rhport;

	if (tu_edpt_number(ep_addr) == 0) {
		nrfx_usbd_setup_stall();
	} else {
		nrfx_usbd_ep_stall(ep_to_nrf(ep_addr));
	}
}

void dcd_edpt_clear_stall (uint8_t rhport, uint8_t ep_addr)
{
	(void) rhport;

	if (tu_edpt_number(ep_addr) != 0) {
		nrfx_usbd_ep_stall_clear(ep_to_nrf(ep_addr));
		nrfx_usbd_ep_dtoggle_clear(ep_to_nrf(ep_addr));
	}
}

#endif
