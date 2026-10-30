/**
 * @file  fsm.c
 * @brief Hardware guardian FSM — ESP32-S3 DDS controller.
 */

#include "fsm.h"
#include <stddef.h>
#include "gpio_helpers.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "FSM";
static FSMState_t  g_state = STATE_BOOT;
static dds_master_t *g_dds = NULL;
static bool g_lead_ok = false;
static bool g_hw_ok = false;

static void try_enter_idle(void)
{
    if (g_state == STATE_BOOT &&
        g_hw_ok &&
        g_lead_ok) {

        g_state = STATE_IDLE;
    }
}

// OLD: ramps down current. I did update the name of the method to be more clear

static void ramp_down_outputs(void)
{
    // 1. Stop all DDS slave outputs over I2C first. 
    if (g_dds && g_dds->initialized) {
        for (uint8_t i = 0; i < DDS_NUM_SLAVES; i++) {
            dds_master_set_enable(g_dds, i, false);
        }
        ESP_LOGW(TAG, "Ramp-down requested for all DDS outputs");
    }
    // UPDATED for 5 mA
    /*Wait 50s for all the otuputs to ramp down to zero before cutting the rails.*/
    //This is a safety measure to ensure stimulation is fully stopped before cutting power, to avoid any risk of hardware damage or patient discomfort.
     vTaskDelay(50000 / portTICK_PERIOD_MS);
    // 2. Then cut the high-voltage rails. 
    channel_rails_disable_all();
    // UPDATE: message that ramp down is complete
    ESP_LOGW(TAG, "Ramp-down complete; HV rails disabled");
}


// UPDATED: new method for lead disconnect, no ramp. immediate shut off rails
static void cut_outputs_now(void)
{
    if (g_dds && g_dds->initialized) {
        for (uint8_t i = 0; i < DDS_NUM_SLAVES; i++) {
            dds_master_set_enable(g_dds, i, false);
        }
        ESP_LOGW(TAG, "All DDS slave outputs disabled (immediate)");
    }
    channel_rails_disable_all();
    ESP_LOGW(TAG, "HV rails disabled immediately");
}

void fsm_init(dds_master_t *dds)
{
    g_dds   = dds;
    g_state = STATE_BOOT;
    g_hw_ok = false;
    g_lead_ok = false;
    ESP_LOGI(TAG, "BOOT");
}

FSMState_t fsm_get_state(void) { return g_state; }

// UPDATED: check before enabling rails
bool fsm_stim_permitted(void)
{
    return (g_state == STATE_IDLE) && g_hw_ok && g_lead_ok;
}

const char *fsm_state_name(FSMState_t s)
{
    switch (s) {
        case STATE_BOOT:      return "BOOT";
        case STATE_IDLE:      return "IDLE";
        case STATE_RAMP_DOWN: return "RAMP_DOWN";  // UPDATED
        case STATE_FAULT:     return "FAULT";
        default:              return "UNKNOWN";
    }
}

void fsm_handle_event(FSMEvent_t ev)
{
    FSMState_t old = g_state;

    switch (ev) {
        // UPDATED
        // Hardware is ok AND leads connected. BOOT -> IDLE
        case EV_HW_OK:
            if (g_state == STATE_BOOT) {
            g_hw_ok = true;
            try_enter_idle();
        }
        break;


        // Hardware failure always results in FAULT.
        case EV_HW_FAIL:
         g_hw_ok = false;
         g_lead_ok = false;
            if (g_state != STATE_FAULT) {
                cut_outputs_now();
                g_state = STATE_FAULT;
            }
        break;

        // // Soft kill switch
        // case EV_SOFTKILL:
        //     if (g_state == STATE_IDLE) {
        //         g_state = STATE_RAMP_DOWN;
        //         ESP_LOGW(TAG, "Soft stop: beginning ramp-down");
        //         ramp_down_outputs();
        //         g_state = STATE_FAULT;
        //     }
        //     break;

        // // IDLE -> RAMP_DOWN -> FAULT if in IDLE state
        // case EV_BATT_CRIT:
        // if (g_state == STATE_IDLE) {
        //     g_state = STATE_RAMP_DOWN;
        //     ESP_LOGW(TAG, "Critical battery: beginning ramp-down");
        //     ramp_down_outputs();
        //     g_state = STATE_FAULT;
        // }
        
        // else {
        //     g_state = STATE_FAULT;
        // }
        // break;

        // // UPDATED: BLE/website connection lost -> do NOT cut current abruptly
        // // IDLE -> RAMP_DOWN -> FAULT
        // case EV_BLE_LOST:
        //     if (g_state == STATE_IDLE) {
        //         g_state = STATE_RAMP_DOWN;
        //         ESP_LOGW(TAG, "BLE connection lost — ramping down");
        //         ESP_LOGI(TAG, "%s → %s", fsm_state_name(old), fsm_state_name(g_state));
        //         old = g_state;             
        //         ramp_down_outputs();
        //         g_state = STATE_FAULT;
        //     } 
        //     else if (g_state == STATE_LEAD_OFF) {
        //         g_state = STATE_FAULT;
        //     }
        //     break;

        case EV_BLE_LOST:
        case EV_SOFTKILL:
            if (g_state == STATE_IDLE) {
                g_state = STATE_RAMP_DOWN;
            }
            break;

        case EV_BATT_CRIT:
            if (g_state == STATE_IDLE) {
                g_state = STATE_RAMP_DOWN;
            } 
            
            else {
                g_state = STATE_FAULT;
            }
            
            break;

        case EV_RAMP_COMPLETE:
            if (g_state == STATE_RAMP_DOWN) {
                g_state = STATE_FAULT;
            }
            break;

        // UPDATED: lead disconnected -> outputs off + HV rails cut immediately
        case EV_LEAD_OFF:
            g_lead_ok = false;
            if (g_state != STATE_FAULT) {
                cut_outputs_now();
                g_state = STATE_FAULT;
            }
            break;
 
        // UPDATED: lead connected -> allow state to go to IDLE
        case EV_LEAD_OK:
            if (g_state == STATE_BOOT) {
                g_lead_ok = true;
                try_enter_idle();
            }
            break;
    }

    if (g_state != old) {
        ESP_LOGI(TAG, "%s → %s", fsm_state_name(old), fsm_state_name(g_state));
    }
}
