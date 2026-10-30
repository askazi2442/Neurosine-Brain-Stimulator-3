/**
 * @file  fsm.h
 * @brief Hardware guardian FSM for the ESP32-S3 DDS controller.
 *
 *   BOOT       Power-on; DDS I2C bus not yet initialised.
 *   IDLE       Hardware ready; channels may be commanded freely via BLE.
 *   EMERGENCY  Soft-stop button pressed; all DDS slave outputs disabled.
 *   FAULT      DDS I2C bus init failed.
 *
 *   BOOT  → EV_HW_OK        → IDLE
 *   BOOT  → EV_HW_FAIL      → FAULT
 *   IDLE  → EV_EMERGENCY    → EMERGENCY  (all slaves disabled)
 *   EMERGENCY  — latched; requires power-cycle to recover
 *   ANY   → EV_BATT_CRIT    → FAULT      (battery critical — all slaves disabled)
 */

#pragma once

#include <stdbool.h>
#include "i2c_dds_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// UPDATED
typedef enum { STATE_BOOT, STATE_IDLE, STATE_FAULT, STATE_RAMP_DOWN} FSMState_t;
typedef enum { EV_HW_OK, EV_HW_FAIL, EV_SOFTKILL, EV_BATT_CRIT, EV_BLE_LOST, EV_LEAD_OFF, EV_LEAD_OK, EV_RAMP_COMPLETE }    FSMEvent_t;



/**
 * @brief Initialise the FSM.  Must be called after dds_master_init_new_bus().
 * @param dds  Pointer to the initialised DDS master (used to disable slaves on
 *             EV_EMERGENCY).
 */
void        fsm_init(dds_master_t *dds);

FSMState_t  fsm_get_state(void);
const char *fsm_state_name(FSMState_t s);
void        fsm_handle_event(FSMEvent_t ev);

/* 
 * UPDATED: When someone sends “ON,” main.c asks the FSM whether stimulation is allowed. If a lead hasn’t been confirmed connected, or the device is shutting down, it refuses the command.
 * UPDATED: When Bluetooth disconnects, main.c immediately blocks new “ON” commands and starts the shutdown job. That job tells the FSM to turn the outputs off, wait 30 seconds, then cut power to them.
 */

bool        fsm_stim_permitted(void);

#ifdef __cplusplus
}
#endif
