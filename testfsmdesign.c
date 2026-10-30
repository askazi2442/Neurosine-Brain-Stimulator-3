/**
 * @file  testfsmdesign.c
 * @brief used to test that fsm in fsm.c is implemented correctly using software test
 */

#include <stdio.h>
#include <stdbool.h>

// ==========================================
// 1. FSM STATES AND EVENTS
// ==========================================

typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_RAMP_DOWN,
    STATE_FAULT
} FSMState;

typedef enum {
    EV_HW_OK,
    EV_HW_FAIL,
    EV_LEAD_OK,
    EV_LEAD_OFF,
    EV_BLE_LOST,
    EV_SOFTKILL,
    EV_BATT_CRIT,
    EV_RAMP_COMPLETE
} FSMEvent;

// ==========================================
// 2. SIMULATED FSM VARIABLES
// ==========================================

static FSMState state;
static bool hw_ok;
static bool lead_ok;
static bool immediate_cutoff;
static bool ramp_requested;

// ==========================================
// 3. HELPER FUNCTIONS
// ==========================================

const char *state_name(FSMState s)
{
    switch (s) {
        case STATE_BOOT:      return "BOOT";
        case STATE_IDLE:      return "IDLE";
        case STATE_RAMP_DOWN: return "RAMP_DOWN";
        case STATE_FAULT:     return "FAULT";
        default:              return "UNKNOWN";
    }
}

void reset_fsm(void)
{
    state = STATE_BOOT;
    hw_ok = false;
    lead_ok = false;
    immediate_cutoff = false;
    ramp_requested = false;
}

void enter_idle(void)
{
    if (state == STATE_BOOT && hw_ok && lead_ok) {
        state = STATE_IDLE;
    }
}

// Simulate immediate shutdown
void cut_outputs_now(void)
{
    immediate_cutoff = true;
}

// Simulate requesting a controlled ramp
// No real 50-second delay is needed for this test
void ramp_down_outputs(void)
{
    ramp_requested = true;
}

bool stimulation_permitted(void)
{
    return state == STATE_IDLE && hw_ok && lead_ok;
}

// ==========================================
// 4. FSM EVENT HANDLER
// ==========================================

void handle_event(FSMEvent event)
{
    switch (event) {

        case EV_HW_OK:
            if (state == STATE_BOOT) {
                hw_ok = true;
                enter_idle();
            }
            break;

        case EV_LEAD_OK:
            if (state == STATE_BOOT) {
                lead_ok = true;
                enter_idle();
            }
            break;

        case EV_HW_FAIL:
            hw_ok = false;
            lead_ok = false;
            if (state != STATE_FAULT) {
                cut_outputs_now();
                state = STATE_FAULT;
            }
            break;

        case EV_LEAD_OFF:
            lead_ok = false;
            if (state != STATE_FAULT) {
                cut_outputs_now();
                state = STATE_FAULT;
            }
            break;

        case EV_BLE_LOST:
        case EV_SOFTKILL:
            if (state == STATE_IDLE) {
                state = STATE_RAMP_DOWN;
                ramp_down_outputs();
            }
            break;

        case EV_BATT_CRIT:
            if (state == STATE_IDLE) {
                state = STATE_RAMP_DOWN;
                ramp_down_outputs();
            } else {
                state = STATE_FAULT;
            }
            break;

        case EV_RAMP_COMPLETE:
            if (state == STATE_RAMP_DOWN) {
                state = STATE_FAULT;
            }
            break;
    }
}

// ==========================================
// 5. TEST FUNCTIONS
// ==========================================

static int tests_run = 0;
static int tests_passed = 0;

void check(const char *name, bool condition)
{
    tests_run++;

    printf("TEST %02d: %-48s %s\n",
           tests_run,
           name,
           condition ? "PASS" : "FAIL");

    if (condition) {
        tests_passed++;
    }
}

void ready_fsm(void)
{
    reset_fsm();
    handle_event(EV_HW_OK);
    handle_event(EV_LEAD_OK);
}

// ==========================================
// 6. RUN ALL SOFTWARE TESTS
// ==========================================

int main(void)
{
    printf("        FSM SOFTWARE DESIGN TEST\n");

    // TEST 1: Initialization
    reset_fsm();
    check("Initial state is BOOT",
          state == STATE_BOOT);

    // TEST 2: Hardware ready, leads unknown
    handle_event(EV_HW_OK);
    check("HW_OK alone stays BOOT",
          state == STATE_BOOT);

    // TEST 3: Hardware and leads ready
    handle_event(EV_LEAD_OK);
    check("HW_OK + LEAD_OK enters IDLE",
          state == STATE_IDLE);

    // TEST 4: Reverse order of events
    reset_fsm();
    handle_event(EV_LEAD_OK);
    handle_event(EV_HW_OK);
    check("LEAD_OK + HW_OK enters IDLE",
          state == STATE_IDLE);

    // TEST 5: Lead disconnect
    ready_fsm();
    handle_event(EV_LEAD_OFF);
    check("Lead disconnect enters FAULT",
          state == STATE_FAULT && immediate_cutoff);

    // TEST 6: Reconnection does not reset fault
    handle_event(EV_LEAD_OK);
    check("Lead reconnect keeps FAULT",
          state == STATE_FAULT);

    // TEST 7: Hardware failure
    ready_fsm();
    handle_event(EV_HW_FAIL);
    check("Hardware failure cuts immediately",
          state == STATE_FAULT && immediate_cutoff);

    // TEST 8: BLE disconnect
    ready_fsm();
    handle_event(EV_BLE_LOST);
    check("BLE lost enters RAMP_DOWN",
          state == STATE_RAMP_DOWN &&
          ramp_requested && !immediate_cutoff);

    // TEST 9: Ramp completion
    handle_event(EV_RAMP_COMPLETE);
    check("Ramp complete enters FAULT",
          state == STATE_FAULT);

    // TEST 10: Software emergency stop
    ready_fsm();
    handle_event(EV_SOFTKILL);
    check("Soft kill requests ramp-down",
          state == STATE_RAMP_DOWN &&
          ramp_requested && !immediate_cutoff);

    // TEST 11: Critical battery
    ready_fsm();
    handle_event(EV_BATT_CRIT);
    check("Battery critical requests ramp-down",
          state == STATE_RAMP_DOWN &&
          ramp_requested && !immediate_cutoff);

    // TEST 12: Hardware failure during ramp
    ready_fsm();
    handle_event(EV_BLE_LOST);
    handle_event(EV_HW_FAIL);
    check("HW failure overrides ramp",
          state == STATE_FAULT && immediate_cutoff);

    // TEST 13: Lead failure during ramp
    ready_fsm();
    handle_event(EV_BLE_LOST);
    handle_event(EV_LEAD_OFF);
    check("Lead failure overrides ramp",
          state == STATE_FAULT && immediate_cutoff);

    // TEST 14: Cannot stimulate during BOOT
    reset_fsm();
    check("Stimulation blocked in BOOT",
          !stimulation_permitted());

    // TEST 15: Can stimulate when ready
    ready_fsm();
    check("Stimulation permitted in IDLE",
          stimulation_permitted());

    // TEST 16: Cannot stimulate during ramp
    handle_event(EV_BLE_LOST);
    check("Stimulation blocked during ramp",
          !stimulation_permitted());

    // TEST 17: Cannot stimulate after fault
    handle_event(EV_HW_FAIL);
    check("Stimulation blocked in FAULT",
          !stimulation_permitted());

    // TEST 18: Hardware failure during boot
    reset_fsm();
    handle_event(EV_HW_FAIL);
    check("HW failure during BOOT enters FAULT",
          state == STATE_FAULT && immediate_cutoff);

    // TEST 19: Soft kill during boot ignored
    reset_fsm();
    handle_event(EV_SOFTKILL);
    check("Soft kill in BOOT is ignored",
          state == STATE_BOOT && !ramp_requested);

    // TEST 20: HW_OK in IDLE ignored
    ready_fsm();
    handle_event(EV_HW_OK);
    check("HW_OK in IDLE keeps IDLE",
          state == STATE_IDLE);

    printf("TEST SUMMARY: %d / %d PASSED\n",
           tests_passed, tests_run);

    return (tests_passed == tests_run) ? 0 : 1;
}