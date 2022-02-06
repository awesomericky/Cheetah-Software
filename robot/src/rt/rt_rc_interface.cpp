#include <pthread.h>
#include <rt/rt_rc_interface.h>
#include "Utilities/EdgeTrigger.h"
#include <string.h> // memcpy
#include <stdio.h>
#include <rt/rt_sbus.h>
#include <cmath>
static pthread_mutex_t lcm_get_set_mutex =
    PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */

// Controller Settings
rc_control_settings rc_control;

/* ------------------------- HANDLERS ------------------------- */

// Controller Settings
void get_rc_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &rc_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

//void get_rc_channels(void *settings) {
//pthread_mutex_lock(&lcm_get_set_mutex);
//v_memcpy(settings, &rc_channels, sizeof(rc_channels));
//pthread_mutex_unlock(&lcm_get_set_mutex);
//}

EdgeTrigger<int> mode_edge_trigger(0);
EdgeTrigger<TaranisSwitchState> backflip_prep_edge_trigger(SWITCH_UP);
EdgeTrigger<TaranisSwitchState> experiment_prep_edge_trigger(SWITCH_UP);
TaranisSwitchState initial_mode_go_switch = SWITCH_DOWN;

void sbus_packet_complete() {
  Taranis_X7_data data;
  update_taranis_x7(&data);

  float v_scale = data.knobs[0]*1.5f + 2.0f; // from 0.5 to 3.5
  float w_scale = data.knobs[0]*1.f + 2.f; // from 1.0 to 3.0

  auto estop_switch = data.right_lower_right_switch;

  int selected_mode = 0;

  switch(estop_switch) {

    case SWITCH_UP: // ESTOP
      selected_mode = RC_mode::OFF;
      break;

    case SWITCH_MIDDLE: // recover
      selected_mode = RC_mode::RECOVERY_STAND;
      break;

    case SWITCH_DOWN: // run
      selected_mode = RC_mode::RL_JOINT_PD;


    // Deadband
    for(int i(0); i<2; ++i){
      data.left_stick[i] = deadband(data.left_stick[i], 0.1, -1., 1.);
      data.right_stick[i] = deadband(data.right_stick[i], 0.1, -1., 1.);
    }

    rc_control.v_des[0] = v_scale * data.left_stick[1];
    rc_control.v_des[1] = -v_scale * data.left_stick[0] / (data.knobs[0]*1.5f + 2.0f);
    rc_control.v_des[2] = 0;

    rc_control.omega_des[0] = 0;
    rc_control.omega_des[1] = 0;
    rc_control.omega_des[2] = -w_scale * data.right_stick[0];

      break;
  }

bool trigger = mode_edge_trigger.trigger(selected_mode);
if(trigger || selected_mode == RC_mode::OFF || selected_mode == RC_mode::RECOVERY_STAND) {
  if(trigger) {
    printf("MODE TRIGGER!\n");
  }
  rc_control.mode = selected_mode;
}

}

void *v_memcpy(void *dest, volatile void *src, size_t n) {
  void *src_2 = (void *)src;
  return memcpy(dest, src_2, n);
}

float deadband(float command, float deadbandRegion, float minVal, float maxVal){
  if (command < deadbandRegion && command > -deadbandRegion) {
    return 0.0;
  } else {
    return (command / (2)) * (maxVal - minVal);
  }
}

