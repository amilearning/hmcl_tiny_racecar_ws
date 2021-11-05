Header header


uint16  scan_index          # Scan index
bool    long_look           # 0 = Med / 1 = Long Look Radar
uint32  dsp_timestamp       # DSP timestamp (ms)
float32 vehicle_speed_calc  # Estimated vehicle speed using stationary targets (m/s) 
float32 yaw_rate_calc       # Estimated bias-compensated yaw-rate (deg/s)
float32 curvature_calc      # Estimated road curvature (1/m)

uint8   alignment_status    # 0=unknown, 1=converged, 2=failed, 3=reserved
bool    is_radiating        # True if sensor is radiating
bool    is_blocked          # True if sensor is blocked
bool    error               # True if sensor has failed
bool    error_temp          # True if sensor has overheated
bool    error_comm          # True if sensor has detected a communication error.

uint8   alignment_state     # 0=off, 1=init, 2=automatic, 3=factory, 4=service, >4=reserved

float32 angle_misalignment  # angle used by the tracker to correct detection values (deg, clockwise)
float32 auto_align_angle    # automatic alignment angle (deg, clockwise)

int32   system_status       # 0=config, 1=startup, 2=running, 3=blocked, 4=faulty, 5=shutdown, 6=hot
bool    td_blocked          # True if there is a track dropped blockage
bool    mnr_blocked         # True if there is a max-to-noise ratio blockage
bool    bb_blocked          # True if there is a blockage beam blockage
bool    radar_blocked       # True if there is a DSP radar blockage
