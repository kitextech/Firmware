## KiteX PX4 Firmware ##

# Main modifications #

1. Created a new VTOL type (kite) with custom transition mixing of MC and FW controls.
2. Modified FW attitude controller to follow a predefined path.
3. Modified MC attitude controller to not fight yaw
4. Modified MC position controller to have a constant pitch for tether tension.  
5. Created new airframe 13013 KiteX
6. Created custom mixer vtol_kitex


# How to fly/use #

1. Take off in manual mode
2. Fly to the kite to the right side of the centre of looping.
3. Set the throttle to between 80-100%
4. Transition to FW flight mode.
⋅⋅* Looping
5. Transtion to MC flight mode when the kite is flying upwards.
6. Land.

# Parameters that needs to be set #

**Pos_B** // position of tether anchoring point in the local coordinate system (if kite is turned on at the origin pos_b = 0).
PARAM_DEFINE_FLOAT(MPC_X_POS_B, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Y_POS_B, 0.0f);
PARAM_DEFINE_FLOAT(MPC_Z_POS_B, 0.0f);


**Angles to C.** Defined the centre of looping relative to the anchoring point

Phi is relative to the X-axis, theta to the XY-plane, positive for negative z.
PARAM_DEFINE_FLOAT(MPC_PHI_C, 0.0f);
PARAM_DEFINE_FLOAT(MPC_THETA_C, 0.0f);

**turn_r** Turning radius in the Pi-plane (general turning radius - 23 meters could be a starting value)
PARAM_DEFINE_FLOAT(MPC_LOOP_TURN_R, 0.0f);


# “manual control” #
We make use of the manual remote aux channels. To control various failsafe methods and enable tethered hovering mode while in MC flight mode. AUX


### Aux1 ###  
Activated above 0.0f.
- Disables yaw compensation **(should be high for the entire tethered flight).**
- Enables fixed pitch in MC mode (not velocity controlled mode only => manual and altitude mode)
Files: mc_att_control, mc_pos_control

### Aux2 ###
AUX2 > 0.0f activates is a failsafe that’s intented to protect against crashing into the ground.
Files: vtol_att_control/kite.cpp

### Aux3 ###
AUX3 > 0.0f turns on an automatic failsafe if the kites is below 10 meters above ground.
Files: vtol_att_control/kite.cpp




# Other information #
VTOL type = 1 is the kite

```
enum vtol_type {
	TAILSITTER = 0,
	KITE,
	TILTROTOR,
	STANDARD
};
```


**Param MPC_TET_POS_CTL**
PARAM_DEFINE_FLOAT(MPC_TET_POS_CTL, 0); Activates above 0.5
// not current in use. Can be used for more sophisticated hovering on the sphere. In position control (using offboard or loiter). Be careful if in “manual” position mode as position setup is not forced to be on the sphere - only the velocity set point is set to be tangential.
