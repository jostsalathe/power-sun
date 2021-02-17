# Power Sun - firmware

## Animation parameters
32 Hz animation rate.
8 seconds delay buffer.

### Live parameters derived from inputs:
- PWR input: ON or OFF state
- HDD input: Pulse intensity value (8 bit?) ramped linearly

### Parameters
- master brightness rings (8 bit)
- master brightness stripes (8 bit)
- propagation delay
- standby brightness
- standby increment
- standby timeout (default = 32)
- starting brightness
- starting increment
- idle brightness
- idle decrement
- active brightness
- active increment
- stopping decrement

## Animation description
The central LED ring essentially runs the animation and the other rings just follow successively with
a fixed delay between each ring. The 6th channel which controls the LED stripes on the edges of the
PC case acts as the outer ring but has a separate maximum brightness.
 
### PWR transition OFF -> ON
Fade to standby brightness for now using starting increment.
Only after standby timeout expires, switch to starting state.

### Actual power on animation after standby timeout
Fade to power-on peak brightness using starting increment.
As soon as power-on peak brightness is reached, HDD dependent fading (running state) commences.

### Running state
Fade the LEDs to to hdd idle (using idle Decrement) or active (using active increment) brightness
depending on HDD input state.

### PWR transition OFF -> ON
Fade to 0 brightness using stopping decrement.

## Configuration interface
Uses virtual COM port via USB full speed.
- As soon as a host opens the USB CDC, the animation is stopped and the configuration CLI is presented.
- The CLI contains trigger for the purpose of testing different settings which start specific actions on the animation.
- All parameters mentioned above can be configured by this CLI.
- Closing the USB CDC stores all settings to Flash memory if they were changed and restarts normal operation.

## Notes
- Maximum timer resolution for 1kHz PWM frequency: 48000
- Gamma correction with LUT provided by [victornpb.github.io/gamma-table-generator](https://victornpb.github.io/gamma-table-generator)
 
