# e-screw
electronic lead screw for lathe using arduino due

reading the hardware encoder part was inspired from 
https://forum.arduino.cc/index.php?topic=140205.30

Status: Warning This code is not fully functional and should not be used 

LCD working ST7920
hardware encoder working
single shot timer for stepper control that generates a single pulse of 1.5ms according stepper driver spec (working)
general timer that runs the control loop (in progress)
at this moment the control loop just try to follow 1 to 1 the spindle.
multiplication and demultiplication will be done later
