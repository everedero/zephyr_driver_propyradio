## Objectives
this file gathers all the remaining requirements (todo) to be implemented for this application. It can be used by as inputs for generative AI agents.

## Requirements (todo)
1. Be able to measure battery level and display it on the main screen
2. Implement 'end point' settings in the settings UI panel to se the 'min' and 'max' of each channel. ==> DONE
3. Revisit the 'display' of all channels for debug in a special panel instead of displaying it on 'main' screen in the header ==> DONE
4. Implement 'Mixing' features between 2 channels (example for Elevon and V Tail)
5. Implement 'curve' of channels: be able to change from a linear function to a more complex (logarithmic) function for each channel 
6. Implement hold throttle (protection) function and be able to assign it to a button
7. Implement Up to 2 timers functions and add the capabilities to assign it a a button (meaning you can start/stop timer from)
8. Be able to display the firmware version
9. Be able to reset all settings to factory (corresponding to erasing internal file system)
10. Do not implement Trainer/Student mode (need HW update)
11. Implement 'Over The Air' or 'USB' Firmware update (for second case need HW)
12. Implement Rx Settings UI and Rx Display UI. Rx Parameters corresponds to: 
    a. Battery level of Rx Device
    b. GPS Data
    c. Position Data
13. Optimize power consumption to reduce (Backlight & Display switch-off) ==> DONE
14. Implement binding mechanism that scan particular RF channel to recieve the receiver characteristics such as dedicated reception/transmission channels then swich to remote controller onto this particular reception/transmission channels
15. Implement autocalibration mode ==> DONE