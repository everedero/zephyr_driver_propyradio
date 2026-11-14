## Design specifications related to ToDo.md Requirements points


## ToDo Requirement point 15 : AutoCalibration

Autocalibration is a special mode in which during a period of time, the user is expected to move all device joysticks in all direction to capture the min and max values of each adc associated to those device joysticks.

From a UI point of view, user will enter a special menu
Ask to start the temporary process
Then once complete , be asked to apply the value or discard

from an application point of view in the main.c in the adc_read_thread, here are the guidelines
1. add a boolean to check if in "calibration mode or not" to differentiate the normal adc measurement from this "special mode"
2. in case of "calibration mode" , for each channel use 2x4 variables to store the min & max value of ROULIS, TANGAGE, ... channels
3. for each model and for ROULIS, TANGAGE, ... the application already specified the default calibration data, keep those data as "default calibration" data when a model is created
4. the calibration mode timer is 30 seconds.
5. once the calibration mode is over, log the min, max values of each channels in the norminal console and return to "nominal" mode for the adc_read_thread
6. once the user agrees to apply those new settings , update the mode (call store) and leave this particular screen
7. Do not implement any changes in the "ui" folder, this implementation will come later
8. Implement above points 1 to 6 in a special module (.c and .h) file , except for the modifications of main.c file 

Implementation notes:
- The calibration service exposes start, active-state, sample-observation, result-read, discard, and result-clear APIs. The UI remains responsible for starting a session and deciding whether to apply or discard completed ranges.
- Existing model `min` and `max` fields remain the active calibration. Model reset restores the original defaults (`CALIBRATION_OFFSET` and `ADC_MAX_VALUE - CALIBRATION_OFFSET`); no additional calibration fields are stored in the model.
- Applying/resetting a model calibration persists the model. Callers must also refresh the runtime RF channel settings after a successful model update.
