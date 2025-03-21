01: MOVE THE WHEELS TO STRAIGHT, SET THE ZERO OFFSETS IN PHEONIX TUNER

02: FIND THE VALUES IN CONSTANTS.JAVA THAT SAY "TUNE ME", AND ADJUST THEM BASED ON PHYSICAL CONFIGURATION

03: MAKE SURE THAT THE ROBOT MOVES +X FORWARD AND +Y LEFT USING SMARTDASHBOARD. MAKE SURE THAT THE MODULES ALL TURN CCW+ (IF THEY DO NOT, REFER BACK TO 01). MAKE SURE THAT THE OVERALL ROBOT HEADING IS CCW+ USING SMARTDASHBOARD

04: TRY AND DRIVE THE ROBOT! YOUR CONTROL SCHEME SHOULD BE: LEFT STICK CONTROLS TRANSLATION (FORWARD, BACKWARD, ETC.) AND RIGHT STICK CONTROLS ROTATION (LEFT = CCW, RIGHT = CW).

COMMON ISSUES:
    01: INCORRECT MODULE ORIENTATION - If modules don't follow the expected orientation pattern, double-check encoder offsets and motor inversion flags

     ROBOT DRIVES IN UNEXPECTED DIRECTIONS - Usually caused by incorrect coordinate system implementation. Invert the controls in RobotContainer.java.

     SMARTDASHBOARD DATA IS THE OPPOSITE OF WHAT IS HAPPENING - Remove the negative sign (-) from the offending value.

     MODULES DRIFT OVER MULTIPLE CODE DEPLOYS - Make sure that when you deploy code, you move the robot a littl forward so that it is perfectly straight.

     UNRESPONSIVE/STRANGE MODULES - Check CAN bus connectivity and device IDs in code match physical configuration

