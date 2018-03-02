//TELEOP

//Package statement
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.PwmControl;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;


//Declaration for display on the driver station
@TeleOp(name = "TeleOpLimits", group = "LAMeets")

//Class declaration
public class TeleopLimits extends OpMode {

    //MOTORS

    //Driving
    DcMotor rightMotor; //Right drive motor, for driving forwards and backwards
    DcMotor leftMotor;  //Left drive motor, for driving forwards and backwards
    DcMotor backMotor;  //Back drive motor, for driving sideways, a.k.a "strafing"

    //Relic
    DcMotor relicMotor; //Motor to extend the relic scoring mechanism to score the relic

    //Intake
    DcMotor rightIntakeMotor; //Motor that controls the right intake/right wheel of the intake, used to intake glyphs
    DcMotor leftIntakeMotor; //Motor that controls the left intake/left wheel of the intake, used to intake glyphs

    //Block Scoring
    DcMotor panLifterMotor; //Motor that lifts and lowers the block scoring mechanism, known as the "pan", to score blocks in the cryptobox

    //SERVOS

    //Block Rotation
    Servo leftPanSpin; //Servo on the left side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo rightPanSpin; //Servo on the right side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks

    //Relic
    Servo upDown; //Servo to raise the relic mechanism up and down, in order to balance the relic in the third zone
    Servo grab; //Servo to grab the relic

    //Glyphs
    Servo touchServo; //Servo that extends an arm from the front of the robot to detect when we are ready to score in autonomous
                      //The arm is attached to a metal bar to block glyphs from entering the robot incorrectly in teleop

    //Ball Scoring
    Servo feelerRaise; //Servo that lifts and lowers the ball scoring mechanism, known as the "feeler"

    //Servo variables
    final double FEELER_RAISE_UP_POSITION = .65; //Position that the feelerRaise is set to in teleop
    final double FEELER_RAISE_ENDGAME_POSITION = .5; //Position that the feelerRaise is set to in endgame, lower than FEELER_RAISE_UP_POSITION as to not interfere with the relic arm

    //CONTROLS

    //Driving controls, all for gamepad 1
    double leftStickX1Raw; //The raw value of the x-axis of the leftStick, used for determining if and how much the robot should rotate while driving
    double rightStickX1Raw; //The raw value of the x-axis of the rightStick, used for determining if and how much the robot should strafe while driving
    double rightStickY1Raw; //The raw value of the y-axis of the rightStick, used for determining if and how much the robot should drive forwards/backwards while driving

    //Cube movement controls, all for gamepad 1
    double rightTrigger; //double for the extent to which rightTrigger is pressed. No press is 0, full press is 1. Used for intaking blocks in teleop and grabbing the relic in endgame.
    boolean rightBumper; //boolean for rightBumper. Set to true if rightBumper is pressed and set to false otherwise. Used to score blocks in teleop.
    double leftTrigger; //double for the extent to which leftTrigger is pressed. No press is 0, full press is 1. Used for outtaking blocks in teleop and releasing the relic in endgame.
    boolean leftBumper; //boolean for leftBumper. Set to true if leftBumper is pressed and set to false otherwise. Used to lower the elevator and pan, or block scoring mechanism, in teleop.
    boolean dpadDownPressed; //boolean for dpadDown. Set to true if dpadDown is pressed and set to false otherwise. Used to lower the elevator in teleop and to retract the relic scoring mechanism in endgame.
    boolean dpadUpPressed; //boolean for dpadUp. Set to true if dpadUp is pressed and set to false otherwise. Used to raise the elevator in teleop and to extend the relic scoring mechanism in endgame.
    boolean dpadLeftPressed; //boolean for dpadLeft. Set to true if dpadLeft is pressed and set to false otherwise
    boolean yPressed; //boolean for the y button.  Set to true if y is pressed and set to false otherwise.
    boolean aPressed; //boolean for the a button.  Set to true if a is pressed and set to false otherwise.
    boolean xPressed; //boolean for the x button.  Set to true if x is pressed and set to false otherwise.
    boolean bPressed; //boolean for the b button.  Set to true if b is pressed and set to false otherwise.
    boolean rightStickButton;
    boolean leftStickButton;

    //Relic controls, all for gamepad 1
    boolean dpadRightPressed; //boolean for dpadRight. Set to true if dpadRight is pressed and set to false otherwise
    double grabPosition;
    double upDownPosition;

    //Driving variables
    double finLeftPower; //The final power to be applied to leftMotor
    double finRightPower; //The final power to be applied to rightMotor
    double finBackPower; //The final power to be applied to backMotor

    //Block variables
    double leftIntakePower; //The power to which we set leftIntakeMotor
    double rightIntakePower; //The power to which we set rightIntakeMotor
    double panLiftingPower; //The power to which we set panLifterMotor
    double panSpinPosition; //The position to which we set leftPanSpin and rightPanSpin

    //BLOCK SENSORS

    DistanceSensor sensorA; //Distance sensor closest to the intake to see how far away potential blocks are
    DistanceSensor sensorB; //Distance sensor between A and C, to see how far potential blocks are
    DistanceSensor sensorC; //Distance sensor farthest from intake, to see how far potential blocks are

    //Limit Switches
    DigitalChannel limitTop; //Limit Switch that tells us if we reach the top of the robot with the Pan
    DigitalChannel limitBottom; //Limit switch that tells us if we reach the bottom of the robot with the Pan

    // Gain Mode Booleans
    boolean highGain = true; //boolean that is true if we are making driving faster and is false otherwise
    boolean lowGain; //boolean that is true if we are making driving slower and is false otherwise
    boolean gainToggle; //boolean that is true if the joystick right stick button and is false otherwise

    //Mode booleans
    boolean previousStatus; //Boolean that is true if the previous mode was end game, and is false otherwise
    boolean currentStatus; //Boolean that is true if the current mode is endgame, and is false otherwise

    boolean toggleLB;
    boolean score;

    int bothBlockCounter;

    boolean hasLifted = false;

    double threeGlyphCounter;

    boolean threeGlyphs;


    double sensorAVal;
    double sensorBVal;
    double sensorCVal;

    double touchServoPosition = .69;

    double feelerRaisePosition;

    double relicMotorPower;


    /* Initialize standard Hardware interfaces */
    public void init() { //init method to configure hardware and set initial values before teleop begins

        //Motor configurations in the hardware map. Used so that the robot controller can apply the correct motor powers depending on what port a motor is plugged into.
        rightMotor = hardwareMap.dcMotor.get("m1"); //m1
        leftMotor = hardwareMap.dcMotor.get("m2"); //m2
        backMotor = hardwareMap.dcMotor.get("m3"); //m3
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //m4
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //m5
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //m6
        relicMotor = hardwareMap.dcMotor.get("m7"); //m7

        //Servo configurations in the hardware map. Used so that the robot controller can apply the correct servo positions depending on what port a servo is plugged into.
        leftPanSpin = hardwareMap.servo.get("s1"); //s1
        rightPanSpin = hardwareMap.servo.get("s2"); //s2
        grab = hardwareMap.servo.get("s6"); //s6
        feelerRaise = hardwareMap.servo.get("s8"); //s8
        upDown = hardwareMap.servo.get("s11"); //s11
        touchServo = hardwareMap.servo.get("s10"); //s10

        //Code to extend the upDown Servo past 180 degrees
        ServoControllerEx theControl = (ServoControllerEx) upDown.getController(); //Declare ServoController "the Control" and specify that upDown is the Servo that will have an extended range
        int thePort = upDown.getPortNumber(); //Declare integer thePort and set it to the port number of upDown, to specify which port will be extended
        PwmControl.PwmRange theRange = new PwmControl.PwmRange(353, 2700); //Specify the extended pwm range of the servo: 353 to 2700
        theControl.setServoPwmRange(thePort, theRange); //Set the extended range to theControl, specifying the port of the extended range servo and the value of this range

        //Digital channel configurations in the hardware map. Used so that the robot controller can apply the correct values to digital channels depending on what port a digital channel is plugged into.
        limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //d1
        limitBottom = hardwareMap.get(DigitalChannel.class, "d2"); //d2

        //Distance sensor configurations in the hardware map. Used so that the robot controller can apply the correct values to distance sensors depending on what port a distance sensor is plugged into.
        sensorA = hardwareMap.get(DistanceSensor.class, "i2"); //i2
        sensorB = hardwareMap.get(DistanceSensor.class, "i3"); //i3
        sensorC = hardwareMap.get(DistanceSensor.class, "i4"); //i4

        //Specify motor directions to ensure values given to motors cause rotation in the correct direction
        rightMotor.setDirection(REVERSE); //Set rightMotor to REVERSE direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to FORWARD direction
        backMotor.setDirection(REVERSE); //Set backMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to REVERSE direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(REVERSE); //Set panLifterMotor to REVERSE direction
        relicMotor.setDirection(REVERSE); //Set relicMotor to REVERSE direction

        //Specify servo directions to ensure values given to servos cause rotation in the correct direction
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        grab.setDirection(Servo.Direction.REVERSE); //Set grab to REVERSE direction
        upDown.setDirection(Servo.Direction.FORWARD); //Set upDown to FORWARD direction
        feelerRaise.setDirection(Servo.Direction.FORWARD); //Set feelerRaise to FORWARD direction
        touchServo.setDirection(Servo.Direction.REVERSE); //Set touchServo to REVERSE direction

        //Init values of servos to ensure at the beginning of teleop servos are in the correct position
        leftPanSpin.setPosition(.2575); //Set leftPanSpin to position .1875
        rightPanSpin.setPosition(.2725); //Set rightPanSpin to position .2075
        upDown.setPosition(0); //Set upDown to position 0
        touchServo.setPosition(.69); //Set touchServo to position .65

        highGain = true;
        lowGain = false;
        gainToggle = true;

        leftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop () {

        leftStickX1Raw = gamepad1.left_stick_x;
        rightStickX1Raw = gamepad1.right_stick_x;
        rightStickY1Raw = gamepad1.right_stick_y;

        sensorAVal=sensorA.getDistance(DistanceUnit.CM);
        sensorBVal=sensorB.getDistance(DistanceUnit.CM);
        sensorCVal=sensorC.getDistance(DistanceUnit.CM);


        //TRANSPORTING
        //Controller Inputs

        rightTrigger = gamepad1.right_trigger; //Set variable rightTrigger to the raw double value of the right trigger
        rightBumper = gamepad1.right_bumper; //Set variable rightBumper to the raw boolean value of the right bumper
        leftTrigger = gamepad1.left_trigger; //Set variable leftTrigger to the raw double value of the left trigger
        leftBumper = gamepad1.left_bumper; //Set variable leftBumper to the raw boolean value of the left bumper
        dpadLeftPressed = gamepad1.dpad_left; //Set variable dpadLeftPressed to the raw boolean value of the left dpad
        dpadRightPressed = gamepad1.dpad_right; //Set variable dpadRightPressed to the raw boolean value of the right dpad
        dpadUpPressed = gamepad1.dpad_up; //Set variable dpadUpPressed to the raw boolean value of the up dpad
        dpadDownPressed = gamepad1.dpad_down; //Set variable dpadDownPressed to the raw boolean value of the down dpad
        aPressed =gamepad1.a; //Set variable aPressed to the raw boolean value of the a button
        yPressed = gamepad1.y; //Set variable yPressed to the raw boolean value of the y button
        xPressed = gamepad1.x; //Set boolean xPressed to gamepad1.x
        bPressed = gamepad1.b; //Set boolean bPressed to gamepad1.b
        rightStickButton = gamepad1.right_stick_button;
        leftStickButton = gamepad1.left_stick_button;

        if (sensorAVal < 13 && sensorBVal < 13 && sensorCVal < 13)
        {
            threeGlyphCounter++;
        }
        else
        {
            threeGlyphCounter/=2;
        }

        if (sensorBVal < 13 && sensorCVal < 13)
        {
            bothBlockCounter++;
        }
        else
        {
            bothBlockCounter/=2;
        }

        if (threeGlyphCounter>40)
        {
            threeGlyphs=true;
        }
        else
        {
            threeGlyphs=false;
        }

        if(dpadRightPressed) //If dpadRightPressed is true
        {
            if (!previousStatus) //If previousStatus is false
            {
                currentStatus = true; //Set currentStatus to true
            }
            else //Else
            {
                currentStatus = false; //Set currentStatus to false
            }
        }
        else //Else
        {
            previousStatus = currentStatus; //Set previousStatus to currentStatus
        }

        //Telemetry



        //Set driving variables to values specified in the variable declaration section

        finRightPower = -1*rightStickY1Raw-.2*rightStickX1Raw-.5*leftStickX1Raw;
        finLeftPower = -1*rightStickY1Raw +.2*rightStickX1Raw +.5*leftStickX1Raw;
        finBackPower = rightStickX1Raw;


        if (!currentStatus) { //If currentStatus is false

            if (leftStickButton)
            {
                panSpinPosition-=.08;
            }

            if (bPressed && !dpadLeftPressed)
            {
                touchServoPosition=.95;
            }

            if (leftBumper && !aPressed && !yPressed && !xPressed && !dpadUpPressed)
            {
                toggleLB = true;
            }
            else if (aPressed || yPressed || xPressed || dpadUpPressed || rightBumper)
            {
                toggleLB = false;
            }

            if (rightBumper && !dpadUpPressed && !dpadDownPressed && !aPressed && !leftBumper && !xPressed && !yPressed && !bPressed)
            {
                score = true;
            }
            else if (dpadUpPressed || dpadDownPressed || aPressed || bPressed || xPressed || yPressed || leftBumper)
            {
                score = false;
            }

            if (threeGlyphs)
            {
                leftIntakePower=-.95;
                rightIntakePower=-.95;
            }


            if (rightTrigger > .05 && leftTrigger > .05 && !threeGlyphs) { //If both rightTrigger and leftTrigger are pressed, set intake powers to 0, as this is a conflicting command
                leftIntakePower = 0; //Set leftIntakePower to 0
                rightIntakePower = 0; //Set rightIntakePower to 0
            } else if (rightTrigger > .05 && !bPressed && !leftBumper && !dpadLeftPressed && !threeGlyphs) { //Else if rightTrigger if pressed
                leftIntakePower = Math.pow(rightTrigger, 2) * .7; //Set leftIntakePower to the square of rightTrigger times .75
                rightIntakePower = Math.pow(rightTrigger, 2) * .7; //Set rightIntakePower to the square of rightTrigger times .85
                touchServoPosition=.74;
            } else if (leftTrigger > .05 && !bPressed && !leftBumper && !dpadLeftPressed && !threeGlyphs) { //Else if leftTrigger is pressed
                leftIntakePower = -.7; //Set leftIntakePower to -.7
                rightIntakePower = -.8; //Set rightIntakePower to -.7
                touchServoPosition=.78;
            } else { //Else
                if (!threeGlyphs) {
                    leftIntakePower = 0; //Set leftIntakePower to 0
                    rightIntakePower = 0; //Set rightIntakePower to 0
                }
            }


            //If the state of limit Top is false and dpad up is pressed, or the state of limit Bottom is false and dpad down is pressed
            if (!limitTop.getState() && score)
            {
                panLiftingPower = 0; //Set panLiftingPower to 0
                hasLifted = true;
            }
            else if (!limitBottom.getState() && (dpadDownPressed||toggleLB))
            {
                panLiftingPower=0;
            }
            else if (dpadUpPressed && !dpadDownPressed || (score && !hasLifted)) //MIGHT NEED TO MAKE THIS AN IF...
            {
                panLiftingPower = -.55; //Set panLiftingPower to -.69
                if (score && !hasLifted && !dpadLeftPressed)
                {
                    touchServoPosition=.70;
                }
            }
            else if ((dpadDownPressed && !dpadUpPressed) || toggleLB) //Else if dpad down is pressed and dpad up is not pressed
            {
                if (toggleLB)
                {
                    panLiftingPower = .25;
                }
                else
                {
                    panLiftingPower = .5; //Set panLiftingPower to .95
                }

            }
            else //Else
            {
                panLiftingPower = 0; //Set panLiftingPower to 0
            }

            if ((xPressed || bothBlockCounter>7) && !score && !leftStickButton && !aPressed && !bPressed && !dpadLeftPressed && !leftBumper && !toggleLB && panSpinPosition!=.8225) //Else if x is pressed
            {
                panSpinPosition = .42; //Set panSpinPosition to .3
                if (!(leftTrigger>.2)) {
                    touchServoPosition=.835;
                }
            }

            telemetry.addData("score", score);
            telemetry.addData("aPress", aPressed);
            telemetry.addData("bPressed", bPressed);
            telemetry.addData("dpadleftpressed", dpadLeftPressed);
            telemetry.addData("lb", leftBumper);
            telemetry.addData("togglelb", toggleLB);
            telemetry.addData("panSpinpos", panSpinPosition);
            telemetry.addData("hasLifted", hasLifted);
            telemetry.addData("bothBlockCounter", bothBlockCounter);
            telemetry.addData("touchServo", touchServoPosition);

            if (!toggleLB && !score && !leftStickButton) {

                if (yPressed && aPressed) //If y and a are pressed
                {
                    //Do nothing due to conflicting commands
                } else if (yPressed) //Else if y is pressed
                {
                    panSpinPosition = panSpinPosition + .05; //Add .05 to panSpinPosition
                } else if (aPressed) //Else if a is pressed
                {
                    if (leftPanSpin.getPosition() > .37) {
                        panSpinPosition-= .05; //Subtract .05 from panSpinPosition
                    } else {
                        panSpinPosition-= .025; //Subtract .025 from panSpinPosition
                    }

                }
            }
            else if (toggleLB && !score && !leftBumper && !aPressed && !leftStickButton) //Else if b is pressed
            {
                if (leftPanSpin.getPosition() > .37 && bothBlockCounter<=7) {
                    panSpinPosition-= .07; //Subtract .05 from panSpinPosition
                } else if (bothBlockCounter<=7){
                    panSpinPosition-= .05; //Subtract .025 from panSpinPosition
                }

                if (panSpinPosition<.25)
                {
                    toggleLB=false;
                }
            }
            
            if (score && !toggleLB && !leftStickButton)
            {
                if (hasLifted && !leftBumper) {
                    panLiftingPower=0;
                    panSpinPosition = .8225;
                    score=false;
                    hasLifted = false;
                }
            }

            feelerRaisePosition=FEELER_RAISE_UP_POSITION;

        }

        //Else
        else
        {
            if (!leftBumper) {
                touchServoPosition=.69;
            }
            else
            {
                touchServoPosition=.4;
            }

            if (dpadUpPressed) //If dPadUpPressed is true
            {
                relicMotorPower=.98;
            }
            else if (dpadDownPressed) //Else if dpadDownPressed is true
            {
                relicMotorPower=-.98;
            }
            else
            {
                relicMotorPower=0;
            }

            if (rightTrigger > .05  && leftTrigger > .05) //If rightTrigger and leftTrigger are adequately pressed
            {
                //Do nothing due to conflicting commands
            }
            else if (leftTrigger>.05 && !xPressed) //Else if leftTrigger is adequately pressed
            {
                grabPosition+=.035 * leftTrigger; //Add .01 times the value of leftTrigger to grabPosition
            }
            else if (rightTrigger>.05 && !xPressed)
            {
                grabPosition-=.035 * rightTrigger; //Subtract .01 times the value of leftTrigger to grabPosition
            }
            else //Else
            {

            }


            if (yPressed && aPressed) //If yPressed and aPressed are true
            {
                //Do nothing due to conflicting commands
            }
            else if (aPressed) //Else if aPressed is true
            {
                if (upDown.getPosition() < .69) //If the position of upDown is less than .69
                {
                    upDownPosition+=.0075; //Add .003 to upDownPosition
                }
                else //Else
                {
                    upDownPosition+=.005; //Add .001 to upDownPosition
                }

                telemetry.addData("upDown", upDownPosition); //Telemetry

            }
            else if (yPressed) //Else if yPressed is true
            {
                upDownPosition-=.01; //Subract .003 from upDownPosition
            }
            else if (xPressed)
            {
                upDownPosition=.77;
                grabPosition=.4;
            }
            else if (bPressed)
            {
                if (upDownPosition>.5)
                {
                    upDownPosition-=.01;
                }
            }
            else if (rightBumper)
            {
                upDownPosition=.82;
                grabPosition=.4;
            }
            else //Else
            {
                //Do nothing
            }

            if (rightStickButton)
            {
                relicMotor.setPower(.99);
            }

            feelerRaisePosition = FEELER_RAISE_ENDGAME_POSITION;

        }

        //SET CONTROLS

        bothBlockCounter=Range.clip(bothBlockCounter, 0, 500);
        threeGlyphCounter=Range.clip(threeGlyphCounter,0,500);
        grabPosition=Range.clip(grabPosition, .1, .45); //Ensure grabPosition is between .1 and .7
        upDownPosition=Range.clip(upDownPosition, .02, .83); //Ensure upDownPosition is between .05 and .83
        panSpinPosition = Range.clip(panSpinPosition, .245, .825); //Ensure panSpinPosition is between .27 and .59
        //Clip final driving motor values between -1 and 1
        finBackPower = Range.clip(finBackPower, -1, 1);
        finRightPower = Range.clip(finRightPower, -1, 1);
        finLeftPower = Range.clip(finLeftPower, -1, 1);


        leftPanSpin.setPosition(panSpinPosition); //Set the position of leftPanSpin to panSpinPosition
        rightPanSpin.setPosition(panSpinPosition + .015); //Set the position of rightPanSpin to panSpinPosition plus .015
        panLifterMotor.setPower(panLiftingPower); //Set the power of panLifterMotor to panLiftingPower
        leftIntakeMotor.setPower(leftIntakePower); //Set the power of leftIntakeMotor to leftIntakePower
        rightIntakeMotor.setPower(rightIntakePower); //Set the power of rightIntakeMotor to rightIntakePower
        leftMotor.setPower(finLeftPower); //Set the power of leftMotor to finLeftPower
        rightMotor.setPower(finRightPower); //Set the power of rightMotor to finRightPower
        backMotor.setPower(finBackPower); //Set the power of backMotor to finBackPower
        grab.setPosition(grabPosition); //Set the power of grab to grabPosition
        upDown.setPosition(upDownPosition); //Set the power of upDown to upDownPosition
        touchServo.setPosition(touchServoPosition);
        feelerRaise.setPosition(feelerRaisePosition);
        relicMotor.setPower(relicMotorPower);

        //Update telemetry
        telemetry.update();
    } // end loop

} // End class
