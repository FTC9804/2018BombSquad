//TELEOP
//to do remove either left or right spin, explain automation vs. manual, and the purpose of teleop, define bar, pan

//.23 is open for back grabber
//.77 is closed for back grabber

//.25 is open for front grabber
//.97 is closed for front grabber

//left pan spin is .1 lower than right pan spin
//add .1 to left pan spin to find equalibrium


//Package statement
package org.firstinspires.ftc.teamcode;

//Import statements, used to ensure that all commands in our code are supported
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

//Class declaration. Public class called Teleop Limits that extends the OpMode Class in the First Tech Challenge SDK
public class TeleopLimits extends OpMode {

    //MOTORS

    //Driving
    DcMotor rightMotor; //Right drive motor, for driving forwards and backwards
    DcMotor leftMotor;  //Left drive motor, for driving forwards and backwards
    DcMotor backMotor;  //Back drive motor, for driving sideways, a.k.a "strafing"

    //Relic
    DcMotor relicMotor; //Motor to extend the relic scoring mechanism to score the relic

    int loopCounter;

    //Intake
    DcMotor rightIntakeMotor; //Motor that controls the right intake/right wheel of the intake, used to intake glyphs
    DcMotor leftIntakeMotor; //Motor that controls the left intake/left wheel of the intake, used to intake glyphs

    //Block Scoring
    DcMotor panLifterMotor; //Motor that lifts and lowers the block scoring mechanism, known as the "pan", to score blocks in the cryptobox

    //SERVOS

    //Block Rotation
    Servo leftPanSpin; //Servo on the left side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo rightPanSpin;
    Servo frontPanGrip; //Servo that moves the grip on the left side of the pan
    Servo backPanGrip; //Servo that moves the grip on the right side of the pan


    //Relic
    Servo upDown; //Servo to raise the relic mechanism up and down, in order to balance the relic in the third zone
    Servo grab; //Servo to grab the relic

    //Glyphs
    Servo touchServo; //Servo that extends an arm from the front of the robot to detect when we are ready to score in autonomous
    //The arm is attached to a metal bar to block glyphs from entering the robot incorrectly in teleop

    //Ball Scoring
    Servo feelerRaise; //Servo that lifts and lowers the ball scoring mechanism, known as the "feeler"

    //BLOCK SENSORS

    //Distance sensors to detect glyphs. If sensor B and sensor C see blocks, that means we are ready to score, so we lift up the pan.
    //If all three see glyphs for too long, that means we have three glyphs, so we outtake as this is a penalty.
    //  DistanceSensor sensorA; //Distance sensor closest to the intake to see how far away potential blocks are
    DistanceSensor sensorB; //Distance sensor between A and C, to see how far potential blocks are
    DistanceSensor sensorC; //Distance sensor farthest from intake, to see how far potential blocks are

    //Limit Switches on the elevator. We use these switches to ensure that the elevator does not go to low or to high.
    DigitalChannel limitTop; //Limit Switch that tells us if we reach the top of the robot with the elevator, which is the scoring position for scoring the third and fourth blocks of a column
    DigitalChannel limitBottom; //Limit switch that tells us if we reach the bottom of the robot with the elevator
    DigitalChannel limitMid; //Limit Switch that tells us if we reach the middle of the elevator, which is the scoring position for the first and secnd blocks of a column

    /*
    //CONTROLS, all for gamepad 1
    double leftStickX1Raw; //The raw value of the x-axis of the leftStick, used for determining if and how much the robot should rotate while driving
    double rightStickX1Raw; //The raw value of the x-axis of the rightStick, used for determining if and how much the robot should strafe while driving
    double rightStickY1Raw; //The raw value of the y-axis of the rightStick, used for determining if and how much the robot should drive forwards/backwards while driving
    double rightTrigger; //double for the extent to which rightTrigger is pressed. No press is 0, full press is 1. Used for intaking blocks in teleop and grabbing the relic in endgame.
     boolean rightBumper; //boolean for rightBumper. Set to true if rightBumper is pressed and set to false otherwise. Used to score blocks in teleop.
     double leftTrigger; //double for the extent to which leftTrigger is pressed. No press is 0, full press is 1. Used for outtaking blocks in teleop and releasing the relic in endgame.
     boolean leftBumper; //boolean for leftBumper. Set to true if leftBumper is pressed and set to false otherwise. Used to lower the elevator and pan, or block scoring mechanism, in teleop.
     boolean dpadDownPressed; //boolean for dpadDown. Set to true if dpadDown is pressed and set to false otherwise. Used to lower the elevator in teleop and to retract the relic scoring mechanism in endgame.
     boolean dpadUpPressed; //boolean for dpadUp. Set to true if dpadUp is pressed and set to false otherwise. Used to raise the elevator in teleop and to extend the relic scoring mechanism in endgame.
     boolean yPressed; //boolean for the y button.  Set to true if y is pressed and set to false otherwise. Used to move the pan up in teleop and to raise the relic grabbing arm in endgame.
     boolean aPressed; //boolean for the a button.  Set to true if a is pressed and set to false otherwise. Used to move the pan down in teleop and lowers the relic grabbing arm in endgame.
     boolean xPressed; //boolean for the x button.  Set to true if x is pressed and set to false otherwise. Used to move the pan up to a set block holding position in teleop and moves the relic arm to a
     //set "ready to score" position in endgame, slightly above where the arm will be when the relic is scored.
     boolean bPressed; //boolean for the b button.  Set to true if b is pressed and set to false otherwise. Used to lower the pan to its lowest position in teleop.
     */boolean rightStickButton; //boolean for the left stick button. Set to true if the right stick button is pressed and set to false otherwise. Used to extend the relic motor in endgame while Kevin (our driver) is driving, so he can keep his hands on the joysticks
    //  boolean dpadRightPressed; //boolean for dpadRight. Set to true if dpadRight is pressed and set to false otherwise. Used to enter in and out of the "endgame" mode, where buttons control the relic rather than the glyph intaking

    //Driving variables
    double finLeftPower; //The final power to be applied to leftMotor, one of our driving motors which makes the robot drive forwards or backwards, or rotate
    double finRightPower; //The final power to be applied to rightMotor, one of our driving motors which makes the robot drive forwards or backwards, or rotate
    double finBackPower; //The final power to be applied to backMotor, one of our driving motors which makes the robot strafe

    //Glyph variables
    //   double leftIntakePower; //The power to which we set leftIntakeMotor, one of the motors to collect glyphs
    //  double rightIntakePower; //The power to which we set rightIntakeMotor, one of the motors to collect glyphs
    // double panLiftingPower; //The power to which we set panLifterMotor, the motor that raises glyphs for scoring in the cryptobox
    double panSpinPosition; //The position to which we set leftPanSpin the servo that rotates the pan to score in the cryptobox
    double panGripPosition;

    //Feeler variables
    double feelerRaisePosition; //Position that feelerRaise will be set to
    final double FEELER_RAISE_UP_POSITION = .91; //Position that the feelerRaise is set to in teleop, which causes the jewel arm mechanism to be pushed against the body of the robot
    final double FEELER_RAISE_ENDGAME_POSITION = .85; //Position that the feelerRaise is set to in endgame, lower than FEELER_RAISE_UP_POSITION as to not interfere with the relic arm

    //Mode booleans. Used for toggling between regular teleop mode and endgame mode. We use modes as we are short on buttons as
    //we only have one driver. Thus, depending on what mode we are in, the same buttons preform different functions.
    boolean previousStatus; //Boolean that is true if the previous mode was endgame, and is false otherwise
    boolean currentStatus; //Boolean that is true if the current mode is endgame, and is false otherwise
    double bothBlockCounter; //Double that increments positively by 1 in the loop every time we see a block on sensors b and c, and decreased otherwise.
    //If bothBlockCounter is high enough, we know we are ready to score and we set the pan servos to a hold position
    int blockCounterThreshold = 20; //Number of loops to indicate glyphs present in the pan
    int blockGrabberThreshold = 10;
    boolean score; //Boolean for automated scoring. Boolean that is set to true if rightBumper is pressed. When score is true, that means we would like to score glyphs, and the elevator raises and the pan servos are set to a scoring position, and score is set to false
    boolean hasLifted = false; //Boolean to complement score that is initially set to false. If score is true, then the elevator lifts until the top limit switch sees the elevator, at which time hasLifted is set to true and the elevator stops moving upwards
    //boolean toggleLB; //Boolean for automated lowering of the pan and elevator after we score. If the leftbumper is pressed, toggleLB is set to true, and the pan servos rotate to an intaking position, while the elevator lowers until the bottom limit switch sees it. Once these actions occur, toggleLB is set to false

    //Distance sensor variables, used to see how far away any block(s) are from each of the distance sensors on the pan
    //double sensorAVal; //The distance value of sensorA, to be measured in centimeters
    // double sensorBVal; //The distance value of sensorB, to be measured in centimeters
    //   double sensorCVal; //The distance value of sensorC, to be measured in centimeters

    //Relic variables
    double relicMotorPower; //The value that will be set to the relicMotor. If we are extending the relic, it will be positive, and if we are retracting the relic it will be negative
    double grabPosition; //The value that will be set to the grab servo. Higher values indicate that grab is closed, lower values indicate that grab is open
    double upDownPosition; //The value that will be set to the upDown servo. Higher values indicate that upDown is lower, lower values indicate that upDown is higher

    //Touch servo variables
    // double touchServoPosition = .59; //The value that will be set to the touchServo, initially set to .69. Lower values indicate that the bar, which the touchServo controls, is closer to the tiles, and visa versa

    boolean isAtTop;
    boolean isAtMiddle;
    boolean isAtBottom;

    final double PAN_SPIN_INCREMENT_DOWN = .0375;
    final double PAN_SPIN_INCREMENT_UP = .05;

    double timeOne; //timeOne, first time variable
    double timeTwo; //timeTwo, second time variable

    double startTime;
    //double stopTime;

    double frontBlockCounter;
    double backBlockCounter;

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
        rightPanSpin = hardwareMap.servo.get("s2");
        grab = hardwareMap.servo.get("s6"); //s6
        feelerRaise = hardwareMap.servo.get("s8"); //s8
        upDown = hardwareMap.servo.get("s11"); //s11
        touchServo = hardwareMap.servo.get("s10"); //s10
        frontPanGrip = hardwareMap.servo.get("s12"); //s12
        backPanGrip = hardwareMap.servo.get("s13"); //s13


        //Code to extend the upDown Servo past 180 degrees
        ServoControllerEx theControl = (ServoControllerEx) upDown.getController(); //Declare ServoController "the Control" and specify that upDown is the Servo that will have an extended range
        int thePort = upDown.getPortNumber(); //Declare integer thePort and set it to the port number of upDown, to specify which port will be extended
        PwmControl.PwmRange theRange = new PwmControl.PwmRange(353, 2700); //Specify the extended pwm range of the servo: 353 to 2700
        theControl.setServoPwmRange(thePort, theRange); //Set the extended range to theControl, specifying the port of the extended range servo and the value of this range

        //Digital channel configurations in the hardware map. Used so that the robot controller can apply the correct values to digital channels depending on what port a digital channel is plugged into.
        limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //d1
        limitBottom = hardwareMap.get(DigitalChannel.class, "d2"); //d2
        limitMid = hardwareMap.get(DigitalChannel.class, "d3"); //d3

        //Distance sensor configurations in the hardware map. Used so that the robot controller can apply the correct values to distance sensors depending on what port a distance sensor is plugged into.
        // sensorA = hardwareMap.get(DistanceSensor.class, "i2"); //i2
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
        leftPanSpin.setDirection(Servo.Direction.FORWARD); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.REVERSE);
        grab.setDirection(Servo.Direction.REVERSE); //Set grab to REVERSE direction
        upDown.setDirection(Servo.Direction.FORWARD); //Set upDown to FORWARD direction
        feelerRaise.setDirection(Servo.Direction.FORWARD); //Set feelerRaise to FORWARD direction
        touchServo.setDirection(Servo.Direction.REVERSE); //Set touchServo to REVERSE direction
        frontPanGrip.setDirection(Servo.Direction.FORWARD); //Set servo leftPanGrip to FORWARD direction
        backPanGrip.setDirection(Servo.Direction.FORWARD); //Set rightPanGrip to FORWARD direction

        //Init values of servos to ensure at the beginning of teleop servos are in the correct position
        leftPanSpin.setPosition(.525); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.586);
        upDown.setPosition(0); //Set upDown to position 0, so our relic arm stays within the robot
        frontPanGrip.setPosition(.25); //Set the position of leftPanGrip and rightPanGrip to 0, so we are able to intake glyphs
        backPanGrip.setPosition(.23);




        //Set the run modes of leftIntakeMotor, rightIntakeMotor, panLifterMotor, and relicMotor to STOP_AND_RESET_ENCODER, and then RUN_USING_ENCODER
        //The first command will reset the encoders on the specified motors, and the second will implement PID control so each motor will always rotate at the
        //same speed regardless of battery power. For the intake motors, this means each intake wheel will rotate at the same power which will allow for easier
        //glyph collection. For the relicMotor and panLifterMotor, PID will allow Kevin to be consistent in his relic and elevator control.
        leftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //panLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //panLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void start ()
    {
        startTime = this.getRuntime();
        timeOne=this.getRuntime();
        touchServo.setPosition(.59); //Set touchServo to position .59, so the bar will not yet extend outside of the framework of the robot to help prevent faulty glyphs from entering

    }

    public void loop() { //Loop method to assign values to motors, servos, and sensors

        loopCounter++;

        //CONTROLLER INPUTS


        //   rightTrigger = gamepad1.right_trigger; //Set variable rightTrigger to the raw double value of the right trigger
        // rightBumper = gamepad1.right_bumper; //Set variable rightBumper to the raw boolean value of the right bumper
        //  leftTrigger = gamepad1.left_trigger; //Set variable leftTrigger to the raw double value of the left trigger
        //leftBumper = gamepad1.left_bumper; //Set variable leftBumper to the raw boolean value of the left bumper
//        leftStickX1Raw = gamepad1.left_stick_x; //Set leftStickX1Raw to gamepad1.left_stick_x, the value which will deterine if and to what extent the robot will rotate
//        rightStickX1Raw = gamepad1.right_stick_x; //Set rightStickX1Raw to gamepad1.right_stick_x, the value which will deterine if and to what extent the robot will strafe
//        rightStickY1Raw = gamepad1.right_stick_y; //Set rightStickY1Raw to gamepad1.right_stick_y, the value which will deterine if and to what extent the robot will drive forwards or backwards
        // sensorAVal = sensorA.getDistance(DistanceUnit.CM); //Set sensorAVal to the centimeter distance of sensor A
        // sensorBVal = sensorB.getDistance(DistanceUnit.CM); //Set sensorBVal to the centimeter distance of sensor B
        // sensorCVal = sensorC.getDistance(DistanceUnit.CM); //Set sensorCVal to the centimeter distance of sensor C
        // dpadRightPressed = gamepad1.dpad_right; //Set variable dpadRightPressed to the raw boolean value of the right dpad
        // dpadUpPressed = gamepad1.dpad_up; //Set variable dpadUpPressed to the raw boolean value of the up dpad
        // dpadDownPressed = gamepad1.dpad_down; //Set variable dpadDownPressed to the raw boolean value of the down dpad
        // aPressed = gamepad1.a; //Set boolean aPressed to the raw boolean value of the a button
        //yPressed = gamepad1.y; //Set boolean yPressed to the raw boolean value of the y button
        // xPressed = gamepad1.x; //Set boolean xPressed to the raw boolean value of gamepad1.x
        // bPressed = gamepad1.b; //Set boolean bPressed to the raw boolean value of gamepad1.b

        if (currentStatus) { //Right stick button is only used in endgame, so we only check its value (to save cycle time) if currentStatus is true, which means we are in endgame
            rightStickButton = gamepad1.right_stick_button; //Set boolean value of rightStickButton to the raw boolean value of gamepad1.right_stick_button
        }

        //Relating to the limit switches on the elevator, if a sensor's getState is true, the sensor is not detecting anything
        if (score)
        {
            isAtTop = !limitTop.getState();
        }
        else if (leftIntakeMotor.getPower()>0 || gamepad1.dpad_down)
        {
            isAtBottom = !limitBottom.getState();
        }
        else
        {
            isAtTop = false;
            isAtMiddle = false;
            isAtBottom = false;
        }

        //Glyph Sensing


        //Toggle to alternate between regular teleop mode and endgame mode
        if (gamepad1.dpad_right) //If dpadRightPressed is true, signifying Kevin would like to alternate between modes
        {
            if (!previousStatus) //If previousStatus is false, meaning the previous mode was regular teleop
            {
                currentStatus = true; //Set currentStatus to true, meaning that we are now in endgame mode
            } else //Else
            {
                currentStatus = false; //Set currentStatus to false, meaning that we are now in regular teleop mode
            }
        } else //Else
        {
            previousStatus = currentStatus; //Set previousStatus to currentStatus if dpadRightPressed is not true, meaning we would not like to change any mode boolean values as Kevin is not toggling
        }


        //Set driving variables to values specified in the variable declaration section

        finRightPower = -1 * gamepad1.right_stick_y - .2 * gamepad1.right_stick_x - .58 * gamepad1.left_stick_x; //Set finRightPower, taking into account driving variables from each driving axis
        finLeftPower = -1 * gamepad1.right_stick_y + .2 * gamepad1.right_stick_x + .58 * gamepad1.left_stick_x; //Set finLeftPower, taking into account driving variables from each driving axis

        finBackPower = gamepad1.right_stick_x; //Sin finBackPower to the raw value of the x axis of the right stick, for strafing

        if (finLeftPower/finRightPower<0 && currentStatus) //If we are in endgame, and finLeftPower and finRightPower are opposite signs, meaning we are rotating, decrease the power of rotation to allow Kevin to have finer rotation controls when trying to grab the relic
        {
            finLeftPower/=1.4; //Divide finLeftPower by 1.5
            finRightPower/=1.4; //Divide finRightPower by 1.5
        }

        if (!currentStatus) { //If currentStatus is false, meaning we are in regular teleop mode

            //If leftBumper is being pressed, Kevin wants to lower the pan and the elevator. We also make sure that no buttons of choice that also control the pan or elevator are
            //being pressed (a, y, x, dpad up, and dpad down) as to avoid giving conflicting values to servos or motors

            if (gamepad1.left_bumper)
            {
                score = true;
            }

            if (gamepad1.right_bumper)
            {
                frontPanGrip.setPosition(.280);
                backPanGrip.setPosition(.112);
                score = false;
            }

            if (gamepad1.right_trigger > .05) { //If rightTrigger if pressed, and b and leftBumper are not pressed, as to avoid conflicting commands for the intake powers
                leftIntakeMotor.setPower(gamepad1.right_trigger * gamepad1.right_trigger * .7); ; //Set leftIntakePower to the square of rightTrigger times .7. We square values so Kevin can have finer control over intake speeds
                rightIntakeMotor.setPower(gamepad1.right_trigger * gamepad1.right_trigger * .7); //Set rightIntakePower to leftIntakePower
                touchServo.setPosition(.44); //Set touchServoPosition to .45, which will set the touchServo to a position so the bar is just above 6 inches above the tiles, so good glyphs can enter the robot, but faulty glyphs cannot enter above where they should
                frontPanGrip.setPosition(.258);
                backPanGrip.setPosition(.112);
                panSpinPosition=.525;
            } else if (gamepad1.left_trigger > .05) { //Else if leftTrigger is pressed, and b and leftBumper are not pressed, as to avoid conflicting commands for the intake powers
                leftIntakeMotor.setPower(-.7); //Set leftIntakePower to -.7. We set outtake powers differently so we can realign glyphs for reentry rather than outtaking them in the same orientation at which they entered
                rightIntakeMotor.setPower(-.8); //Set rightIntakePower to -.8. We set outtake powers differently so we can realign glyphs for reentry rather than outtaking them in the same orientation at which they entered
                touchServo.setPosition(.48); //Set touchServoPosition to .48, which will set the touchServo to a position so the bar is just above the intake position, .74, to give exiting glyphs slightly more room to exit the robot
            } else { //Else
                rightIntakeMotor.setPower(0); //Set leftIntakePower to 0
                leftIntakeMotor.setPower(0);  //Set rightIntakePower to 0

            }

            //ELEVATOR CONTROL

            if (score) //If score is true, we check to see if we are at limit switches and whether we want to score
            {
                if (isAtTop) { //If isAtTop, then we want to score
                    hasLifted = true; //Set hasLifted to true
                    score = false;
                    panLifterMotor.setPower(0); //Set panLiftingPower to 0 to stop the motion of the elevator
                }
                else {
                    panLifterMotor.setPower(-.6);  //Set panLiftingPower to -.16, to raise the elevator
                    touchServo.setPosition(.45 ); //Set touchServoPosition to .70, which is higher than the outtaking and intaking positions, as to put the bar close to the blocks in the pan so they cannot fall out
                }
            }
            else if (gamepad1.dpad_up) //If dpad up is pressed and dpad down is not, signifying we want to lift the elevator, or score is true and hasLifted is not, signifying we want to raise the elevator and then score blocks
            {
                panLifterMotor.setPower(-.6); ; //Set panLiftingPower to -.16, to raise the elevator
            }
            else if (gamepad1.dpad_down && !isAtBottom) //If we have reached the bottom limit switch, and we are lowering the elevator through dpadDown or from pressing leftbumper
            {
                panLifterMotor.setPower(.2); //Set panLiftingPower to .2 to go down with the elevator
            }
            else if (gamepad1.right_trigger > .05)
            {
                if (isAtBottom)
                {
                    panLifterMotor.setPower(0);
                }
                else
                {
                    panLifterMotor.setPower(.5);
                }
            }
            else //Else, if none of the elevator movement commands are being applied on the joystick
            {
                panLifterMotor.setPower(0); //Set panLiftingPower to 0
            }

            if (gamepad1.y) //Else if y is pressed, we want to raise the pan
            {
                panSpinPosition += .02; //Add .05 to panSpinPosition
            }
            else if (gamepad1.a) //Else if a is pressed
            {
                panSpinPosition -= .02; //Subtract .04 from panSpinPosition to lower the pan
            }


            if (gamepad1.b) { //If b is pressed, we set the touchServo to a position so the bar will press against glyphs in the pan so they are secure
                touchServo.setPosition (.6); //Set touchServoPosition to .6
            }

            telemetry.addData("bothBlockCounter", bothBlockCounter);
            telemetry.addData("blockGrabberThreshold", blockGrabberThreshold);
            telemetry.addData("score", score);
            telemetry.addData("b", gamepad1.b);
            telemetry.addData("pan spin", panSpinPosition);

            //If x is pressed or sensors b and c have seen blocks for more than blockCounterThreshold loop iterations, we want to adjust the pan to a hold, rather than intake or score, position
            //We also make sure that no other commands that control panSpinPosition are being applied to avoid conflicting values
            if ((gamepad1.x || (bothBlockCounter > blockGrabberThreshold)) && !score && panSpinPosition < .6)            {
                frontPanGrip.setPosition(.964);
                backPanGrip.setPosition(.812);

            }
            if ((gamepad1.b || (bothBlockCounter > blockCounterThreshold)) && !score && panSpinPosition < .6 && !gamepad1.a && !gamepad1.y)            {

                panSpinPosition = .93; //Set panSpinPosition to .82, a hold position
                if (gamepad1.left_trigger <= .2) { //If we are not outtaking (which is what leftTrigger does) we want to set the touchServo/bar to a position close to the blocks that are holding so they do not fall out
                    //We include this statement because even if we have 2 glyphs Kevin may want to outtake to realign them
                    touchServo.setPosition (.45); //Set touchServoPosition to .53 to prent holding glyphs from exiting hte pan
                }
            }
            if (!score) {
                //If pan distance sensors b and c see an object within 13 centimeters. If this is true, we have two glyphs in the pan
                if (sensorB.getDistance(DistanceUnit.CM) < 13)
                {
                    if (sensorC.getDistance(DistanceUnit.CM) < 13) {
                        bothBlockCounter++;
                    }//Add 1 to bothBlockCounter, signifying we have had two glyphs in the pan for an iteration of loop
                }
                else //Else
                {
                    bothBlockCounter /= 2; //Divide bothBlockCounter by 2, signifying we have not had two glyphs in the pan for an iteration of loop.
                    //We divide by 2 so bothBlockCounter quickly decreases, so we know sooner when we do not have two glyphs
                    //and can more quickly adjust for this change
                }
            }




            feelerRaisePosition = FEELER_RAISE_UP_POSITION; //Set feelerRaisePosition to FEELER_RAISE_UP_POSITION, which will set the feelerRaise to a position so the jewel arm is pressed against the body of the robot
        }

        //ENDGAME

        //Else
        else {
            //If there is a jewel in the cryptobox, we allow Kevin, in endgame mode, to lower the touchServo bar, which will allow Kevin to hit the jewel with the bar
            //Thus, if leftBumper is not pressed, we keep the touchServo at a neutral position in the robot, but if it is pressed we give the touchServo
            //a position so the bar will lower
            if (!gamepad1.left_bumper) { //If leftBumper is not pressed
                touchServo.setPosition(.65); //Set touchServoPosition to .69, a position that will keep the bar high above the tiles
            }
            else { //Else, if leftBumper is being pressed
                touchServo.setPosition(.175);; ////Set touchServoPosition to .4, a position that will keep the bar close to the tiles so Kevin can use the bar to remove jewels from the cryptobox
            }

            if (gamepad1.dpad_up || rightStickButton) //If dPadUpPressed is true or rightStickButton is true (so Kevin can extend the relic while driving), we want to extend the relic.
            {
                relicMotorPower = .99; //Set relicMotorPower to .98, which will extend the relic
            }
            else if (gamepad1.dpad_down && !rightStickButton) //Else if dpadDownPressed is true, we want to retract the relic. Also make sure rightStickButton is false to avoid conflicting motor power commands.
            {
                relicMotorPower = -.99; //Set relicMotorPower to -.98, which will retract the relic
            }
            else //Else, if no commands above are extending or retracting the relic motor, keep the relic motor not moving
            {
                relicMotorPower = 0; //Set relicMotorPower to 0
            }

            if (gamepad1.right_trigger > .05 && gamepad1.left_trigger > .05) //If rightTrigger and leftTrigger are adequately pressed
            {
                //Do nothing due to conflicting commands, as rightTrigger increases grabPosition and leftTrigger decreases grabPosition
            }
            else if (gamepad1.left_trigger > .05 && !gamepad1.x && !gamepad1.right_bumper) //Else if leftTrigger is adequately pressed, and x and right bumper are not pressed to avoid conflicting grab servo commands, as x and right bumper also control grab
            {
                grabPosition += .0021 * gamepad1.left_trigger; //Add .035 (experimentally tested value to move grab to a speed of Kevin's liking) times the value of leftTrigger to grabPosition, which will open grab during each loop when the statement is triggered.
            }
            else if (gamepad1.right_trigger > .05 && !gamepad1.x && !gamepad1.right_bumper)  //Else if rightTrigger is adequately pressed, and x and right bumper are not pressed to avoid conflicting grab servo commands, as x and right bumper also control grab
            {
                grabPosition -= .0052 * gamepad1.right_trigger; //Subtract .055 (experimentally tested value to move grab to a speed of Kevin's liking) times the value of rightTrigger to grabPosition, which will close grab during each loop when the statement is triggered.
            }


            if (gamepad1.a) //Else if aPressed is true, Kevin wants to move the relic arm down
            {
                upDownPosition += .0015; //Add .05 to upDownPosition

            }
            else if (gamepad1.y) //Else if yPressed is true, then Kevin wants to move the relic up
            {
                upDownPosition -= .0023; //Subract .03 from upDownPosition
            } else if (gamepad1.x) { //Else if x is pressed, Kevin wants to move the relic arm down and release the relic, which is reflected in the upDownPosition and grab position.
                //The upDownPosition is .77, which puts the arm higher than rightBumper does, as Kevin will press x when the relic motor linear slides are extended,
                //Whch will lower the arm as more weight is extended.

                upDownPosition = .77; //Set upDownPosition to .77
                grabPosition = .44; //Set grabPosition to .44
            } else if (gamepad1.b) { //Else if b is pressed, Kevin wants to move the relic arm up to position .5. This position ensures the relic arm is not sotred, but is not all the way down, so Kevin can easily retrieve another relic if time allows
                if (upDownPosition > .5) { //If upDownPosition is greater than .5
                    upDownPosition -= .005; //Subtract .01 from upDownPosition
                }
            } else if (gamepad1.right_bumper) { //Else if right bumper is being pressed, //Else if right bumper is pressed, Kevin wants to move the relic arm down and release the relic, which is reflected in the upDownPosition and grab position.
                //The upDownPosition is .82, which puts the arm lower than x does, as Kevin will press right bumper when the relic motor linear slides are not extended,
                //Which will keep the arm high as less weight is extended.
                upDownPosition = .82; //Set upDownPosition to .82
                grabPosition = .44; //Set grabPosition to .44
            } else //Else, if no commands relating to upDown or grab are being triggered
            {
                //Do nothing
            }

            feelerRaisePosition = FEELER_RAISE_ENDGAME_POSITION; //Set feelerRaise to FEELER_RAISE_ENDGAME_POSITION, a position which will angle the jewel mechanism away from the robot so it cannot interfere with the relicMotor

        }

        //CLIP VALUES

        bothBlockCounter = Range.clip(bothBlockCounter, 0, 500); //Clip bothBlockCounter from 0 to 500. This is because if the value gets too high, it will take to long to come back down when two glyphs are no longer seen, and visa versa
        grabPosition = Range.clip(grabPosition, .07, .48); //Ensure grabPosition is between .07 and .48, so the grabber does not grab the relic excessively tightly, which could stall the grab servo, and so that grab does not open up too much, which will make closing it take longer
        //upDownPosition = Range.clip(upDownPosition, .02, .83); //Ensure upDownPosition is between .02 and .83, so upDown does not run the arm into the robot when upDown is being stored, and so upDown does not go under the relic retrieving position
        panSpinPosition = Range.clip(panSpinPosition, .525, .95); //Ensure panSpinPosition is between .21 and .825. The lower limit is the collecting glyphs position, and the top limit is the scoring glyphs positin

        //Clip final driving motor values between -1 and 1, as DC motors only accept values in this range
        finBackPower = Range.clip(finBackPower, -1, 1); //Ensure finBackPower is between -1 and 1
        finRightPower = Range.clip(finRightPower, -1, 1); //Ensure finRightPower is between -1 and 1
        finLeftPower = Range.clip(finLeftPower, -1, 1); //Ensure finLeftPower is between -1 and 1

        // panSpinPosition = Range.clip(panSpinPosition, .9, .1);

        //SET VALUES

        leftPanSpin.setPosition(panSpinPosition); //Set the position of leftPanSpin to panSpinPosition
        rightPanSpin.setPosition(panSpinPosition + .036);
        //panLifterMotor.setPower(panLiftingPower); //Set the power of panLifterMotor to panLiftingPower
        // leftIntakeMotor.setPower(leftIntakePower); //Set the power of leftIntakeMotor to leftIntakePower
        //    rightIntakeMotor.setPower(rightIntakePower); //Set the power of rightIntakeMotor to rightIntakePower
        leftMotor.setPower(finLeftPower); //Set the power of leftMotor to finLeftPower
        rightMotor.setPower(finRightPower); //Set the power of rightMotor to finRightPower
        backMotor.setPower(finBackPower); //Set the power of backMotor to finBackPower
        grab.setPosition(grabPosition); //Set the power of grab to grabPosition
        //   upDown.setPosition(upDownPosition); //Set the power of upDown to upDownPosition
        // touchServo.setPosition(touchServoPosition); //Set the position of touchServo to touchServoPosition
        feelerRaise.setPosition(feelerRaisePosition); //Set the position of feelerRaise to feelerRaisePosition
        relicMotor.setPower(relicMotorPower); //Set the power of relicMotor to relicMotorPower
        upDown.setPosition(upDownPosition);
        timeTwo=this.getRuntime();

        telemetry.addData("loopCounter", loopCounter);
        telemetry.addData("lps", loopCounter/(timeTwo-timeOne));
        telemetry.update();

    } // end loop




}
