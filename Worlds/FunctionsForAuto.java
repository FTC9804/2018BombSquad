//FUNCTIONS FOR AUTONOMOUS

//Package statement
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

//Class declaration
public abstract class FunctionsForAuto extends LinearOpMode {

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

    //Ball Scoring
    Servo feelerRaise; //Servo that lifts and lowers the ball scoring mechanism, known as the "feeler"
    Servo feelerSwipe; //Servo that swipes the ball once feelerRaise is lowered to score the ball

    //Block Rotation
    Servo leftPanSpin; //Servo on the left side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo rightPanSpin;
    Servo frontPanGrip; //Servo that moves the grip on the left side of the pan
    Servo backPanGrip; //Servo that moves the grip on the right side of the pan

    //Touch driving
    Servo touchServo; //Servo that extends an arm from the front of the robot to detect when we are ready to score in autonomous
    //The arm is attached to a metal bar to block glyphs from entering the robot incorrectly in teleop

    //COLOR SENSOR
    ColorSensor sensorColorFeeler; //Color sensor for detecting ball color in autonomous

    //DISTANCESENSORS

    //Distance sensors to detect glyphs. If sensor B and sensor C see blocks, that means we are ready to score, so we lift up the pan.
    //If all three see glyphs for too long, that means we have three glyphs, so we outtake as this is a penalty.

    DistanceSensor sensorB; //Distance sensor between A and C, to see how far potential blocks are
    DistanceSensor sensorC; //Distance sensor farthest from intake, to see how far potential blocks are

    DistanceSensor sensorTouch; //Distance sensor on the front of the robot to see how far we are from the cryptobox

    //Time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne; //timeOne, first time variable
    double timeTwo; //timeTwo, second time variable
    double timeThree; //timeThree, third time variable
    double timeFour; //timeFour, fourth time variable
    double threeGlyphTimeOne; //fifth time variable, calculates run time at the beginning of auto to see if we later have time to collect two more glyphs

    //Driving variables
    double inches, inches2; //Desired number of inches to drive
    double rotations, rotations2;  //Wheel rotations necessary to drive the above amount of inches
    double counts, counts2; //Encoder counts necessary to drive the above amount of inches/rotations
    final double ENCODER_CPR = 537.6; //Encoder counts per rotation (CPR, AndyMark NeveRest 20 Motor)
    final double GEAR_RATIO = .727; //Gear ratio of driving motors, 16/22
    final double WHEEL_DIAMETER = 4; //Wheel diameter in inches
    double spinPower; //The absolute value of the power we apply to the left and right motor when executing a spin

    //Loop counter
    int loopCounter = 0; //Integer that is incremented positively by 1 during every iteration of a given loop

    //Alliance variables
    String allianceColor; //The alliance color of a given match
    String robotStartingPosition; //The starting position of the robot, either relicSide or triangle Side, representing different places on the field

    //Ball scoring variables
    final double FEELER_SWIPE_NEUTRAL_POSITION_BLUE = .43; //Straight position of feelerSwipe for blue
    final double FEELER_SWIPE_NEUTRAL_POSITION_RED = .41; //Straight position of feelerSwipe for red
    final double FEELER_SWIPE_CW_POSITION = .15; //Clockwise turned position of feelerSwipe
    final double FEELER_SWIPE_CCW_POSITION = .99; //Counter-clockwise turned position of feelerSwipe
    final double FEELER_RAISE_UP_POSITION = .93; //Position that the feelerRaise is set to when we are not scoring the ball
    double FEELER_RAISE_DOWN_POSITION = .48; //Position that the feelerRaise is set to when we are scoring the ball

    //Vuforia
    VuforiaLocalizer vuforia; //Variable to store our instance of the Vuforia localization engine
    RelicRecoveryVuMark vuMark; //Variable to store the reading from Vuforia, either left, center, or right
    String vuMarkChecker = new String(); //Variable to store the reading from Vuforia in String form
    VuforiaTrackable relicTemplate; //Variable to load the VuMarks in Relic Recovery
    VuforiaTrackables relicTrackables; //Variable to load the VuMarks in Relic Recovery
    String vuMarkReturn; //The string that our vuforia method returns to tell us where to score, either left, center, or right


    //Limit Switches
    DigitalChannel limitTop; //Limit Switch that tells us if we reach the top of the robot with the Pan
    DigitalChannel limitBottom; //Limit switch that tells us if we reach the bottom of the robot with the Pan
    DigitalChannel limitMid; //Limit Switch that tells us if we reach the middle of the elevator
    DigitalChannel touchEnd; //Touch sensor on the arm in front of the robot, which tells us when we touch the back of the cryptobox to know when to score a block in autonomous

    //IMU
    BNO055IMU imu; //IMU sensor for detecting the angle of the robot

    //IMU variables
    Orientation lastAngles = new Orientation(); //Angle state used for updating telemetry
    double initialHeading; //The angle of the robot at the beginning of a given method
    double globalAngle, currentAngle; //The angle of the robot at any given moment
    double correction; //A value to add to or subtract from our driving power based on our angle error

    //Pan spin variables
    double bothBlockCounter = 0; //Double representing the number of iterations of a loop in which sensors B and C both see a block

    //OVERALL VARIABLES
    double timeFive;
    double timeSix;
    int stepInMenu = 1;
    boolean notAllChosen = true;
    boolean choiceNotSelected = true;
    boolean goBack = false;

    //setTimeDelay() variables
    int turnPosition;
    String attackConfirmation;
    boolean glyphPositionNotSelected = true;

    double frontBlockCounter;

    double backBlockCounter;


    //Configures all hardware devices, and sets them to their initial values, if necessary
    public void configure( String initialAllianceColor, String initialRobotStartingPosition ) {

        setGlyphAttackCorner();

        telemetry.clearAll();
        telemetry.addData("Glyph Attack Position = ", attackConfirmation);
        telemetry.addLine("Executed");
        telemetry.update();


        attackConfirmation = "center";

        //Color and position
        allianceColor = initialAllianceColor; //Options: "red" or "blue"
        robotStartingPosition = initialRobotStartingPosition; //Options: "relicSide" or "triangleSide"

        //Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //Configure phone camera to sense Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //Initialize the storage of our Vuforia results
        parameters.vuforiaLicenseKey =  "AZfTpOj/////AAAAGYCE1z7z6E5whPRKfYeRJHEN/u/+LZ7AMmBU0bBa" +
                "/7u6aTruUWfYeLur6nSFdKP0w9JPmK1gstNxVHqiaZN6iuZGxPcbnDnm" +
                "NJdoLIMtZheeNWphUMjHKoTUgsmcloZe67TG2V9duc+8jxxCLFzH5rlq" +
                "PPdcgvvtIO0orpxVcpENBunY2GChhVgP6V5T9Iby7MyM9tN+y7Egm7Xy" +
                "Iz/Tzpmlj19b3FUCW4WUDjTNQ4JoKZeB1jkhPxKGFRECoPw02jJXtQSK" +
                "zNfzmhtugA7PTOZNehc61UjOXEexTO9TRy7ZfMtW8OggcYssvIabyJ8b" +
                "DK4ePLCUP+Q4PMf7kL9lM6yDuxxKF0oqLgRglX9Axqrf"; //Our custom Vuforia license key, implemented to authorize our use of Vuforia
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //Specify that we are using the back camera of the phone to run Vuforia
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //Configure the use of the back camera of our phone
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //Load the data sets that for the trackable objects we wish to track
        relicTemplate = relicTrackables.get(0); //Specify we are reading relicTrackables, using the three possible outcomes, left, right, and center

        //Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1"); //m1
        leftMotor = hardwareMap.dcMotor.get("m2"); //m2
        backMotor = hardwareMap.dcMotor.get("m3"); //m3
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //m4
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //m5
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //m6
        relicMotor = hardwareMap.dcMotor.get("m7"); //m7

        //Servo configurations in the hardware map
        leftPanSpin = hardwareMap.servo.get("s1"); //s1
        rightPanSpin = hardwareMap.servo.get("s2");
        feelerRaise = hardwareMap.servo.get("s8"); //s8
        feelerSwipe = hardwareMap.servo.get("s9"); //s9
        touchServo = hardwareMap.servo.get("s10"); //s10
        frontPanGrip = hardwareMap.servo.get("s12"); //s12
        backPanGrip = hardwareMap.servo.get("s13"); //s13

        //Set up the parameters with which we will use the IMU
        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
        IMUparameters.mode                = BNO055IMU.SensorMode.IMU;
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled      = true;
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //Sensor configurations in the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "i0"); //i0
        imu.initialize(IMUparameters); //Initialize the IMU
        limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //d1
        limitBottom = hardwareMap.get(DigitalChannel.class, "d2"); //d2
        touchEnd = hardwareMap.get(DigitalChannel.class, "d5"); //d3
        limitMid=hardwareMap.get(DigitalChannel.class, "d3");
        sensorColorFeeler = hardwareMap.get(ColorSensor.class, "i1"); //i1
        sensorColorFeeler.enableLed(true); //Enable the LED of the color sensor

        sensorB = hardwareMap.get(DistanceSensor.class, "i3"); //i3
        sensorC = hardwareMap.get(DistanceSensor.class, "i4"); //i4
        sensorTouch = hardwareMap.get(DistanceSensor.class, "i5"); //i5


        //Motor directions
        rightMotor.setDirection(REVERSE); //Set rightMotor to REVERSE direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to FORWARD direction
        backMotor.setDirection(REVERSE); //Set backMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to REVERSE direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(FORWARD); //Set panLifterMotor to FORWARD direction

        //Servo directions
        leftPanSpin.setDirection(Servo.Direction.FORWARD); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.REVERSE); //Set rightPanSpin to REVERSE direction
        feelerRaise.setDirection(Servo.Direction.FORWARD); //Set feelerRaise to FORWARD direction
        feelerSwipe.setDirection(Servo.Direction.REVERSE); //Set feelerSwipe to REVERSE direction
        touchServo.setDirection(Servo.Direction.REVERSE); //Set touchServo to REVERSE direction
        frontPanGrip.setDirection(Servo.Direction.FORWARD); //Set servo leftPanGrip to FORWARD direction
        backPanGrip.setDirection(Servo.Direction.FORWARD); //Set rightPanGrip to FORWARD direction

        //Init values
        feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to FEELER_SWIPE_CCW_POSITION, so the jewel arm stays within 18 inches
        feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Set feelerRaise to FEELER_RAISE_UP_POSITION, so the jewel arm is pressed against the robot
        leftPanSpin.setPosition(.46); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.33);
        frontPanGrip.setPosition(.964);
        backPanGrip.setPosition(.812);
        touchServo.setPosition(.68); //Set the position of touchServo to .68, so the bar stays within 18 inches
    }

    //Method to calibrate the IMU gyro
    public void calibrateGyro() {
        while (!isStopRequested() && !imu.isGyroCalibrated()) //While a stop to the op mode has not been requested and the imu is not calibrated
        {
            sleep(50); //Sleep for 50 milliseconds
            idle(); //Mark process as swappable and lowers its priority
        }
    }

    //Method to introduce the angle of the robot at the beginning of a method
    public void introduceAngle()
    {
        realAngle(); //Call realAngle(), which returns the current angle of the robot
    }

    //Method to score either one or two blocks during autonomous
    public void scoreBlock (double angle) {

        touchServo.setPosition(.35); //Set the positon of touchServo to .35 which will raise the bar down to not interfere with scoring

        pause(.15); //Pause for .15 seconds to allow the touchServo to get to position

        while (limitMid.getState()) //Lift panLifterMotor until limitMid sees the elevator
        {
            panLifterMotor.setPower(.75); //Set panLifterMotor to .75 power
            telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed
            telemetry.update();
        }

        panLifterMotor.setPower(0); //Stop the movement of panLifterMotor
        driveNewIMU(5, .8, .3, true, angle); //Drive forward 3.2 inches to allow room for block scoring
        leftPanSpin.setPosition(.835); //Set leftPanSpin to position .835 to score
        rightPanSpin.setPosition(.871);
        pause(.82); //pause for .82 seconds to allow leftPanSpin to get to position

        //Driving motions to wedge glyph(s) in cryptobox
        driveNewIMU(3.7, .8, -.3, true, angle); //Drive forward for 2.7 inches at .3 power, with a 5 second time limit, maintaining the parameter angle degree heading
        driveNewIMU(4.8, .8, .3, true, angle); //Drive forward for 4.8 inches at .3 power, with a 5 second time limit, maintaining the parameter angle degree heading

    }


    //Method to drive forward until our touch sensor, attatched to the front bar, hits the cryptobox, to help us allign to score glyphs
    public void driveToTouch (double power, double desiredAngle)
    {
        touchServo.setPosition(.24); //Set touchServo to position .27, so the touch sensor has an optimal chance of hitting the back of the cryptobo

        //Usually we give the touchServo time to lower to .27 during our first spin in auto, but on red relic side there is no spin, so we give said time here
        if (allianceColor.equalsIgnoreCase("red") && robotStartingPosition.equalsIgnoreCase("relicSide"))
        {
            pause(.5);
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (touchEnd.getState() && timeTwo-timeOne < 2.2)
        {
            //Set currentAngle to this.getAngleSimple()
            currentAngle = this.realAngle();

            //Set correction to the current angle minus the desired angle times .016
            correction = (this.realAngle()-desiredAngle) * .016;

            //Set the power of leftMotor to power plus correction, and the power of rightMotor to power minus correction
            leftMotor.setPower(power + correction);
            rightMotor.setPower(power - correction);

            //Set timeTwo to this.getRuntime()
            timeTwo=this.getRuntime();
        }

        //Stop the movement of leftMotor and rightMotor
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //Method to drive until the touch sensor is pressed, then strafe until the front distance sensor sees the cryptobox, and then score a glyph
    public void touch (boolean blue, boolean blueCorner, boolean redCorner)
    {
        if (blue) { //If blue is true, meaning we are the blue alliance
            if (blueCorner) { //If blueCorner is true, meaning we are scoring from the balancing stone in the corner position
                driveToTouch(.3, -90); //Run method driveToTouch at .3 power and -90 degrees to touch the back of the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(1.2, 3, -.3, false, -90); //Drive backwards for 1.2 inches at -.3 power with a 3 second limit maintaining a -90 degree heading to give the arm breathing room so strafe

                pause(.025); //Pause for .025 seconds

                strafeToTouch(3.5, .65, -90); //Run strafeToTouch method at -90 degrees and .65 power, with a 3.5 second timeout, to align with the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(2.8, 3, -.3, false, -90); //Drive backwards for 2.8 inches at -.3 power with a 3 second timeout at -90 degrees, to give the bar room to raise

                pause(.025); //Pause for .025 seconds

                touchServo.setPosition(.66); //Set the position of touchServo to .66 to give the pre-loaded glyph space to exit the robot

                pause(.05); //Pause for .05 Seconds to give the touchServo time to get to position .66

                driveNewIMU(6.4, 4, .4, true, -90); //Drive forward for 6.4 inches at a -90 degree heading with a 4 second time limit to prepare glyph for entry into the cryptobox

                pause(.05); //Pause for .05 seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power for outtaking, set to different powers so block comes out diagonally and we have a higher chance of scoring
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.25); //Pause for .25 seconds to give intake motors time to speed up

                driveNewIMU(2.65, 5, -.4, false, -90); //Drive backwards 2.65 inches at -.4 power keeping a -90 degree heading with a 5 second limit to allow room for the glyph to exit the robot

                //Outtake for 1 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 1) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors, as we are now (hopefully) done scoring the glyphs
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.025); //pause for .025 seconds

                driveNewIMU(3.5, 2, -.4, false, -90); //Drive backwards 3.5 inches at -.4 power keeping a -90 degree heading with a 2 second limit, to make sure we are not touching the glyph that we scored
            }

            else {
                driveToTouch(.3, 180); //Run method driveToTouch at .3 power and 180 degrees to touch the back of the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(1.2, 3, -.3, false, 180); //Drive backwards for 1.2 inches at -.3 power with a 3 second limit maintaining a 180 degree heading to give the arm breathing room so strafe

                pause(.025); //Pause for .025 seconds

                strafeToTouch(3.5, .65, 180); //Run strafeToTouch method at 180 degrees and .65 power, with a 3.5 second timeout, to align with the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(2.8, 3, -.3, false, 180); //Drive backwards for 2.8 inches at -.3 power with a 3 second timeout at 180 degrees, to give the bar room to raise

                pause(.025); //Pause for .025 seconds

                touchServo.setPosition(.66); //Set the position of touchServo to .66 to give the pre-loaded glyph space to exit the robot

                pause(.05); //Pause for .05 Seconds

                driveNewIMU(6.4, 4, .4, true, 180); //Drive forward for 6.4 inches at a 180 degree heading with a 4 second time limit to prepare glyph for entry into the cryptobox

                pause(.05); //Pause for .2 Seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power for outtaking, set to different powers so block comes out diagonally and we have a higher chance of scoring
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.25); //Pause for .25 seconds to give intake motors time to speed up

                driveNewIMU(2.65, 5, -.4, false, 180); //Drive backwards 2.65 inches at -.4 power keeping a 180 degree heading with a 5 second limit to allow room for the glyph to exit the robot

                //Outtake for 1 second
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 1) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors, as we are now (hopefully) done scoring the glyphs
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.025); //pause for .025 seconds

                driveNewIMU(3.5, 2, -.4, false, 180); //Drive backwards 3.5 inches at -.4 power keeping a 180 degree heading with a 2 second limit, to make sure we are not touching the glyph that we scored
            }
        }

        else {
            if (redCorner) {
                driveToTouch(.3, -90); //Run method driveToTouch at .3 power and -90 degrees to touch the back of the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(1.2, 3, -.3, false, -90); //Drive backwards for 1.2 inches at -.3 power with a 3 second limit maintaining a -90 degree heading to give the arm breathing room so strafe

                pause(.025); //Pause for .025 seconds

                strafeToTouch(3.5, .65, -90); //Run strafeToTouch method at -90 degrees and .65 power, with a 3.5 second timeout, to align with the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(2.8, 3, -.3, false, -90); //Drive backwards for 2.8 inches at -.3 power with a 3 second timeout at -90 degrees, to give the bar room to raise

                pause(.025); //Pause for .025 seconds

                touchServo.setPosition(.66); //Set the position of touchServo to .66 to give the pre-loaded glyph space to exit the robot

                pause(.05); //Pause for .05 Seconds to give the touchServo time to get to position .66

                driveNewIMU(6.4, 4, .4, true, -90); //Drive forward for 6.4 inches at a -90 degree heading with a 4 second time limit to prepare glyph for entry into the cryptobox

                pause(.05); //Pause for .05 seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power for outtaking, set to different powers so block comes out diagonally and we have a higher chance of scoring
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.25); //Pause for .25 seconds to give intake motors time to speed up

                driveNewIMU(2.65, 5, -.4, false, -90); //Drive backwards 2.65 inches at -.4 power keeping a -90 degree heading with a 5 second limit to allow room for the glyph to exit the robot

                //Outtake for 1 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 1) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors, as we are now (hopefully) done scoring the glyphs
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.025); //pause for .025 seconds

                driveNewIMU(3.5, 2, -.4, false, -90); //Drive backwards 3.5 inches at -.4 power keeping a -90 degree heading with a 2 second limit, to make sure we are not touching the glyph that we scored
            }
            else
            {
                driveToTouch(.3, 0); //Run method driveToTouch at .3 power and 0 degrees to touch the back of the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(1.2, 3, -.3, false, 0); //Drive backwards for 1.2 inches at -.3 power with a 3 second limit maintaining a 0 degree heading to give the arm breathing room so strafe

                pause(.025); //Pause for .025 seconds

                strafeToTouch(3.5, .65, 0); //Run strafeToTouch method at 0 degrees and .65 power, with a 3.5 second timeout, to align with the cryptobox

                pause(.025); //Pause for .025 seconds

                driveNewIMU(2.8, 3, -.3, false, 0); //Drive backwards for 2.8 inches at -.3 power with a 3 second timeout at 0 degrees, to give the bar room to raise

                pause(.025); //Pause for .025 seconds

                touchServo.setPosition(.66); //Set the position of touchServo to .66 to give the pre-loaded glyph space to exit the robot

                pause(.05); //Pause for .05 Seconds to give the touchServo time to get to position .66

                driveNewIMU(8.6, 4, .4, true, 0); //Drive forward for 8.6 inches at a 0 degree heading with a 4 second time limit to prepare glyph for entry into the cryptobox

                pause(.05); //Pause for .05 seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power for outtaking, set to different powers so block comes out diagonally and we have a higher chance of scoring
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.25); //Pause for .2 seconds

                driveNewIMU(2.65, 5, -.4, false, 0); //Drive backwards 2.25 inches at -.4 power keeping a 0 degree heading with a 5 second limit

                pause(.25); //Pause for .25 seconds to give intake motors time to speed up

                //Outtake for 1 second
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 1) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors, as we are now (hopefully) done scoring the glyphs
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.025); //pause for .025 seconds

                driveNewIMU(3.5, 2, -.4, false, 0); //Drive backwards 3.5 inches at -.4 power keeping a 0 degree heading with a 2 second limit, to make sure we are not touching the glyph that we scored
            }
        }

    }



    //Method to collect blocks during autonomous
    public void getBlockOne() {

        loopCounter = 1000; //Set loopCounter to 1000

        inches = 30; //Set inches2 to 60
        rotations = inches / (Math.PI * WHEEL_DIAMETER); //Set rotations2 to inches2 divided by pi, approximately 3.14, divided by the wheel diameter
        counts = ENCODER_CPR * rotations * GEAR_RATIO; //Set counts2 to the encoder CPR times rotations2 times the gear ratio

        leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.17);

        touchServo.setPosition(.45); //Set touchServo to .42 to block bad glyphs from coming into the robot

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of rightMotor to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); ////Set run mode of rightMotor to RUN_USING_ENCODER

        //Set timeOne and timeTwo to this.getRuntime()
        timeThree= this.getRuntime();
        timeFour=this.getRuntime();

        frontBlockCounter=0;

        //While bothBlockCounter is under 5 and the current position of rightMotor is less than counts2 and less than 4.5 seconds have elapsed
        while (frontBlockCounter < 5 && rightMotor.getCurrentPosition()<(counts) && timeFour-timeThree < 2.3)
        {
            leftIntakeMotor.setPower(.58); //Set leftIntakeMotor to .73 power to intake blocks
            rightIntakeMotor.setPower(.58); //Set rightIntakeMotor to 73 power to intake blocks

            leftMotor.setPower(.52);
            rightMotor.setPower(.52);

            if (sensorB.getDistance(DistanceUnit.CM) < 13) {
                frontBlockCounter++;
            }

            timeFour=this.getRuntime(); //Set timeFOUR to this.getRuntime()

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        pause(.2);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < .08)
        {
            timeTwo = this.getRuntime();
            leftIntakeMotor.setPower(-.7);
            rightIntakeMotor.setPower(-.8);
        }

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        pause(.5);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < .4)
        {
            timeTwo = this.getRuntime();
            leftIntakeMotor.setPower(.58);
            rightIntakeMotor.setPower(.58);
        }



        //Telemetry
        //telemetry.addData("left", leftMotor.getPower());
        //telemetry.addData("loopCounter", loopCounter);
        telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed

        telemetry.addData("blocks", bothBlockCounter);
        telemetry.addData("time", timeFour-timeThree);
        telemetry.update();

    }

    //Method to collect blocks during autonomous
    public void getBlockTwo() {

        loopCounter = 1000; //Set loopCounter to 1000

        inches = 15; //Set inches2 to 60
        rotations = inches / (Math.PI * WHEEL_DIAMETER); //Set rotations2 to inches2 divided by pi, approximately 3.14, divided by the wheel diameter
        counts = ENCODER_CPR * rotations * GEAR_RATIO; //Set counts2 to the encoder CPR times rotations2 times the gear ratio

        leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.17);

        touchServo.setPosition(.45); //Set touchServo to .42 to block bad glyphs from coming into the robot

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of rightMotor to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); ////Set run mode of rightMotor to RUN_USING_ENCODER

        //Set timeOne and timeTwo to this.getRuntime()
        timeThree= this.getRuntime();
        timeFour=this.getRuntime();

        backBlockCounter=0;

        //While bothBlockCounter is under 5 and the current position of rightMotor is less than counts2 and less than 4.5 seconds have elapsed
        while (backBlockCounter < 5 && rightMotor.getCurrentPosition()<(counts) && timeFour-timeThree < 2)
        {
            leftIntakeMotor.setPower(.58); //Set leftIntakeMotor to .73 power to intake blocks
            rightIntakeMotor.setPower(.58); //Set rightIntakeMotor to 73 power to intake blocks

            leftMotor.setPower(.42);
            rightMotor.setPower(.42);

            if (sensorC.getDistance(DistanceUnit.CM) < 13) {
                backBlockCounter++;
            }

            timeFour=this.getRuntime(); //Set timeFOUR to this.getRuntime()

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        pause(.2);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < .08)
        {
            timeTwo = this.getRuntime();
            leftIntakeMotor.setPower(-.7);
            rightIntakeMotor.setPower(-.8);
        }

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        pause(.5);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < .4)
        {
            timeTwo = this.getRuntime();
            leftIntakeMotor.setPower(.58);
            rightIntakeMotor.setPower(.58);
        }



        //Telemetry
        //telemetry.addData("left", leftMotor.getPower());
        //telemetry.addData("loopCounter", loopCounter);
        telemetry.addData("time elapsed", this.getRuntime()-threeGlyphTimeOne); //Telemetry for time elapsed

        telemetry.addData("blocks", bothBlockCounter);
        telemetry.addData("time", timeFour-timeThree);
        telemetry.update();

    }


    //Method to detect the Vuforia reading
    public String detectVuMark(double timeToCheck) {

        relicTrackables.activate(); //Activate relicTrackables to prepare vuforia reading

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < timeToCheck) { //While less time than timeToCheck has elapsed

            vuMark = RelicRecoveryVuMark.from(relicTemplate); //Read the vuMark from relicTemplate

            if (vumarkToString().equalsIgnoreCase("LEFT")) { //If the vuMark is left, set vuMarkChecker to "left"
                vuMarkChecker = "left";
            } else if (vumarkToString().equalsIgnoreCase("RIGHT")) { //If the vuMark is right, set vuMarkChecker to "right"
                vuMarkChecker = "right";
            } else if (vumarkToString().equalsIgnoreCase("CENTER")) { //If the vuMark is center, set vuMarkChecker to "center"
                vuMarkChecker = "center";
            } else if (vumarkToString().equals("UNKOWN")){ //If the vuMark is unknown, set vuMarkChecker to "unknown"
                vuMarkChecker = "unknown as answer";
            }
            else {
                vuMarkChecker = "novalue"; //Else, set vuMarkChecker to "no value"
            }

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //If vuMark is not unknown, add telemetry to indicate that vuMark is visible
                telemetry.addData("VuMark", "%s visible", vuMark);

            } else { //If vuMark is unknown, add telemetry to indicate that vuMark is not visible
                telemetry.addData("VuMark", "not visible");
            }

            timeTwo = this.getRuntime(); //Set timeTwo to this.getRuntime()

            //Telemetry to display time and the vuMark result
            telemetry.addData("Time: ", timeTwo - timeOne);
            telemetry.addData("Vu mark detector: ", vuMarkChecker);
            telemetry.update(); //update telemetry

            if (timeTwo - timeOne > .6 && timeTwo-timeOne <= 1.4) //We want to start preparing to score the jewel during vuforia detection to save time
            //so during this time interval we set feeler swipe to a neutral position so when feeler raise
            //is lowered feeler swipe puts the jewel arm between the two balls for optimal color detection
            {
                if (allianceColor.equalsIgnoreCase("blue")) {
                    feelerSwipe.setPosition(FEELER_SWIPE_NEUTRAL_POSITION_BLUE); //Set feelerSwipe to FEELER_SWIPE_NEUTRAL_POSITION_BLUE
                }
                else
                {
                    feelerSwipe.setPosition(FEELER_SWIPE_NEUTRAL_POSITION_RED); //Set feelerSwipe to FEELER_SWIPE_NEUTRAL_POSITION_RED
                }
            }

            if (timeTwo - timeOne > 1.4  && timeTwo-timeOne <= 2) //After the time interval above we lower the feelerRaise to get into jewel scoring position
            {
                feelerRaise.setPosition(FEELER_RAISE_DOWN_POSITION); //Set feelerRaise to position FEELER_RAISE_DOWN_POSITION
            }
        }
        return vuMarkChecker; //Return the String vuMarkChecker
    }


    //Method to return the vuMark as a string
    public String vumarkToString() {
        String output; //Declare local String variable output
        output = "" + vuMark; //Set output to vuMark in String form
        return output; //return output
    }



    //Method to stop all action for a given amount of time
    public void pause(double time) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Pause for variable time seconds
        while (timeTwo - timeOne < time) {
            timeTwo = this.getRuntime();
        }

    }

    //Method to score the ball in autonomous
    public void dropFeelerMoveBallOnlyNewRobot() {

        threeGlyphTimeOne = this.getRuntime(); //Set threeGlyphTimeOne to the current run time, this variable (representime the run time at the beginning of the first action in auto, scoring the jewel) is later used to determine if we have time to collect two more glyphs

        feelerRaise.setPosition(.65); //Set feelerRaise to position .49 to lower the jewel arm

        vuMarkReturn = detectVuMark(2); //Run vuforia detection during the time when the feelerRaise is moving down, and set the output of detectVuMark to the String vuMarkReturn
        pause(.5); //.1 second pause
        if (allianceColor.equalsIgnoreCase("red") && sensorColorFeeler.blue() >=  sensorColorFeeler.red()) { //If we are the red alliance and see a blue ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red());
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue());
            telemetry.update(); //Update telemetry
            feelerSwipe.setPosition(FEELER_SWIPE_CW_POSITION); //Set feelerSwipe to its clockwise position
            pause(.5); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.5); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("red") && sensorColorFeeler.red() >= sensorColorFeeler.blue()) { //If we are the red alliance and see a red ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.5); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to its counter-clockwise position
            pause(.5); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.5); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColorFeeler.blue() >= sensorColorFeeler.red()) { //If we are the blue alliance and see a blue ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.5); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to its counter-clockwise position
            pause(.5); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.5); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColorFeeler.red() >= sensorColorFeeler.blue()) { //If we are the blue alliance and see a red ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.5); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CW_POSITION); //Set feelerSwipe to its clockwise position
            pause(.5); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.5); //.1 second pause
        }

    }

    //Method to make the robot drive sideways (strafe)
    public void strafeNewIMU(double distance, double time, double power, double desiredAngle) {

        inches = distance; //Set inches to parameter distance
        rotations = inches / (Math.PI * WHEEL_DIAMETER); //Set rotations to inches divided by pi, approximately 3.14, divided by the wheel diameter
        counts = ENCODER_CPR * rotations * GEAR_RATIO; //Set counts to the encoder CPR times rotations times the gear ratio

        backMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set mode of backMotor to STOP_AND_RESET_ENCODER
        backMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set mode of back Motor to RUN_WITHOUT_ENCODER

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //If power is greater than 0
        if (power > 0) {

            //While less seconds than the parameter time have elapsed and the absolute value of the position of backMotor is less than the absolute value of counts
            while (Math.abs(backMotor.getCurrentPosition()) < Math.abs(counts) && timeTwo - timeOne < time) {
                //Set correction to the current angle minus the desired angle plus 9 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) + 4) * .034;

                //Set the power of backMotor to parameter power
                backMotor.setPower(power);

                //If the absolute value of correction is greater than .08
                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction); //Set the power of leftMotor to correction
                    rightMotor.setPower(-correction); //Set the power of rightMotor to negative correction
                } else { //Else
                    //Stop the movement of leftMotor and rightMotor
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();
            }

            //Stop the movement of backMotor, leftMotor, and rightMotor
            backMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        else { //Else

            //While less seconds than the parameter time have elapsed and the absolute value of the position of backMotor is less than the absolute value of counts
            while (Math.abs(backMotor.getCurrentPosition()) < Math.abs(counts) && timeTwo - timeOne < time) {

                //Set correction to the current angle minus the desired angle minus 2 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) - 2)  * .034;

                //Telemetry
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Angle", this.realAngle());
                telemetry.addData("inches", inches);
                telemetry.addData("power", power);
                telemetry.addData("pos", backMotor.getCurrentPosition());
                telemetry.addData("counts", counts);
                telemetry.update();

                //Set the power of backMotor to parameter power
                backMotor.setPower(power);

                //If the absolute value of correction is greater than .08
                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction); //Set the power of leftMotor to correction
                    rightMotor.setPower(-correction); //Set the power of rightMotor to negative correction
                } else {
                    //Stop the movement of leftMotor and rightMotor
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();

            }

            //Stop the movement of backMotor, leftMotor, and rightMotor
            backMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    public void driveBack (double time)
    {
        timeOne=this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne<time)
        {
            timeTwo=this.getRuntime();
            leftMotor.setPower(-.5);
            rightMotor.setPower(-.5);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    //Method to make the robot drive forwards or backwards
    public void driveNewIMU(double distance, double time, double power, boolean forwards, double desiredAngle) {

        inches = distance; //Set inches to parameter distance
        rotations = inches / (Math.PI * WHEEL_DIAMETER); //Set rotations to inches divided by pi, approximately 3.14, divided by the wheel diameter
        counts = ENCODER_CPR * rotations * GEAR_RATIO; //Set counts to the encoder CPR times rotations times the gear ratio

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of rightMotor to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of rightMotor to RUN_WITHOUT_ENCODER

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //While less seconds than the parameter time have elapsed and the absolute value of the position of rightMotor is less than the absolute value of counts
        while (Math.abs(rightMotor.getCurrentPosition())<counts && timeTwo-timeOne<time) {

            //Set currentAngle to this.getAngleSimple()
            currentAngle = this.realAngle();

            //Set correction to the current angle minus the desired angle times .016 gain to adjust heading
            correction = (this.realAngle()-desiredAngle) * .02;
            telemetry.update();

            //Positively increment loopCounter by 1
            loopCounter++;

            //Set the power of leftMotor to power plus correction, and the power of rightMotor to power minus correction
            leftMotor.setPower(power + correction);
            rightMotor.setPower(power - correction);

            //Set timeTwo to this.getRuntime()
            timeTwo=this.getRuntime();

            //Lower the absolute value of the power of leftMotor and rightMotor as the drive progresses in order to brake
            if (forwards) { //If robot is moving forward
                //If the absolute value of rightMotor's position is greater than the absolute value of counts - 900, and loopCounter is under 100
                //This serves to not immediately start decreasing the speed of the motors and only brake when necessary
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 900)) {
                    power -= loopCounter * .0225; //Subtract loopCounter times .0225 from power
                    power = Range.clip(power, .23, 1); //Ensure power is between .18 and 1
                }

                if (distance==9 || distance == 5 || distance == 5.1)
                {
                    leftPanSpin.setPosition(.67);
                    rightPanSpin.setPosition(.64);
                }
                if (distance==6.1)
                {
                    leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
                    rightPanSpin.setPosition(.17);
                }
            }
            //Lower the absolute value of the power of leftMotor and rightMotor as the drive progresses in order to brake
            else //If robot is not moving forward
            {
                //If the absolute value of rightMotor's position is greater than the absolute value of counts - 700, and loopCounter is under 100
                //This serves to not immediately start decreasing the speed of the motors and only brake when necessary
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 700) && loopCounter < 100) {
                    power += loopCounter * .01;  //Add loopCounter times .01 from power
                    power = Range.clip(power, -1, -.23); //Ensure power is between -1 and -.27
                }

                if (distance==9 || distance == 5 || distance == 5.1)
                {
                    leftPanSpin.setPosition(.67);
                    rightPanSpin.setPosition(.64);
                }
                if (distance==6.1)
                {
                    leftPanSpin.setPosition(.2); //Set leftPanSpin to position .21, as this is the intaking glyph position
                    rightPanSpin.setPosition(.17);
                }
            }
        }

        //Stop the motion of leftMotor and rightMotor
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        loopCounter = 0; //Set loopCounter to 0
    }


    public void spinMove (double desiredAngle, boolean pointSix, double time, boolean up)
    {
        if (pointSix) //If boolean pointThree is true, we set the initial spinPower to .3 for a slower turn
        {
            spinPower = .62;
        }
        else  { //Else set initial spinPower to .4845
            spinPower = .4645;
        }




        //Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        initialHeading = this.realAngle(); //Set initialHeading to the current angle
        //Telemetry for initial heading
        telemetry.addData("init angle", initialHeading);
        telemetry.update();


        if (desiredAngle<initialHeading) { //If desiredAngle is less than initialHeading
            while (this.realAngle() > desiredAngle && timeTwo - timeOne < time) { //While current angle is greater than desiredAngle and less seconds than the time variable have elapsed
                if (Math.abs(this.realAngle() - desiredAngle) < 80) { //Brake: if currentAngle and desiredAngle are within 80 degrees of each other
                    loopCounter++; //Positively increment loopCounter by 1
                    spinPower -= loopCounter * .0085; //Decrease spinPower by loopCounter times .0085
                }

                spinPower = Range.clip(spinPower, .265, 1); //Ensure spinPower is between .27 and 1
                leftMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                rightMotor.setPower(-spinPower); //Set rightMotor's power to negative spinPower
                timeTwo = this.getRuntime(); //Set timeTwo to the current run time

                if (up) //If down is true then we are about to run driveToTouch, and want to move the touchServo down to do so during spinMove to save time
                {
                    leftPanSpin.setPosition(.92);
                    rightPanSpin.setPosition(.8);
                }
            }

            //Stop the motion of leftMotor and rightMotor
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        else { //If desiredAngle is less than initialHeading
            while (this.realAngle() < desiredAngle && timeTwo - timeOne < time) { //While current angle is less than desiredAngle and less seconds than the time variable have elapsed
                if (Math.abs(this.realAngle() - desiredAngle) < 80) { //Brake: if currentAngle and desiredAngle are within 80 degrees of each other
                    loopCounter++; //Positively increment loopCounter by 1
                    spinPower -= loopCounter * .0085; //Decrease spinPower by loopCounter times .0085
                }
                spinPower = Range.clip(spinPower, .265, 1); //Ensure spinPower is between .27 and 1
                leftMotor.setPower(-spinPower); //Set leftMotor's power to negative spinPower
                rightMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                timeTwo = this.getRuntime(); //Set timeTwo to the current run time

                if (up) //If down is true then we are about to run driveToTouch, and want to move the touchServo down to do so during spinMove to save time
                {
                    leftPanSpin.setPosition(.92);
                    rightPanSpin.setPosition(.8);
                }

            }

            //Stop the motion of leftMotor and rightMotor
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        //Set loopCounter to 0
        loopCounter = 0;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.


        //The range of the imu is only -180 to 180, however we want this range to extend infinitely in both directions,
        //and the algorithim below does this and outputs the value of the z-axis angle
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double realAngle () //Return this.getAngle() to output the angle of the robot
    {
        return this.getAngle();
    }

    //Method to strafe until a distance sensor sees a wall of the cryptobox, at which time we score a glpyh
    public void strafeToTouch(double time, double power, double desiredAngle) {

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //If power is greater than 0
        if (power > 0) {

            //While less seconds than the parameter time have elapsed and the centimeter value of sensorTouch is not less than 5
            while (!(sensorTouch.getDistance(DistanceUnit.CM) < 6.2) && timeTwo - timeOne < time) {
                //Set correction to the current angle minus the desired angle plus 7 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) + 7) * .027;

                //Set the power of backMotor to parameter power
                backMotor.setPower(power);

                //If the absolute value of correction is greater than .08
                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction); //Set the power of leftMotor to correction
                    rightMotor.setPower(-correction); //Set the power of rightMotor to negative correction
                } else { //Else
                    //Stop the movement of leftMotor and rightMotor
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();
            }

            //Stop the movement of backMotor, leftMotor, and rightMotor
            backMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        else { //Else

            //While less seconds than the parameter time have elapsed and the centimeter value of sensorTouch is not less than 6
            while (!(sensorTouch.getDistance(DistanceUnit.CM) < 7) && timeTwo - timeOne < time) {

                //Set correction to the current angle minus the desired angle minus 8 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) - 8) * .027;

                //Set the power of backMotor to parameter power
                backMotor.setPower(power);

                //If the absolute value of correction is greater than .08
                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction); //Set the power of leftMotor to correction
                    rightMotor.setPower(-correction); //Set the power of rightMotor to negative correction
                } else {
                    //Stop the movement of leftMotor and rightMotor
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();

            }

            //Stop the movement of backMotor, leftMotor, and rightMotor
            backMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }


    public void checkAutoMenu()
    {

        //checkGamepads();

        telemetry.addLine("USE GAMEPAD 1 FOR SELECTING OPTIONS");
        telemetry.update();
        delayLong();

        while (notAllChosen)
        {

            goBack = false;

            if (stepInMenu == 1){
                setGlyphAttackCorner();
            }

            delayShort();


            if ( !glyphPositionNotSelected ) {
                notAllChosen = false;
            }
            else {
                notAllChosen = true;
            }

        }

        telemetry.addLine("GO BOMB SQUAD");
        telemetry.update();
        delayLong();

    }

    public void setGlyphAttackCorner ()
    {

        telemetry.clearAll();
        choiceNotSelected = true;

        while  (choiceNotSelected) {
            if (glyphPositionNotSelected){
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addLine("Choose Glyph Attack Position");
                telemetry.addLine("B = RIGHT");
                telemetry.addLine("Y = MIDDLE");
                telemetry.addLine("A = LEFT");
                telemetry.update();
                if (gamepad1.b) {
                    turnPosition = 1;
                    attackConfirmation = "RIGHT";
                    glyphPositionNotSelected = false;
                    delayShort();

                }
                if (gamepad1.y) {
                    turnPosition=2;
                    attackConfirmation = "MIDDLE";
                    glyphPositionNotSelected = false;
                    delayShort();

                }
                if (gamepad1.a) {
                    turnPosition = 3;
                    attackConfirmation = "LEFT";
                    glyphPositionNotSelected = false;
                    delayShort();

                }

            }

            if (!glyphPositionNotSelected) {
                telemetry.addData("STEP NUMBER = ", stepInMenu);
                telemetry.addLine("Gamepad 1 ONLY");
                telemetry.addData("Confirm Glyph Attack Choice", attackConfirmation);

                telemetry.addLine("Y is Correct.  A is Incorrect");

                telemetry.update();
                if (gamepad1.y){
                    choiceNotSelected = false;
                    glyphPositionNotSelected = false;
                    delayShort();

                }
                if (gamepad1.a) {
                    glyphPositionNotSelected = true;
                    delayShort();

                }

            }

            telemetry.update();

        }

        telemetry.clearAll();
    }

    public void delayShort ()
    {
        timeFive = this.getRuntime();
        timeSix = this.getRuntime();

        while (timeSix - timeFive < .25) {
            timeSix = this.getRuntime();
        }
    }

    public void delayLong ()
    {
        timeFive = this.getRuntime();
        timeSix = this.getRuntime();

        while (timeSix - timeFive < 1.5) {
            timeSix = this.getRuntime();
        }
    }

    public void pivot (double desiredAngle, double time)
    {

        spinPower = .4945;

        //Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        initialHeading = this.realAngle(); //Set initialHeading to the current angle
        //Telemetry for initial heading
        telemetry.addData("init angle", initialHeading);
        telemetry.update();


        if (desiredAngle<initialHeading) { //If desiredAngle is less than initialHeading
            while (this.realAngle() > desiredAngle && timeTwo - timeOne < time) { //While current angle is greater than desiredAngle and less seconds than the time variable have elapsed
                if (Math.abs(this.realAngle() - desiredAngle) < 80) { //Brake: if currentAngle and desiredAngle are within 80 degrees of each other
                    loopCounter++; //Positively increment loopCounter by 1
                    spinPower -= loopCounter * .0085; //Decrease spinPower by loopCounter times .0085
                }

                spinPower = Range.clip(spinPower, .275, 1); //Ensure spinPower is between .27 and 1
                leftMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                rightMotor.setPower(-spinPower/2);
                timeTwo = this.getRuntime(); //Set timeTwo to the current run time
            }

            //Stop the motion of leftMotor and rightMotor
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        else { //If desiredAngle is less than initialHeading
            while (this.realAngle() < desiredAngle && timeTwo - timeOne < time) { //While current angle is less than desiredAngle and less seconds than the time variable have elapsed
                if (Math.abs(this.realAngle() - desiredAngle) < 80) { //Brake: if currentAngle and desiredAngle are within 80 degrees of each other
                    loopCounter++; //Positively increment loopCounter by 1
                    spinPower -= loopCounter * .0085; //Decrease spinPower by loopCounter times .0085
                }
                spinPower = Range.clip(spinPower, .275, 1); //Ensure spinPower is between .27 and 1
                rightMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                leftMotor.setPower(-spinPower/2);
                timeTwo = this.getRuntime(); //Set timeTwo to the current run time

            }

            //Stop the motion of leftMotor and rightMotor
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }

        //Set loopCounter to 0
        loopCounter = 0;
    }
    public void raisePan()
    {
        leftPanSpin.setPosition(.92);
        rightPanSpin.setPosition(.8);
    }
    public void lowerPan()
    {
        leftPanSpin.setPosition(.42); //Set leftPanSpin to position .21, as this is the intaking glyph position
        rightPanSpin.setPosition(.3);
    }
    public void grab()
    {
        frontPanGrip.setPosition(.964);
        backPanGrip.setPosition(.812);
    }
    public void release()
    {
        frontPanGrip.setPosition(.258);
        backPanGrip.setPosition(.112);
    }
    public void frontBarDown ()
    {
        touchServo.setPosition(.45);
    }


} //End class
