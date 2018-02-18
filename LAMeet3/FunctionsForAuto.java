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
    DcMotor relicMotor; //Motor to extend the relic scoring mechanism
 
    //Intake 
    DcMotor rightIntakeMotor; //Motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Motor that controls the left intake/left wheel of the intake
    
    //Block Scoring
    DcMotor panLifterMotor; //Motor that lifts and lowers the block scoring mechanism, known as the "pan"

    //SERVOS

    //Ball Scoring
    Servo feelerRaise; //Servo that lifts and lowers the ball scoring mechanism, known as the "feeler"
    Servo feelerSwipe; //Servo that swipes the ball once feelerRaise is lowered to score the ball

    //Block Rotation
    Servo leftPanSpin; //Servo on the left side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks
    Servo rightPanSpin; //Servo on the right side of the robot that rotates the pan, or block scoring mechanism, in order to score blocks

    //Touch driving
    Servo touchServo; //Servo that extends an arm from the front of the robot to detect when we are ready to score in autonomous

    //COLOR SENSOR
    ColorSensor sensorColorFeeler; //Color sensor for detecting ball color in autonomous

    //BLOCK SENSORS

    DistanceSensor sensorA; //Distance sensor closest to the intake to see how far away potential blocks are
    DistanceSensor sensorB; //Distance sensor between A and C, to see how far potential blocks are
    DistanceSensor sensorC; //Distance sensor farthest from intake, to see how far potential blocks are
    DistanceSensor sensorTouch; //Distance sensor on the front of the robot to see how far we are from the cryptobox

    //Time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne; //timeOne, first time variable
    double timeTwo; //timeTwo, second time variable

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
    final double FEELER_SWIPE_CW_POSITION = .2; //Clockwise turned position of feelerSwipe
    final double FEELER_SWIPE_CCW_POSITION = .95; //Counter-clockwise turned position of feelerSwipe
    final double FEELER_RAISE_UP_POSITION = .9; //Position that the feelerRaise is set to when we are not scoring the ball
    double FEELER_RAISE_DOWN_POSITION = .1; //Position that the feelerRaise is set to when we are scoring the ball

    //Vuforia
    VuforiaLocalizer vuforia; //Variable to store our instance of the Vuforia localization engine
    RelicRecoveryVuMark vuMark; //Variable to store the reading from Vuforia, either left, center, or right
    String vuMarkChecker = new String(); //Variable to store the reading from Vuforia in String form
    VuforiaTrackable relicTemplate; //Variable to load the VuMarks in Relic Recovery
    VuforiaTrackables relicTrackables; //Variable to load the VuMarks in Relic Recovery

    //Limit Switches
    DigitalChannel limitTop; //Limit Switch that tells us if we reach the top of the robot with the Pan
    DigitalChannel limitBottom; //Limit switch that tells us if we reach the bottom of the robot with the Pan

    //IMU
    BNO055IMU imu; //IMU sensor for detecting the angle of the robot

    //IMU variables
    Orientation lastAngles = new Orientation(); //Angle state used for updating telemetry
    double initialHeading; //The angle of the robot at the beginning of a given method
    double globalAngle, currentAngle; //The angle of the robot at any given moment
    double correction; //A value to add to or subtract from our driving power based on our angle error

    //Pan spin variables
    final double PAN_SPIN_UP = .66; //The value to which we set the pan spin motors when we are scoring blocks
    double PAN_SPIN_DOWN = .3; //The value to which we set the pan spin motors when we are not scoring/intaking blocks
    double bothBlockCounter = 0; //Double representing the number of iterations of a loop in which sensors B and C both see a block


    //Configures all hardware devices, and sets them to their initial values, if necessary
    public void configure( String initialAllianceColor, String initialRobotStartingPosition ) {

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
        rightPanSpin = hardwareMap.servo.get("s2"); //s2
        feelerRaise = hardwareMap.servo.get("s8"); //s8
        feelerSwipe = hardwareMap.servo.get("s9"); //s9
        touchServo = hardwareMap.servo.get("s10"); //s10

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
        sensorColorFeeler = hardwareMap.get(ColorSensor.class, "i1"); //i1
        sensorColorFeeler.enableLed(true); //Enable the LED of the color sensor
        sensorA = hardwareMap.get(DistanceSensor.class, "i2"); //i2
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
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        feelerRaise.setDirection(Servo.Direction.FORWARD); //Set feelerRaise to FORWARD direction
        feelerSwipe.setDirection(Servo.Direction.REVERSE); //Set feelerSwipe to REVERSE direction
        touchServo.setDirection(Servo.Direction.REVERSE); //Set touchServo to REVERSE direction

        //Init values
        feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to FEELER_SWIPE_CCW_POSITION
        feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Set feelerRaise to FEELER_RAISE_UP_POSITION
        leftPanSpin.setPosition(.2475); //Set the position of leftPanSpin to .2175
        rightPanSpin.setPosition(.2675); //Set the position of rightPanSpin to .2375
        touchServo.setPosition(.65); //Set the position of touchServo to .65
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
        realAngle(); //Call getAngleSimple(), which returns the current angle of the robot
    }

    //Method to score either one or two blocks during autonomous
    public void scoreBlock (double angle) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        //For .775 seconds, set panLifterMotor to .55 power and during this time, set leftPanSpin to the .3 position and rightPanSpin to the .32 position
        while (timeTwo-timeOne < .775)
        {
            timeTwo = this.getRuntime();
            panLifterMotor.setPower(.55);
            leftPanSpin.setPosition(.3);
            rightPanSpin.setPosition(.32);
        }
        panLifterMotor.setPower(0); //Stop the movement of panLifterMotor

        leftPanSpin.setPosition(PAN_SPIN_UP); //Set leftPanSpin to the PAN_SPIN_UP position
        rightPanSpin.setPosition(PAN_SPIN_UP + .02); //Set rightPanSpin to the PAN_SPIN_UP position plus .02

        pause(.7); //pause for .7 seconds

        driveNewIMU(2.8, 5, .3, true, angle); //Drive forward for 2.8 inches at .3 power, with a 5 second time limit, maintaining the parameter angle degree heading

        leftPanSpin.setPosition(PAN_SPIN_DOWN); //Set leftPanSpin to the PAN_SPIN_DOWN position
        rightPanSpin.setPosition(PAN_SPIN_DOWN + .02); //Set rightPanSpin to the PAN_SPIN_DOWN position plus .02

        pause(.2); //pause for .2 seconds

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //For .5 seconds, set panLifterMotor to -.55 power
        while (timeTwo-timeOne < .5)
        {
            timeTwo = this.getRuntime();
            panLifterMotor.setPower(-.55);
        }
        panLifterMotor.setPower(0); //Stop the movement of panLifterMotor

    }

    public void touch (boolean blue, boolean blueCorner, boolean redCorner)
    {
        if (blue) { //If blue is true
            if (blueCorner) { //If blueCorner is true
                driveNewIMU(10.25, 4, .4, true, -90); //Drive forward for 10.25 inches at .4 power with a 4 second limit maintaining a -90 degree heading

                pause(.1); //Pause for .1 seconds

                driveNewIMU(3, 3, -.3, false, -90); //Drive backwards for 2.94 inches at -.4 power with a 3 second limit maintaining a -90 degree heading

                pause(.1); //Pause for .1 seconds

                //spinMove(-90, true); //Spin move to -90 degrees starting at .3 drive motor power

                pause(.1); //Pause for .1 seconds

                touchServo.setPosition(.02); //Set touchServo to position .02

                pause(.4); //Pause for .4 seconds

                strafeToTouch(2.5, .65, -90); //Run strafeToTouch method at -90 degrees and .63 power, with a 2.5 second timeout


                pause(.2); //Pause for .2 seconds

                touchServo.setPosition(.65); //Set the position of touchServo to .65

                pause(.2); //Pause for .2 Seconds

                driveNewIMU(2.2, 4, .4, true, -90);

                pause(.2); //Pause for .2 Seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.2); //Pause for .2 seconds

                driveNewIMU(2.6, 5, -.4, false, -90); //Drive backwards 2.25 inches at -.4 power keeping a -90 degree heading with a 5 second limit

                pause(.15); //pause for .15 seconds

                //Outtake for 2 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();
                
                while (timeTwo - timeOne < 2) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.1); //.1 second pause

                driveNewIMU(4, 2, -.4, false, -90); //Drive backwards 4 inches at -.4 power keeping a -90 degree heading with a 2 second limit

                pause(.15); //pause for .15 seconds

                //Set the power of leftIntakeMotor to -.15 and the power of rightIntakeMotor to -.17
                leftIntakeMotor.setPower(-.15);
                rightIntakeMotor.setPower(-.17);

                driveNewIMU(16, 4, .4, true, -90); //Drive forwards 24 inches at .4 power keeping a -90 degree heading with a 4 second limit

                pause(.15); //pause for .15 seconds

                driveNewIMU(4, 3, -.4, false, -90); //Drive backwards 4 inches at -.4 power keeping a -90 degree heading with a 3 second limit

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

            }

            else
            {
                driveNewIMU(10.25, 4, .4, true, 179); //Drive forward for 10.25 inches at .4 power with a 4 second limit maintaining a 179 degree heading

                pause(.1); //Pause for .1 seconds

                driveNewIMU(3.9, 3, -.3, false, 179); //Drive backwards for 2.94 inches at -.4 power with a 3 second limit maintaining a 179 degree heading

                pause(.1); //Pause for .1 seconds

                //spinMove(179, true); //Spin move to 179 degrees starting at .3 drive motor power

                pause(.1); //Pause for .1 seconds

                touchServo.setPosition(.02); //Set touchServo to position .02

                pause(.4); //Pause for .4 seconds

                strafeToTouch(3.5, .65, 178); //Run strafeToTouch method at 178 degrees and .63 power, with a 3.5 second timeout


                pause(.2); //Pause for .2 seconds

                touchServo.setPosition(.65); //Set the position of touchServo to .65

                pause(.2); //Pause for .2 Seconds

                driveNewIMU(2.2, 4, .4, true, 180);

                pause(.2); //Pause for .2 Seconds
                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.2); //Pause for .2 seconds

                driveNewIMU(2.6, 5, -.4, false, 179); //Drive backwards 2.25 inches at -.4 power keeping a 179 degree heading with a 5 second limit

                pause(.15); //pause for .15 seconds

                //Outtake for 2 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 2) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.1); //.1 second pause

                driveNewIMU(4, 2, -.4, false, 179); //Drive backwards 4 inches at -.4 power keeping a 179 degree heading with a 2 second limit

                //Set the power of leftIntakeMotor to -.15 and the power of rightIntakeMotor to -.17
                leftIntakeMotor.setPower(-.15);
                rightIntakeMotor.setPower(-.17);

                driveNewIMU(16, 4, .4, true, 179); //Drive forwards 24 inches at .4 power keeping a 179 degree heading with a 4 second limit

                pause(.15); //pause for .15 seconds

                driveNewIMU(4, 3, -.4, false, 179); //Drive backwards 4 inches at -.4 power keeping a 179 degree heading with a 3 second limit

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

            }
        }

        else {
            if (redCorner) {
                driveNewIMU(10.25, 4, .4, true, -90); //Drive forward for 10.25 inches at .4 power with a 4 second limit maintaining a -90 degree heading

                pause(.1); //Pause for .1 seconds

                driveNewIMU(3, 3, -.3, false, -90); //Drive backwards for 2.34 inches at -.4 power with a 3 second limit maintaining a -90 degree heading

                pause(.1); //Pause for .1 seconds

                //spinMove(-90, true); //Spin move to -90 degrees starting at .3 drive motor power

                pause(.1); //Pause for .1 seconds

                touchServo.setPosition(.02); //Set touchServo to position .02

                pause(.4); //Pause for .4 seconds

                strafeToTouch(3.5, .65, -88); //Run strafeToTouch method at -88 degrees and .63 power, with a 3.5 second timeout

                pause(.2); //Pause for .2 seconds

                touchServo.setPosition(.65); //Set the position of touchServo to .65

                pause(.2); //Pause for .2 Seconds

                driveNewIMU(2.2, 4, .4, true, -90);

                pause(.2); //Pause for .2 Seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.2); //Pause for .2 seconds

                driveNewIMU(2.6, 5, -.4, false, -90); //Drive backwards 2.25 inches at -.4 power keeping a -90 degree heading with a 5 second limit

                pause(.15); //pause for .15 seconds

                //Outtake for 2 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 2) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.1); //.1 second pause

                driveNewIMU(4, 2, -.4, false, -90); //Drive backwards 4 inches at -.4 power keeping a -90 degree heading with a 2 second limit

                //Set the power of leftIntakeMotor to -.15 and the power of rightIntakeMotor to -.17
                leftIntakeMotor.setPower(-.15);
                rightIntakeMotor.setPower(-.17);

                driveNewIMU(16, 4, .4, true, -90); //Drive forwards 24 inches at .4 power keeping a -90 degree heading with a 4 second limit

                pause(.15); //pause for .15 seconds

                driveNewIMU(4, 3, -.4, false, -90); //Drive backwards 4 inches at -.4 power keeping a 90 degree heading with a 3 second limit

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);
            }
            else
            {
                driveNewIMU(10.25, 4, .4, true, 0); //Drive forward for 10.25 inches at .4 power with a 4 second limit maintaining a 0 degree heading

                pause(.1); //Pause for .1 seconds

                driveNewIMU(3.9, 3, -.3, false, 0); //Drive backwards for 2.94 inches at -.4 power with a 3 second limit maintaining a 0 degree heading

                pause(.1); //Pause for .1 seconds

                //spinMove(0, true); //Spin move to 0 degrees starting at .3 drive motor power

                pause(.1); //Pfause for .1 seconds

                touchServo.setPosition(.02); //Set touchServo to position .02

                pause(.4); //Pause for .4 seconds

                strafeToTouch(3.5, .65, 0); //Run strafeToTouch method at 0 degrees and .63 power, with a 3.5 second timeout

                pause(.2); //Pause for .2 seconds

                touchServo.setPosition(.65); //Set the position of touchServo to .65

                pause(.2); //Pause for .2 Seconds

                driveNewIMU(2.6, 4, .4, true, 0);

                pause(.2); //Pause for .2 Seconds

                //Set leftIntakeMotor to -.6 power and rightIntakeMotor to -.7 power
                leftIntakeMotor.setPower(-.6);
                rightIntakeMotor.setPower(-.7);

                pause(.2); //Pause for .2 seconds

                driveNewIMU(2.25, 5, -.4, false, 0); //Drive backwards 2.25 inches at -.4 power keeping a 0 degree heading with a 5 second limit

                pause(.15); //pause for .15 seconds

                //Outtake for 2 seconds
                timeOne = this.getRuntime();
                timeTwo = this.getRuntime();

                while (timeTwo - timeOne < 2) {
                    leftIntakeMotor.setPower(-.6);
                    rightIntakeMotor.setPower(-.7);
                    timeTwo = this.getRuntime();
                }

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);

                pause(.1); //.1 second pause

                driveNewIMU(4, 2, -.4, false, 0); //Drive backwards 4 inches at -.4 power keeping a 0 degree heading with a 2 second limit

                //Set the power of leftIntakeMotor to -.15 and the power of rightIntakeMotor to -.17
                leftIntakeMotor.setPower(-.15);
                rightIntakeMotor.setPower(-.17);

                driveNewIMU(16, 4, .4, true, 0); //Drive forwards 24 inches at .4 power keeping a 0 degree heading with a 4 second limit

                pause(.15); //pause for .15 seconds

                driveNewIMU(4, 3, -.4, false, 0); //Drive backwards 4 inches at -.4 power keeping a 0 degree heading with a 3 second limit

                //Stop the motion of the intake motors
                leftIntakeMotor.setPower(0);
                rightIntakeMotor.setPower(0);
            }
        }

    }


    //Method to collect blocks during autonomous
    public void getBlocks(double distance) {

        inches = distance; //Set inches to parameter distance
        rotations = inches / (Math.PI * WHEEL_DIAMETER); //Set rotations to inches divided by pi, approximately 3.14, divided by the wheel diameter
        counts = ENCODER_CPR * rotations * GEAR_RATIO; //Set counts to the encoder CPR times rotations times the gear ratio

        loopCounter = 1000; //Set loopCounter to 1000

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of rightMotor to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //Set run mode of rightMotor to RUN_WITHOUT_ENCODER

        inches2 = 42; //Set inches2 to 60
        rotations2 = inches2 / (Math.PI * WHEEL_DIAMETER); //Set rotations2 to inches2 divided by pi, approximately 3.14, divided by the wheel diameter
        counts2= ENCODER_CPR * rotations2 * GEAR_RATIO; //Set counts2 to the encoder CPR times rotations2 times the gear ratio

        leftPanSpin.setPosition(.2675); //Set the position of leftPanSpin to .2175
        rightPanSpin.setPosition(.2875); //Set the position of rightPanSpin to .2375

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne= this.getRuntime();
        timeTwo=this.getRuntime();

        //While bothBlockCounter is under 5 and the current position of rightMotor is less than counts2 and less than 10 seconds have elapsed
        while (bothBlockCounter < 5 && rightMotor.getCurrentPosition()<(counts2) && timeTwo-timeOne < 5.4)
        {
            leftIntakeMotor.setPower(.85); //Set leftIntakeMotor to .85 power
            rightIntakeMotor.setPower(1); //Set rightIntakeMotor to 1 power
            timeTwo=this.getRuntime(); //Set timeTwo to this.getRuntime()
            //loopCounter++; //Positively increment loopCounter by 1

            //currentAngle = this.realAngle(); //Set currentAngle to the current angle of the robot

            //If the distance sensed by sensors B and C are less than 13, positively increment bothBlockCounter by 1
            if (sensorB.getDistance(DistanceUnit.CM) < 13 && sensorC.getDistance(DistanceUnit.CM) < 13)
            {
                bothBlockCounter++;
            }


            leftMotor.setPower(.5);
            rightMotor.setPower(.5);


            //Telemetry
            //telemetry.addData("left", leftMotor.getPower());
            //telemetry.addData("loopCounter", loopCounter);
            telemetry.addData("blocks", bothBlockCounter);
            telemetry.update();
        }

        //Stop the motion of leftMotor and rightMotor
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        loopCounter = 0;//Set loopCounter to 0

        //Stop the motion of leftIntakeMotor and rightIntakeMotor
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        //Set the position of leftPanSpin to .3 and the position of rightPanSpin to .32
        leftPanSpin.setPosition(.3);
        rightPanSpin.setPosition(.32);

        bothBlockCounter= 0; //Set the value of bothBlockCounter to 0

        pause(.5); //Pause for .5 seconds (allow time for pan servos to raise)

        leftIntakeMotor.setPower(-.5);
        rightIntakeMotor.setPower(-.5);

        driveNewIMU (88, 5, -.4, false, 90); //Drive backwards for 88 inches at -.4 power, keeping a 90 degree heading and timing out after 5 seconds

        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);

        pause(.05); //Pause for .05 seconds

        driveNewIMU(2.25, 5, .3, true, 90); //Drive forwards for 2.25 inches at .3 power, keeping a 90 degree heading and timing out after 5 seconds

    }


    //Method to detect the Vuforia reading
    public String detectVuMark(int timeToCheck) {

        relicTrackables.activate(); //Activate relicTrackables

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

            //Telemetry
            telemetry.addData("Time: ", timeTwo - timeOne);
            telemetry.addData("Vu mark detector: ", vuMarkChecker);
            telemetry.update();
        }
        return vuMarkChecker; //Return the String vuMarkChecker
    }


    //Method to return the vuMark as a string
    public String vumarkToString() {
        String output; //Declare local String variable output
        output = "" + vuMark; //Set output to vuMark in String form
        return output; //return output
    }

    // Sets all drive train motors to 0 power
    public void stopDriving() {
        leftMotor.setPower(0); //Set power of leftMotor to 0
        rightMotor.setPower(0); //Set power of rightMotor to 0
        backMotor.setPower(0); //Set power of backMotor to 0
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

        feelerRaise.setPosition(.6); //Set feelerRaise to position .6

        pause(.7); //Pause for .7 seconds

        if (allianceColor.equalsIgnoreCase("blue")) {
            feelerSwipe.setPosition(FEELER_SWIPE_NEUTRAL_POSITION_BLUE); //Set feelerSwipe to FEELER_SWIPE_NEUTRAL_POSITION_BLUE
        }
        else
        {
            feelerSwipe.setPosition(FEELER_SWIPE_NEUTRAL_POSITION_RED); //Set feelerSwipe to FEELER_SWIPE_NEUTRAL_POSITION_RED
        }


        pause (.7); //.7 second pause

        feelerRaise.setPosition(FEELER_RAISE_DOWN_POSITION); //Lower feelerRaise to put ball scoring mechanism next to the balls

        pause(1.5); //1 second pause

        if (allianceColor.equalsIgnoreCase("red") && sensorColorFeeler.blue() >=  sensorColorFeeler.red()) { //If we are the red alliance and see a blue ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red());
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue());
            telemetry.update(); //Update telemetry
            pause(.1); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CW_POSITION); //Set feelerSwipe to its clockwise position
            pause(.1); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.1); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("red") && sensorColorFeeler.red() >= sensorColorFeeler.blue()) { //If we are the red alliance and see a red ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.1); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to its counter-clockwise position
            pause(.1); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.1); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColorFeeler.blue() >= sensorColorFeeler.red()) { //If we are the blue alliance and see a blue ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.1); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CCW_POSITION); //Set feelerSwipe to its counter-clockwise position
            pause(.1); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.1); //.1 second pause
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColorFeeler.red() >= sensorColorFeeler.blue()) { //If we are the blue alliance and see a red ball with the color sensor
            //Display red and blue values of the color sensor on telemetry
            telemetry.addData("Value of RED: ", sensorColorFeeler.red() );
            telemetry.addData("Value of BLUE: ", sensorColorFeeler.blue() );
            telemetry.update(); //Update telemetry
            pause(.1); //.1 second pause
            feelerSwipe.setPosition(FEELER_SWIPE_CW_POSITION); //Set feelerSwipe to its clockwise position
            pause(.1); //.1 second pause
            feelerRaise.setPosition(FEELER_RAISE_UP_POSITION); //Raise feelerRaise after ball is knocked off
            pause(.1); //.1 second pause
        }

    }

    //Method to format the angle of the robot
    public String formatAngle(AngleUnit angleUnit, double angle) {
        //Return the formatDegrees angle using the parameters below
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    //Method to return degree of the robot
    public String formatDegrees(double degrees) {
        //Return a String, formatted using the parameters below
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
                //Set correction to the current angle minus the desired angle plus 7 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) + 7) * .027;

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

                //Set correction to the current angle minus the desired angle minus 8 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) - 8)  * .027;

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

            //Set correction to the current angle minus the desired angle times .016
            correction = (this.realAngle()-desiredAngle) * .016;

            //Telemetry
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("Angle", this.realAngle());
            telemetry.addData("inches", inches);
            telemetry.addData("power", power);
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
                //If the absolute value of rightMotor's position is greater than the absolute value of counts - 700, and loopCounter is under 100
                //This serves to not immediately start decreasing the speed of the motors and only brake when necessary
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 700) && loopCounter < 100) {
                    power -= loopCounter * .01; //Subtract loopCounter times .01 from power
                    power = Range.clip(power, .27, 1); //Ensure power is between .27 and 1
                }
            }
            //Lower the absolute value of the power of leftMotor and rightMotor as the drive progresses in order to brake
            else //If robot is not moving forward
            {
                //If the absolute value of rightMotor's position is greater than the absolute value of counts - 700, and loopCounter is under 100
                //This serves to not immediately start decreasing the speed of the motors and only brake when necessary
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 700) && loopCounter < 100) {
                    power += loopCounter * .01;  //Add loopCounter times .01 from power
                    power = Range.clip(power, -1, -.27); //Ensure power is between -1 and -.27
                }
            }
        }

        //Stop the motion of leftMotor and rightMotor
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        loopCounter = 0; //Set loopCounter to 0
    }

    private double getAngleSimple() {
        //Set last Angles to the getAngularOrientation method of the imu, using the parameters specified below
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //Return the parseDouble method, using the parameters specified below.  This represents the current angle of the robot.
        return Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
    }



    public void spinMove (double desiredAngle, boolean pointThree, double time)
    {
        if (pointThree) //If boolean pointThree is true, we set the initial spinPower to .3
        {
            spinPower = .3;
        }
        else { //Else set initial spinPower to .4445
            spinPower = .4445;
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
                spinPower = Range.clip(spinPower, .275, 1); //Ensure spinPower is between .26 and 1
                leftMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                rightMotor.setPower(-spinPower); //Set rightMotor's power to negative spinPower
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
                spinPower = Range.clip(spinPower, .275, 1); //Ensure spinPower is between .26 and 1
                leftMotor.setPower(-spinPower); //Set leftMotor's power to negative spinPower
                rightMotor.setPower(spinPower); //Set leftMotor's power to spinPower
                timeTwo = this.getRuntime(); //Set timeTwo to the current run time
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

    public double realAngle ()
    {
        return this.getAngle();
    }

    public void strafeToTouch(double time, double power, double desiredAngle) {

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //If power is greater than 0
        if (power > 0) {

            //While less seconds than the parameter time have elapsed and the centimeter value of sensorTouch is not less than 6
            while (!(sensorTouch.getDistance(DistanceUnit.CM) < 5.8) && timeTwo - timeOne < time) {
                //Set correction to the current angle minus the desired angle plus 7 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) + 7) * .027;

                //Telemetry
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Angle", this.realAngle());
                telemetry.addData("inches", inches);
                telemetry.addData("power", power);
                telemetry.addData("pos", backMotor.getCurrentPosition());
                telemetry.addData("counts", counts);
                telemetry.addData("distance", sensorTouch.getDistance(DistanceUnit.CM));
                telemetry.update();

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
            while (!(sensorTouch.getDistance(DistanceUnit.CM) < 5.8) && timeTwo - timeOne < time) {

                //Set correction to the current angle minus the desired angle minus 8 (value to keep strafe straight found through experimental testing) times .027
                correction = ((this.realAngle()-desiredAngle) - 8) * .027;

                //Telemetry
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Angle", this.realAngle());
                telemetry.addData("inches", inches);
                telemetry.addData("power", power);
                telemetry.addData("pos", backMotor.getCurrentPosition());
                telemetry.addData("counts", counts);
                telemetry.addData("distance", sensorTouch.getDistance(DistanceUnit.CM));
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


} //End class
