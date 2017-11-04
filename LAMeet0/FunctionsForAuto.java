/**
 * Created by stevecox on 11/4/17.
 */
//package declaration
package org.firstinspires.ftc.teamcode;

//import statements

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public abstract class FunctionsForAuto extends LinearOpMode {

    //Variable Declarations
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    //Motors
    // DCMotors
    DcMotor rightMotor;     //right drive motor front
    DcMotor leftMotor;      //left drive motor front
    DcMotor topMotor;       //right drive motor back
    DcMotor bottomMotor;    //left drive motor back


    //  declare touch seniors
    TouchSensor touchSensorTop;
    TouchSensor touchSensorBottom;

    boolean wlsRightlight = false;  //Boolean for whether right ods has seen white light, initially set to false
    boolean wlsLeftlight = false;   //Boolean for whether left ods has seen white light, initially set to false

    //Servos
    Servo horizontalTop; // Servo that rotate's the grabber horizontally
    Servo openCloseTop; // Sevo that opens and closes the two grabbers
    Servo rightGrabberTop; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberTop; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo
    Servo horizontalBottom; // Servo that rotate's the grabber horizontally
    Servo openCloseBottom; // Sevo that opens and closes the two grabbers
    Servo rightGrabberBottom; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberBottom; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo


    //encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    //encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.6;     //Gear ratio used in Big Sur in 30/18, so in code we multiply by 18/30
    final static double WHEEL_DIAMETER = 4; //wheel diameter in inches

    double shooterPower; //power applied to the shooter
    double turretPosition;  //position set to the turret servo

    //Driving variables
    double inches;  //Desired number of inches to drive
    double rotations;       //Wheel rotations necessary to drive the above amount of inches
    double counts;//Encoder counts necessary to drive the above amount of inches/rotations

    double turnSpeed = 0; //Turn speed outputted from gyro gain during spinMove method

    ColorSensor colorSensorRight; //Right color sensor for beacon autonomous
    ColorSensor colorSensorLeft; //Left color sensor for beacon autonomous

    //Optical distance sensors to detect white light
    OpticalDistanceSensor whiteLineSensorLeft; //left
    OpticalDistanceSensor whiteLineSensorRight; //right

    int loopCounter = 0; //Variable to count how many times a given loop has been entered

    final double WHITE_THRESHOLD = .25; //The threshold of white light necessary to set the below variables to true USED TO BE .4

    //rpm variables
    //shooter variables
    //encoder count variables
    double encoderClicksOne = 0;
    double encoderClicksTwo = 0;
    //rates per minute
    int rpm;
    //double array to compound the 5 most recent rpm values
    double[] averageRpmArray = {0, 0, 0, 0, 0};
    //int value so we know when we must calculate a new average rpm.  When this value is 5 we do so.
    int arrayCount = 0;
    //the value of the sum of the last 5 rpm values
    int totalRpm = 0;
    //the value of totalRpm divided by 5: the average of the last 5 rpm values
    int avgRpm = 0;
    //the value of the average of the last 5 rpm values, weighted to give preference to more recent values
    int weightedAvg = 0;
    //the weight given to the oldest value, incremented by .05 for each more recent value
    double baseWeight;
    //a boolean set to false unless outside force is currently being applied to the motor disturbing its rpm.  Reflective of how the motor will be disturbed when a ball is being shot in our actual robot.
    double tempWeightedAvg = 0;

    //time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne;
    double timeTwo;
    double timeRunningLoop;

    //Variables to count how many times white light has been seen in a given loop
    double whitesCount; //General
    double whiteCountLeft; //Specific to only left ods
    double whiteCountRight; //Specific to only right ods

    //Gain to control rpm on the robot
    final double RPM_GAIN = .00001;

    //Positions of the beacon pushers, based on whether retracting or extending
    final double BEACON_PUSHER_LEFT_RETRACT_POSITION = .05; //left retract position
    final double BEACON_PUSHER_LEFT_EXTEND_POSITION = .95; //left extend position
    final double BEACON_PUSHER_RIGHT_RETRACT_POSITION = .95; //right retract position
    final double BEACON_PUSHER_RIGHT_EXTEND_POSITION = .05; //right extend position

    //Boolean that is true when a beacon has been extended and is false otherwise
    boolean push;

    //Boolean that is true when a white line has not been detected and is false otherwise
    boolean whiteLineNotDetected = true;

    //Boolean that is true when a beacon has not been detected and is false otherwise
    boolean beaconNotDetected = true;

    //Gyro sensor declaration
    ModernRoboticsI2cGyro gyro;

    //measure gyro heading/position
    double currentHeading;

    //measure difference between current and desired gyro heading
    double headingError;

    //variable to measure the gyro heading at the beginning of a method
    double initialHeading;

    //Gain for the gyro when executing a spin move
    final double GYRO_GAIN = .0099;

    //Gain for the gyro when driving straight
    double straightGyroGain = .025;

    //Adjustment factor for motor powers when driving straight based on straightGyroGain
    double straightDriveAdjust = 0;

    // booleans for limit switch state
    boolean touchTopPress = false;
    boolean touchBottomPress = false;

    // F U N C T I O N S   F O R   A U T O


    //Drives straight and backwards for a provided distance, in inches
    //and at a given speed and a given gyro heading
    public void driveBack(double distance, double speed, double targetHeading) {
        //Set variable initialHeading to gyro's integrated z value
        initialHeading = gyro.getIntegratedZValue();
        //math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Execute loop while the absolute value of leftMotor1's current encoder position is under target encoeder count
        //And while time ellapsed in loop is less than 4
        while (Math.abs(leftMotor1.getCurrentPosition()) < counts && (timeTwo - timeOne < 4)) {
            //Set variable initialHeading to gyro's integrated z value
            currentHeading = gyro.getIntegratedZValue();
            //Calculate motor power adjustment factor based on proportional control in order to maintain
            //the desired gyro heading
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;
            //Set motor powers based on paramater speed and straightDriveAdjust
            leftMotor1.setPower(-speed + straightDriveAdjust);
            leftMotor2.setPower(-speed + straightDriveAdjust);
            rightMotor1.setPower(-speed - straightDriveAdjust);
            rightMotor2.setPower(-speed - straightDriveAdjust);
            //Telemetry for encoder position
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            //Set timeTwo to this.getRuntime ()
            timeTwo = this.getRuntime();
        }
        //Safety timeout based on if the loop above executed in under 4 seconds
        //If it did not, do not execute the rest of the program
        if (timeTwo - timeOne > 4) {
            while (this.opModeIsActive()) {
                stopDriving();
                timeTwo = this.getRuntime();
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
            }
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Sets all drive train motors to 0 power
    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        topMotor.setPower(0);
        bottomMotor.setPower(0);
    }

    //Execute a robot spin using both sides of the drive train and the gyro
    public void spinMove(double desiredHeading) //desired gyro heading
    {
        //Set all drive train motor's run modes to RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set initialHeading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if (desiredHeading < initialHeading) //CLOCKWISE TURN
        {
            do {
                //Math to calculate turnSpeed of robot using proportional control
                //with turnSpeed lowering as robot reaches its heading target
                currentHeading = gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = -headingError * GYRO_GAIN;

                //Clip turnSpeed to be .4 through .8
                if (turnSpeed < 0.25) {
                    turnSpeed = 0.25;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }

                //Set motor powers
                rightMotor1.setPower(-turnSpeed);
                rightMotor2.setPower(-turnSpeed);
                leftMotor1.setPower(turnSpeed);
                leftMotor2.setPower(turnSpeed);
                //Execute gyroTelemetry method
                gyroTelemetry();

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();
            }
            while (currentHeading > desiredHeading && (timeTwo - timeOne < 13));
            //while current heading is greater than desired heading, and time elapsed in loop
            //is under 6 seconds
        } else //Counterclockwise turn
        {
            do {
                //Math to calculate turnSpeed of robot using proportional control
                //with turnSpeed lowering as robot reaches its heading target
                currentHeading = gyro.getIntegratedZValue();
                headingError = desiredHeading - currentHeading;
                turnSpeed = headingError * GYRO_GAIN;

                //Clip turnSpeed to be .4 through .8
                if (turnSpeed < 0.25) {
                    turnSpeed = 0.25;
                }
                if (turnSpeed > .8) {
                    turnSpeed = .8;
                }

                //Set motor powers
                rightMotor1.setPower(turnSpeed);
                rightMotor2.setPower(turnSpeed);
                leftMotor1.setPower(-turnSpeed);
                leftMotor2.setPower(-turnSpeed);
                //Execute gyroTelemetry method
                gyroTelemetry();

                //Set timeTwo to this.getRuntime()
                timeTwo = this.getRuntime();
            }
            while (currentHeading < desiredHeading && (timeTwo - timeOne < 13));
            //while current heading is less than desired heading, and time elapsed in loop
            //is under 6 seconds
        }
        //Safety timeout
        if (timeTwo - timeOne > 13) {
            stopDriving();
            while (this.opModeIsActive()) {
                timeTwo = this.getRuntime();
            }

            telemetry.addLine("Timed out");
            telemetry.update();
        }
        //Execute stopDriving method
        stopDriving();
    }

    //Provide telemetry related to the gyro sensor
    public void gyroTelemetry() {
        //Current value of the gyro's integrated z value
        telemetry.addData("Heading", gyro.getIntegratedZValue());
        //Current value of the turnSpeed variable
        telemetry.addData("Turn Speed", turnSpeed);
        telemetry.update();
    }

    //Configures all hardware devices, and sets them to their initial values, if necessary

    //WRITE IN ENCODER PORTS
    public void Configure() {

        /**
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /**
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */

        parameters.vuforiaLicenseKey = "AZfTpOj/////AAAAGYCE1z7z6E5whPRKfYeRJHEN/u/+LZ7AMmBU0bBa" +
                "/7u6aTruUWfYeLur6nSFdKP0w9JPmK1gstNxVHqiaZN6iuZGxPcbnDnm" +
                "NJdoLIMtZheeNWphUMjHKoTUgsmcloZe67TG2V9duc+8jxxCLFzH5rlq" +
                "PPdcgvvtIO0orpxVcpENBunY2GChhVgP6V5T9Iby7MyM9tN+y7Egm7Xy" +
                "Iz/Tzpmlj19b3FUCW4WUDjTNQ4JoKZeB1jkhPxKGFRECoPw02jJXtQSK" +
                "zNfzmhtugA7PTOZNehc61UjOXEexTO9TRy7ZfMtW8OggcYssvIabyJ8b" +
                "DK4ePLCUP+Q4PMf7kL9lM6yDuxxKF0oqLgRglX9Axqrf";

        /**
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1");
        leftMotor = hardwareMap.dcMotor.get("m2");
        topMotor = hardwareMap.dcMotor.get("m3");
        bottomMotor = hardwareMap.dcMotor.get("m4");

        // Motor directions: set forward/reverse
        rightMotor.setDirection(REVERSE);
        leftMotor.setDirection(FORWARD);
        topMotor.setDirection(REVERSE);
        bottomMotor.setDirection(FORWARD);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //I2C port 0

        I2cAddr i2cColorLeft = I2cAddr.create8bit(0x5c); //Create I2C address of colorSensorLeft
        I2cAddr i2cColorRight = I2cAddr.create8bit(0x3c); //Create I2C address of colorSensorRight

        //requires moving connection based on alliance color
        colorSensorRight = hardwareMap.colorSensor.get("colorright");     //I2C port 2
        colorSensorRight.setI2cAddress(i2cColorRight); //set I2C address of colorSensorRight
        colorSensorRight.enableLed(false); //Set enableLed of colorSensorRight to false

        colorSensorLeft = hardwareMap.colorSensor.get("colorleft");     //I2C port 5
        colorSensorLeft.setI2cAddress(i2cColorLeft); //set I2C address of colorSensorRight
        colorSensorLeft.enableLed(false); //Set enableLed of colorSensorLeft to false

        //Set gyro variables to 0
        currentHeading = 0;
        initialHeading = 0;
        headingError = 0;
        turnSpeed = 0;

        //Initialize encoder variables to 0
        inches = 0;
        rotations = 0;
        counts = 0;

        // harware map configurations
        horizontalTop = hardwareMap.servo.get("s1");
        openCloseTop = hardwareMap.servo.get("s2");
        rightGrabberTop = hardwareMap.servo.get("s4");
        leftGrabberTop = hardwareMap.servo.get("s3");
        horizontalBottom = hardwareMap.servo.get("s5");
        openCloseBottom = hardwareMap.servo.get("s6");
        leftGrabberBottom = hardwareMap.servo.get("s7");
        rightGrabberBottom = hardwareMap.servo.get("s8");


        touchSensorTop = hardwareMap.touchSensor.get("touch1");
        touchSensorBottom = hardwareMap.touchSensor.get("touch2");


        // Set servo direction orientations forward or reverse
        horizontalTop.setDirection(Servo.Direction.FORWARD);
        openCloseTop.setDirection(Servo.Direction.FORWARD);
        rightGrabberTop.setDirection(Servo.Direction.REVERSE);
        leftGrabberTop.setDirection(Servo.Direction.FORWARD);

        horizontalBottom.setDirection(Servo.Direction.FORWARD);
        openCloseBottom.setDirection(Servo.Direction.FORWARD);
        rightGrabberBottom.setDirection(Servo.Direction.REVERSE);
        leftGrabberBottom.setDirection(Servo.Direction.FORWARD);

        // Initial positions for servos
        horizontalTop.setPosition(.486);
        openCloseTop.setPosition(.5);
        rightGrabberTop.setPosition(.5);
        leftGrabberTop.setPosition(.5);

        horizontalBottom.setPosition(.486);
        openCloseBottom.setPosition(.5);
        rightGrabberBottom.setPosition(.5);
        leftGrabberBottom.setPosition(.5);
    }

    //Calibrate the gyro sensor
    public void calibrateGyro() throws InterruptedException {
        //Run calibrate method on gyro
        gyro.calibrate();

        //While gyro is calibrating
        while (gyro.isCalibrating()) {
            //Sleep for 500 miliseconds
            sleep(500);
            //telemetry
            telemetry.addLine("Gyro is not calibrated");
            telemetry.update();

        }

    }

    //Runs the flywheel shooter, attempting to maintain a constant
    //rpm of the shooter, and also a constant speed of the intake
    //motor.  If the rpm is not close enough to the target value,
    //the intake will not run as too ensure balls are given the
    //best opportunity to score.
    public void shootAndLift(double targetRPM, double intakeSpeed, double initialWaitTime) throws InterruptedException {
        //Set shooter power to variable shooterPower
        shooter.setPower(shooterPower);

        //Set timeOne and timeTwo to this.getRuntime
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //1 second wait
        while (timeTwo - timeOne < initialWaitTime) {
            timeTwo = this.getRuntime();
        }

        //Set timeOne and timeTwo and timeRunningLoop to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        timeRunningLoop = this.getRuntime();

        //3 second loop
        while (timeTwo < (timeRunningLoop + 3)) {

            //Current Run Time
            timeTwo = this.getRuntime();
            //Current Encoder Clicks
            encoderClicksTwo = shooter.getCurrentPosition();

            //telemetry for shooting speed
            if (timeTwo - timeOne >= 0.1) {//if timeTwo and timeOne are more than .1 sec apart
                timeTwo = this.getRuntime();//set time Two to curret runtime
                encoderClicksTwo = shooter.getCurrentPosition();//set encoderClicksTwo to the current position of the shooter motor
                rpm = (int) ((encoderClicksTwo - encoderClicksOne) / (timeTwo - timeOne) * (60 / 28)); //(clicks/seconds)(60seconds/1min)(1rev/28clicks)
                averageRpmArray[arrayCount] = rpm; //Set position arrayCount of averageRpmArray to current rpm
                timeOne = this.getRuntime(); //set timeOne to current run time
                encoderClicksOne = shooter.getCurrentPosition(); //set encoderClicksOne to the current position of the shooter motor
                arrayCount++;//increment arrayCount by 1
            }


            if (arrayCount == 5) //if arrayCount equals 5
            {
                for (int i = 0; i < 5; i++) { //loop 5 times
                    totalRpm += averageRpmArray[i]; //increment totalRpm by the value at position i of averageRpmArray
                }
                avgRpm = (int) totalRpm / 5; //set avgRpm to totalRpm divided by five casted as an int
                baseWeight = .1; //Set base weight to .1
                for (int i = 0; i < 5; i++) { //Loop 5 times
                    weightedAvg += (int) averageRpmArray[i] * baseWeight; //Increment weightedAvg by the value of averageRpmArray at position i times baseWeight casted as an int
                    baseWeight += .05; //Increment base weight by .05
                }
                //Set tempWeightedAvg to weightedAvg
                tempWeightedAvg = weightedAvg;

                //If avgRpm>targetRPM, set intake powerto intakeSpeed, otherwise
                //set intake power to 0
                if (avgRpm > targetRPM) {
                    intake.setPower(intakeSpeed);
                    ballControl.setPosition(.6);
                } else {
                    intake.setPower(0);
                }

                //Adjust shooter speed based on RPM_GAIN, and difference between
                //targetRPM and weightedAvg
                shooterPower += RPM_GAIN * (targetRPM - weightedAvg);

                //Telemetry
                weightedAvg = 0; //set weightedAvg to 0
                arrayCount = 0; //set arrayCount to 0
                //set each value in averageRpmArray to 0
                averageRpmArray[0] = 0;
                averageRpmArray[1] = 0;
                averageRpmArray[2] = 0;
                averageRpmArray[3] = 0;
                averageRpmArray[4] = 0;
                //set totalRpm to 0;
                totalRpm = 0;
                //Set shooter power to variable shooterPower
                shooterPower = Range.clip(shooterPower, .8, .99);
                shooter.setPower(shooterPower);
            }

            //telemetry for rpm and averages
            telemetry.addData("WeightedRPM: ", tempWeightedAvg);
            telemetry.addData("RPM : ", rpm);
            telemetry.addData("AvgRPM : ", avgRpm);
            telemetry.update();
        }

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //.5 second pause
        while (timeTwo - timeOne < 2) {
            shooter.setPower(.96);
            timeTwo = this.getRuntime();
        }

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        //while timeTwo < timeRunningLoop + 6
        while (timeTwo - timeOne < 4) {
            timeTwo = this.getRuntime();
            shooter.setPower(.95);
            kicker.setPosition(.8);
            intake.setPower(.9);
        }

        //telemetry for rpm and averages
        telemetry.addData("WeightedRPM: ", tempWeightedAvg);
        telemetry.addData("RPM : ", rpm);
        telemetry.addData("AvgRPM : ", avgRpm);
        telemetry.update();


        //Execute stopShooting method
        stopShooting();

        //Set intake and shooter powers to 0, and kicker position to 0
        kicker.setPosition(0);
        shooter.setPower(0);
        intake.setPower(0);
    }


    //drive forward at a given distance, speed and gyro heading
    public void drive(double distance, double speed, double targetHeading, boolean isStopAtEnd) {
        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();
        //math to calculate total counts robot should travel
        inches = distance;
        rotations = distance / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        //Set RunMode of leftMotor1 to STOP_AND_RESET_ENCODER
        //then RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //execute while loop for four seconds, or until leftMotor1.getCurrentPosition()
        //is less than counts
        while (leftMotor1.getCurrentPosition() < counts && (timeTwo - timeOne < 4)) {
            //Set current heading to gyro's integrated Z value
            currentHeading = gyro.getIntegratedZValue();
            //Math to calculate adjustment factor for motor powers
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;
            //Set all drivetrain motor powers
            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);
            //telemetry for leftMotor1's current position
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            //Set timeTwo to this.getRuntime
            timeTwo = this.getRuntime();
        }

        //Safety timeout
        if (timeTwo - timeOne > 4) {
            stopDriving();
            while (this.opModeIsActive()) {
                timeTwo = this.getRuntime();
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
            }
        }
        //if parameter isStopAtEnd is true,
        //execute stopDriving method
        if (isStopAtEnd) {
            stopDriving();
        }

    }

    //drive forward at a given distance, speed, but without considering gyro heading
    public void driveNoGyro(double distance, double speed) {
        //math to calculate total counts robot should travel
        inches = distance;
        rotations = distance / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        //Set RunMode of leftMotor1 to STOP_AND_RESET_ENCODER
        //then RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //execute while loop for three seconds, or until leftMotor1.getCurrentPosition()
        //is less than counts
        while (leftMotor1.getCurrentPosition() < counts && (timeTwo - timeOne < 3)) {
            //set all drivetrain motor powers to parameter speed
            leftMotor1.setPower(speed);
            leftMotor2.setPower(speed);
            rightMotor1.setPower(speed);
            rightMotor2.setPower(speed);
            //telemetry for leftMotor1's current position
            telemetry.addData("Current", leftMotor1.getCurrentPosition());
            telemetry.update();
            //Set timeTwo to this.getRuntime
            timeTwo = this.getRuntime();
        }

        //Safety timeout
        if (timeTwo - timeOne > 3) {
            stopDriving();
            while (this.opModeIsActive()) {
                timeTwo = this.getRuntime();
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
            }
        }

        //Execute stopDriving() method
        stopDriving();
    }

    //Drive at a given speed until both ods sensors see adequate white light
    public void driveToWhiteLine(double speed, double targetHeading) {
        //Set whitesCountLeft and whitesCountRight to 0
        whiteCountLeft = 0;
        whiteCountRight = 0;

        //Set wlsRightlight and wlsLeftlight to false
        wlsRightlight = false;
        wlsLeftlight = false;

        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //Set RunMode of leftMotor1 to RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set all drivetrain motor powers to parameter speed
        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the drivetrain motors running while op mode is
        // active and not enough white light
        // has been detected on both ods sensors
        do {
            //Set current heading to gyro's integrated Z Value
            currentHeading = gyro.getIntegratedZValue();
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            //Telemetry for left and rightods' getRawLightDetected
            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //updating time2 to prevent infinite running of this loop if game conditions are not met
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the left ods boolean to true,
            //increment whiteCounterLeft by 1, and set wlsLeftlight to true
            if (whiteLineSensorLeft.getRawLightDetected() >= WHITE_THRESHOLD) {
                wlsLeftlight = true;
                whiteCountLeft++;
            }

            //If enough white light has been detected, set the right ods boolean to true,
            //increment whiteCounterRight by 1, and set wlsRightlight to true
            if (whiteLineSensorRight.getRawLightDetected() >= WHITE_THRESHOLD) {
                wlsRightlight = true;
                whiteCountRight++;
            }

            //If enough white light has not been detected, set right motor powers
            //according to parameter speed and straightDriveAdjust,
            //to maintain an appropriate heading.  If enough white light has
            //been detected, set the right motor powers to 0
            if (wlsRightlight == false) {
                rightMotor1.setPower(speed - straightDriveAdjust);
                rightMotor2.setPower(speed - straightDriveAdjust);
            } else {
                rightMotor1.setPower(0);
                rightMotor2.setPower(0);
            }

            //If enough white light has not been detected, set left motor powers
            //according to parameter speed and straightDriveAdjust,
            //to maintain an appropriate heading.  If enough white light has
            //been detected, set the left motor powers to 0
            if (wlsLeftlight == false) {
                leftMotor1.setPower(speed + straightDriveAdjust);
                leftMotor2.setPower(speed + straightDriveAdjust);
            } else {
                leftMotor1.setPower(0);
                leftMotor2.setPower(0);
            }
        }

        //execute do loop while either wlsRightlight or wlsLeftlight, and while op mode is active,
        //and the loop has been executing for less than eight seconds
        while ((wlsRightlight == false
                || wlsLeftlight == false)
                && this.opModeIsActive()
                && (timeTwo - timeOne < 8));  //Repeat do loop until both odss have detected enough white light

        //Safety timeout
        if (timeTwo - timeOne > 8) {
            stopDriving();
            while (this.opModeIsActive()) {
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
                timeTwo = this.getRuntime();
            }
        }

        //Execute stopDriving() method
        stopDriving();
    }

    //Drive at a given speed until the left ods sees adequate white light
    public void driveToWhiteLineLeft(double speed, double targetHeading, boolean isStopAtEnd) {
        //Set whitesCount to 0
        whitesCount = 0;

        //Set whiteLineNotDetected to false
        whiteLineNotDetected = true;

        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //Set RunMode of leftMotor1 to RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set all drivetrain motor powers to parameter speed
        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the drivetrain motors running while op mode is
        // active and not enough white light
        // has been detected on the left ods
        do {
            //Set current heading to gyro's integrated Z Value
            currentHeading = gyro.getIntegratedZValue();

            //Calculate straightDriveAdjust using the robot's heading error
            //And straightGyroGain
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            //Increment loopCounter by 1
            loopCounter++;

            //Telemetry for left ods' getRawLightDetected
            telemetry.addData("White Value Left: ", whiteLineSensorLeft.getRawLightDetected());
            telemetry.update();
            //Set timeTwo to this.getRuntime()
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean whiteLineNotDetected to true
            //and increment variable whitesCount by 1
            if (whiteLineSensorLeft.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
                whitesCount++;
            }

            //Set all motor powers using parameter speed and straightDriveAdjust
            //to maintain appropriate heading
            leftMotor1.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);
        }

        //Repeat do loop until left ods has detected enough white light, and while op mode is active
        //whiteCount is under 3, and the loop has been running for less than three seconds
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo - timeOne < 3) && whitesCount < 3);

        //Safety timeout
        if (timeTwo - timeOne > 3) {
            stopDriving();
            while (this.opModeIsActive()) {
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
                timeTwo = this.getRuntime();
            }
        }

        //If parameter isStopAtEnd is true, execute stopDriving() method
        if (isStopAtEnd) {
            stopDriving();
        }
    }

    //Drive at a given speed until the right ods sees adequate white light
    public void driveToWhiteLineRight(double speed, double targetHeading, boolean isStopAtEnd) {
        //Set whitesCount to 0
        whitesCount = 0;

        //Set whiteLineNotDetected to false
        whiteLineNotDetected = true;

        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //Set RunMode of leftMotor1 to RUN_WITHOUT_ENCODER
        leftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set all drivetrain motor powers to parameter speed
        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Keep the drivetrain motors running while op mode is
        // active and not enough white light
        // has been detected on the left ods
        do {
            //Set current heading to gyro's integrated Z Value
            currentHeading = gyro.getIntegratedZValue();

            //Calculate straightDriveAdjust using the robot's heading error
            //And straightGyroGain
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            //Telemetry for right ods' getRawLightDetected
            telemetry.addData("White Value Right: ", whiteLineSensorRight.getRawLightDetected());
            telemetry.update();
            //Set timeTwo to this.getRuntime()
            timeTwo = this.getRuntime();

            //If enough white light has been detected, set the ods boolean whiteLineNotDetected to true
            //and increment variable whitesCount by 1
            if (whiteLineSensorRight.getRawLightDetected() >= WHITE_THRESHOLD) {
                whiteLineNotDetected = false;
                whitesCount++;
            }

            //Set all motor powers using parameter speed and straightDriveAdjust
            //to maintain appropriate heading
            leftMotor1.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);
        }

        //Repeat do loop until right ods has detected enough white light, and while op mode is active
        //whiteCount is under 3, and the loop has been running for less than three seconds
        while (whiteLineNotDetected && this.opModeIsActive() && (timeTwo - timeOne < 3) && whitesCount < 3);  //Repeat do loop until both odss have detected enough white light

        //Safety timeout
        if (timeTwo - timeOne > 3) {
            stopDriving();
            while (this.opModeIsActive()) {
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
                timeTwo = this.getRuntime();
            }
        }

        //If parameter isStopAtEnd is true, execute stopDriving() method
        if (isStopAtEnd) {
            stopDriving();
        }
    }

    //Run method to drive at a given speed until adequate blue light is detected
    //at which time the appropriate beacon pusher extends and retracts to push
    //the beacon
    public void pressBeaconSideBlue(double speed, double targetHeading) {
        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //set all drivetrain motor powers to parameter speed
        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Set beaconNotDetected to true
        beaconNotDetected = true;

        //Set beaconPusherRight to variable BEACON_PUSHER_RIGHT_RETRACT_POSITION
        beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION);

        do {
            //Set current heading to gyro's integrated Z Value
            currentHeading = gyro.getIntegratedZValue();

            //Calculate straightDriveAdjust using the robot's heading error
            //And straightGyroGain
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            //Telemetry for blue value of colorSensorRight
            telemetry.addData("Blue Value: ", colorSensorRight.blue());
            telemetry.update();

            //If enough blue light has been detected, and there is a small amount of red light
            // set the beacon boolean to true
            if (colorSensorRight.blue() >= 2 && colorSensorRight.red() < 2) {
                beaconNotDetected = false;
            }

            //Set all motor powers using parameter speed and straightDriveAdjust
            //to maintain appropriate heading
            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);

            //Set timeTwo to this.getRuntime()
            timeTwo = this.getRuntime();

            //Repeat do loop while beaconNotDetected is true, op mode is active
            //and loop has been running for less than 12 secons
        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo - timeOne < 12));

        //Execute stopDriving() method
        stopDriving();

        //Safety timeout
        if (timeTwo - timeOne > 12) {
            stopDriving();
            while (this.opModeIsActive()) {
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
                timeTwo = this.getRuntime();
            }
        }

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Extend right beacon pusher for three seconds
        while (timeTwo - timeOne < 3) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Retract right beacon pusher for one second
        while (timeTwo - timeOne < 1) {
            beaconPusherRight.setPosition(BEACON_PUSHER_RIGHT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }
    }


    //Run method to drive at a given speed until adequate red light is detected
    //at which time the appropriate beacon pusher extends and retracts to push
    //the beacon
    public void pressBeaconSideRed(double speed, double targetHeading) {
        //Set initial heading to gyro's integrated Z value
        initialHeading = gyro.getIntegratedZValue();

        //set all drivetrain motor powers to parameter speed
        leftMotor1.setPower(speed);
        rightMotor1.setPower(speed);
        leftMotor2.setPower(speed);
        rightMotor2.setPower(speed);

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Set beaconNotDetected to true
        beaconNotDetected = true;

        //Set beaconPusherLeft to variable BEACON_PUSHER_LEFT_RETRACT_POSITION
        beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION);

        do {
            //Set current heading to gyro's integrated Z Value
            currentHeading = gyro.getIntegratedZValue();

            //Calculate straightDriveAdjust using the robot's heading error
            //And straightGyroGain
            straightDriveAdjust = (currentHeading - targetHeading) * straightGyroGain;

            //Telemetry for red value of colorSensorLeft
            telemetry.addData("Red Value: ", colorSensorLeft.red());
            telemetry.update();

            //If enough red light has been detected, and there is a small amount of blue light
            // set the beacon boolean to true
            if (colorSensorLeft.red() >= 2 && colorSensorLeft.blue() < 2) {
                beaconNotDetected = false;
            }

            //Set all motor powers using parameter speed and straightDriveAdjust
            //to maintain appropriate heading
            leftMotor1.setPower(speed + straightDriveAdjust);
            leftMotor2.setPower(speed + straightDriveAdjust);
            rightMotor1.setPower(speed - straightDriveAdjust);
            rightMotor2.setPower(speed - straightDriveAdjust);

            //Set timeTwo to this.getRuntime()
            timeTwo = this.getRuntime();

            //Repeat do loop while beaconNotDetected is true, op mode is active
            //and loop has been running for less than 12 secons
        } while (beaconNotDetected && this.opModeIsActive() && (timeTwo - timeOne < 12));

        //Execute stopDriving() method
        stopDriving();

        //Safety timeout
        if (timeTwo - timeOne > 12) {
            stopDriving();
            while (this.opModeIsActive()) {
                //Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
                timeTwo = this.getRuntime();
            }
        }

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Extend left beacon pusher for three seconds
        while (timeTwo - timeOne < 3) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_EXTEND_POSITION);
            timeTwo = this.getRuntime();
        }

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //Retract left beacon pusher for one second
        while (timeTwo - timeOne < 1) {
            beaconPusherLeft.setPosition(BEACON_PUSHER_LEFT_RETRACT_POSITION);
            timeTwo = this.getRuntime();
        }

    }

}
