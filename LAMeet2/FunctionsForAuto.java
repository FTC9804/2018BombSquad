/**
 * Created by stevecox on 11/4/17.
 * Code for Harvey's autonomous
 */
// package declaration

package org.firstinspires.ftc.teamcode;

// import statements

// G R A V I T Y

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Locale;


public abstract class FunctionsForAuto extends LinearOpMode {

    /******************* MISC *******************/

    // time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne;
    double timeTwo;

    boolean hasStrafedLoop = false;

    int loopCounter = 0;

    String allianceColor; //The alliance color of a given match
    String robotStartingPosition; //The starting position of the robot, either relicSide
    //or triangle Side, representing different places on the field

    /******************* V U F O R I A *******************/

    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark;
    String vuMarkChecker = new String();
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;

    /******************* T O U C H  S E R V O *******************/

    Servo touchLiftFeelerRaise;
    double touchLiftRetractPosition = .8;
    double touchLiftExtendPosition = .2;

    /******************* S E N S O R S *******************/

    DigitalChannel touchSensorFront; //  declare touch sensors for grabbers
    DigitalChannel touchSensorLeft;
    DigitalChannel touchSensorRight;

    DigitalChannel limitTop; //Touch sensor that tells us if we reach the top with the Pan
    DigitalChannel limitBottom; //Touch sensor that tells us if we reach the bottom with the Pan

    ColorSensor sensorColor; // Right color feeler for balls autonomous

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation lastAngles = new Orientation();
    //Acceleration gravity = new Acceleration();

    /******************* D R I V I N G *******************/

    // DRIVE MOTORS
    DcMotor rightMotor;     // right drive motor front
    DcMotor leftMotor;      // left drive motor front
    DcMotor backLeftMotor;       // right drive motor back
    DcMotor backRightMotor;    // left drive motor back

    // driving powers
    double rightPower;
    double leftPower;
    double topPower;
    double bottomPower;

    // encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 537.6;    // encoder counts per rotation (CPR)
    final static double GEAR_RATIO = .727;     // Gear ratio used in Harvey in 22/16, so in code we multiply by 16/22
    final static double WHEEL_DIAMETER = 4; // wheel diameter in inches

    //Gyro/imu variables
    double initialHeading;
    double currentHeading;
    double headingError;
    double gyroGain = .0099;
    double straightGyroGain = .025;
    double straightGyroAdjust;

    double globalAngle;
    double correction;


    // Driving variables
    double inches;  // Desired number of inches to drive
    double rotations;       // Wheel rotations necessary to drive the above amount of inches
    double counts;// Encoder counts necessary to drive the above amount of inches/rotations

    /******************* P A N *******************/

    DcMotor rightIntakeMotor; //Dc motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Dc motor that controls the left intake/left wheel of the intake
    DcMotor panLifterMotor;
    Servo leftPanSpin;
    Servo rightPanSpin;
    Servo pan45;

    double intakePower = .5;
    double outtakePower = -.5;

    double panLiftingPower;

    double panSpinUpFirst = .7;
    double panSpinUpSecond = .8;
    double panSpinDown = .3;

    double pan45Still = .5;
    double pan45Redirect = .75;

    boolean liftHasGoneUp = false;

    /******************* F E E L E R S   S E R V O S *******************/

    // grabber
    Servo feelerSwipe;

    double feelerRaiseUpPosition = 1;
    double feelerRaiseDownPosition = .25;
    double feelerSwipeNeutralPosition = .5;
    double feelerSwipeCWPosition = .25; //tentative, cw
    double feelerSwipeCCWPosition = .75; //tentative, ccw

    /******************* B L O C K    D I S T A N C E *******************/

    DistanceSensor sensorA;
    DistanceSensor sensorB;
    DistanceSensor sensorC;

    boolean threeBlocks = false;

    /******************* F U N C T I O N S   F O R   A U T O *******************/

    // Configures all hardware devices, and sets them to their initial values, if necessary
    public void configure( String initialAllianceColor, String initialRobotStartingPosition ) {

        /******************* A L L I A N C E *******************/
        allianceColor = initialAllianceColor; // Options: "red" or "blue"
        robotStartingPosition = initialRobotStartingPosition; // Options: "relicSide" or "triangleSide"

        /******************* V U F O R I A *******************/
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey =  "AZfTpOj/////AAAAGYCE1z7z6E5whPRKfYeRJHEN/u/+LZ7AMmBU0bBa" +
                "/7u6aTruUWfYeLur6nSFdKP0w9JPmK1gstNxVHqiaZN6iuZGxPcbnDnm" +
                "NJdoLIMtZheeNWphUMjHKoTUgsmcloZe67TG2V9duc+8jxxCLFzH5rlq" +
                "PPdcgvvtIO0orpxVcpENBunY2GChhVgP6V5T9Iby7MyM9tN+y7Egm7Xy" +
                "Iz/Tzpmlj19b3FUCW4WUDjTNQ4JoKZeB1jkhPxKGFRECoPw02jJXtQSK" +
                "zNfzmhtugA7PTOZNehc61UjOXEexTO9TRy7ZfMtW8OggcYssvIabyJ8b" +
                "DK4ePLCUP+Q4PMf7kL9lM6yDuxxKF0oqLgRglX9Axqrf";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        /******************* D R I V I N G *******************/

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Motor directions: set forward/reverse
        rightMotor.setDirection(REVERSE);
        leftMotor.setDirection(FORWARD);
        backLeftMotor.setDirection(REVERSE);
        backRightMotor.setDirection(REVERSE);

        /******************* I M U *******************/

        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();

        IMUparameters.mode                = BNO055IMU.SensorMode.IMU;
        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
        IMUparameters.loggingEnabled      = true; //F A L S E???
        IMUparameters.loggingTag          = "IMU";
        IMUparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMUparameters);
        /******************* P A N *******************/

        leftPanSpin = hardwareMap.servo.get("leftPanSpin"); //leftPanSpin configuration
        rightPanSpin = hardwareMap.servo.get("rightPanSpin"); //rightPanSpin configuration

        limitTop = hardwareMap.get(DigitalChannel.class, "limitTop"); //Top touchSensor configuration
        limitBottom = hardwareMap.get(DigitalChannel.class, "limitBottom");

        pan45 = hardwareMap.servo.get("panKicker");

        panLifterMotor = hardwareMap.dcMotor.get("elevator"); //panLifterMotor configuration

        pan45.setDirection(Servo.Direction.FORWARD);

        rightIntakeMotor = hardwareMap.dcMotor.get("rightIntake"); //rightIntakeMotor configuration
        leftIntakeMotor = hardwareMap.dcMotor.get("leftIntake"); //leftIntakeMotor configuration
        panLifterMotor = hardwareMap.dcMotor.get("elevator"); //panLifterMotor configuration

        rightIntakeMotor.setDirection(FORWARD); //Set rightIntakeMotor to FORWARD direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(FORWARD); //Set panLifterMotor to FORWARD direction

        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction

        /******************* T O U C H  S E R V O *******************/

        touchLiftFeelerRaise = hardwareMap.servo.get("touchLiftFeelerRaise");
        touchLiftFeelerRaise.setDirection(Servo.Direction.FORWARD);


        /******************* S E N S O R S *******************/

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColor.enableLed(true);

        sensorA = hardwareMap.get(DistanceSensor.class, "sensor A");
        sensorB = hardwareMap.get(DistanceSensor.class, "sensor B");
        sensorC = hardwareMap.get(DistanceSensor.class, "sensor C");

        touchSensorFront = hardwareMap.get(DigitalChannel.class, "touchFront");
        touchSensorLeft = hardwareMap.get(DigitalChannel.class, "touchLeft");
        touchSensorRight = hardwareMap.get(DigitalChannel.class, "touchRight");

        /******************* B A L L   S E R V O S *******************/

        feelerSwipe = hardwareMap.servo.get("feeler swipe");

        feelerSwipe.setDirection(Servo.Direction.REVERSE);

        feelerSwipe.setPosition(feelerSwipeNeutralPosition);

        touchLiftFeelerRaise.setPosition(.5);


        /******************* P A N    S P I N *******************/
        leftPanSpin.setPosition(.05);
        rightPanSpin.setPosition(.05);

        pan45.setPosition(pan45Still);

    }

    public void calibrateGyro ()
    {
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
    }

    public void introduceAngle ()
    {
        getAngleSimple();
    }

    public void touchServoExtend ()
    {
        touchLiftFeelerRaise.setPosition(touchLiftExtendPosition);
    }

    public void touchServoRetract ()
    {
        touchLiftFeelerRaise.setPosition(touchLiftRetractPosition);
    }

    public void startAcceleration ()
    {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    public void encoders ()
    {
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        while (this.opModeIsActive())
        {
            telemetry.addData("counts", rightMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    public void imu ()
    {
        // Set up our telemetry dashboard
        composeTelemetry();
    }

    public void scoreBlock (boolean bottom)
    {
        if (bottom)
        {
            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();
            while (timeTwo-timeOne < .5)
            {
                timeTwo = this.getRuntime();
                panLifterMotor.setPower(.4);
            }
            panLifterMotor.setPower(0);
            leftPanSpin.setPosition(panSpinUpFirst);
            rightPanSpin.setPosition(panSpinUpFirst);
            pause(.3);
            pan45.setPosition(pan45Redirect);
            pause(.3);
            leftPanSpin.setPosition(panSpinUpSecond);
            rightPanSpin.setPosition(panSpinUpSecond);
            pause(1);
            leftPanSpin.setPosition(panSpinDown);
            rightPanSpin.setPosition(panSpinDown);
        }
        else
        {
            if (!liftHasGoneUp) {
                while (limitTop.getState()) {
                    panLifterMotor.setPower(.4);
                }
                panLifterMotor.setPower(0);
                liftHasGoneUp = true;
            }

            leftPanSpin.setPosition(panSpinUpFirst);
            rightPanSpin.setPosition(panSpinUpFirst);
            pause(.3);
            pan45.setPosition(pan45Redirect);
            pause(.3);
            leftPanSpin.setPosition(panSpinUpSecond);
            rightPanSpin.setPosition(panSpinUpSecond);
            pause(1);
            leftPanSpin.setPosition(panSpinDown);
            rightPanSpin.setPosition(panSpinDown);
        }

    }

    public void getBlocks ()
    {
        while (sensorA.getDistance(DistanceUnit.CM) > 14)
        {
            leftIntakeMotor.setPower(intakePower);
            rightIntakeMotor.setPower(intakePower);
            leftMotor.setPower(.3);
            rightMotor.setPower(.3);

        }

        while (sensorA.getDistance(DistanceUnit.CM) < 14 && !threeBlocks) {
            leftIntakeMotor.setPower(intakePower);
            rightIntakeMotor.setPower(intakePower);
            leftMotor.setPower(-.3);
            rightMotor.setPower(-.3);
            if (sensorA.getDistance(DistanceUnit.CM) < 14 && sensorB.getDistance(DistanceUnit.CM) < 14 && sensorC.getDistance(DistanceUnit.CM) < 14) {
                threeBlocks = true;
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        if (threeBlocks) {
            while (sensorA.getDistance(DistanceUnit.CM) < 14) {
                leftIntakeMotor.setPower(outtakePower);
                rightIntakeMotor.setPower(outtakePower);
            }

            timeOne = this.getRuntime();
            timeTwo = this.getRuntime();

            while (timeTwo - timeOne < 1) {
                leftIntakeMotor.setPower(outtakePower);
                rightIntakeMotor.setPower(outtakePower);
                timeTwo=this.getRuntime();
            }
        }

    }

    public String detectVuMark( int timeToCheck ) {

        relicTrackables.activate();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < timeToCheck) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vumarkToString().equalsIgnoreCase("LEFT")) {
                vuMarkChecker = "left";
            } else if (vumarkToString().equalsIgnoreCase("RIGHT")) {
                vuMarkChecker = "right";
            } else if (vumarkToString().equalsIgnoreCase("CENTER")) {
                vuMarkChecker = "center";
            } else if (vumarkToString().equals("UNKOWN")){
                vuMarkChecker = "unknown as answer";
            }
            else {
                vuMarkChecker = "novalue";
            }

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    /* Found an instance of the template. In the actual game, you will probably
                     * loop until this condition occurs, then move on to act accordingly depending
                     * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

            } else {
                telemetry.addData("VuMark", "not visible");
            }

            timeTwo = this.getRuntime();
            telemetry.addData("Time: ", timeTwo - timeOne);

            telemetry.addData("Vu mark detector: ", vuMarkChecker);
            telemetry.update();
        }
        return vuMarkChecker;
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    public String vumarkToString()
    {
        String output = new String();
        output = "" + vuMark;
        return output;
    }

    public void testIMUForDrive ()
    {
        telemetry.update();

        initialHeading = Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));

        while (this.opModeIsActive())
        {
            telemetry.addData("used value", Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle)));
            telemetry.update();
        }
    }

    // drive function for any direction
    public void drive(String direction, double distance, double speed, double time, double targetHeading) {

        telemetry.update();

        initialHeading = Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));

        // math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) { // check should be tob and bottom motors instead
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//check should do right too?
        }
        else {
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        }

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right")) {


            while ( Math.abs(backLeftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time)) {//check here
                imu();

                currentHeading = Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));

                straightGyroAdjust = (currentHeading - targetHeading) * straightGyroGain;

                straightGyroAdjust = Range.clip(straightGyroAdjust, -.5, .5);

                if ( direction.equalsIgnoreCase("left") ) {
                    // Set motor powers based on paramater power
                    backLeftMotor.setPower(-speed + straightGyroAdjust );
                    backRightMotor.setPower(-speed + straightGyroAdjust );
                }
                else if ( direction.equalsIgnoreCase("right") ){
                    // Set motor powers based on paramater power
                    backLeftMotor.setPower(speed + straightGyroAdjust);
                    backRightMotor.setPower(speed + straightGyroAdjust);
                }


                // Telemetry for encoder position
                telemetry.addData("Current", backLeftMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        else {
            while ( Math.abs(rightMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) { //check here

                imu();

                currentHeading = Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));

                straightGyroAdjust = (currentHeading - targetHeading) * straightGyroGain;

                straightGyroAdjust = Range.clip(straightGyroAdjust, -.5, .5);

                if ( direction.equalsIgnoreCase("backwards") ) {
                    // Set motor powers based on paramater power
                    leftMotor.setPower( speed + straightGyroAdjust );
                    rightMotor.setPower( speed - straightGyroAdjust );
                }
                else if ( direction.equalsIgnoreCase("forwards") ) {
                    // Set motor powers based on paramater power
                    leftMotor.setPower( -speed + straightGyroAdjust );
                    rightMotor.setPower( -speed - straightGyroAdjust );
                }

                // Telemetry for encoder position
                telemetry.addData("Current", rightMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }

        // Safety timeout based on if the loop above executed in under 4 seconds
//        // If it did not, do not execute the rest of the program
//        if (timeTwo - timeOne > time) { //check
//            while (this.opModeIsActive()) {
//                stopDriving();
//                timeTwo = this.getRuntime();
//                // Telemetry alerting drive team of safety timeout
//                telemetry.addLine("Timed out");
//                telemetry.update();
//            }
//        }
        // Execute stopDriving method
        //stopDriving();

    }

    public void test(double power) {
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // check to change to bottom

        backLeftMotor.setPower( power );
        backRightMotor.setPower( power );

        pause(1);
    }

    // Sets all drive train motors to 0 power
    public void stopDriving() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void pause( double time ) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < time) {
            timeTwo = this.getRuntime();
        }
    }

    public void spin180( double power, double timeToRun)
    {
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = 1440; // per Wilder's testing 12-1-17 at midnight

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while ( Math.abs(leftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < timeToRun) )
        {

            leftPower = power;
            rightPower = -power;

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);


            // Telemetry for encoder position
            telemetry.addData("Current", Math.abs(leftMotor.getCurrentPosition()));
            telemetry.addData("To completion", (counts - Math.abs(leftMotor.getCurrentPosition())));
            telemetry.update();

            timeTwo = this.getRuntime();
        }

        stopDriving();
    }

    public void dropFeelerMoveBallOnlyNewRobot (){

        touchLiftFeelerRaise.setPosition(.5);

        pause(1);

        timeOne = this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne < .5)
        {
            timeTwo=this.getRuntime();
            touchLiftFeelerRaise.setPosition(1);
        }

        touchLiftFeelerRaise.setPosition(.5);

        feelerSwipe.setPosition(.5);

        while (touchSensorFront.getState())
        {
            touchLiftFeelerRaise.setPosition(1);
        }

        touchLiftFeelerRaise.setPosition(.5);


        if (allianceColor.equalsIgnoreCase("red") && sensorColor.blue() >=  sensorColor.red()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCWPosition);
            pause(.2);
            touchLiftFeelerRaise.setPosition(feelerRaiseUpPosition);
            pause(1);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("red") && sensorColor.red() >= sensorColor.blue()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCCWPosition);
            pause(.2);
            touchLiftFeelerRaise.setPosition(feelerRaiseUpPosition);
            pause(1);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.blue() >= sensorColor.red()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCCWPosition);
            pause(.2);
            touchLiftFeelerRaise.setPosition(feelerRaiseUpPosition);
            pause(1);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.red() >= sensorColor.blue()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCWPosition);
            pause(.2);
            touchLiftFeelerRaise.setPosition(feelerRaiseUpPosition);
            pause(1);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }

        stopDriving();

    }

    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            lastAngles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(lastAngles.angleUnit, lastAngles.thirdAngle);
                    }
                });

//        telemetry.addLine()
//            .addData("grvty", new Func<String>() {
//                @Override public String value() {
//                    return gravity.toString();
//                    }
//                })
//            .addData("mag", new Func<String>() {
//                @Override public String value() {
//                    return String.format(Locale.getDefault(), "%.3f",
//                            Math.sqrt(gravity.xAccel*gravity.xAccel
//                                    + gravity.yAccel*gravity.yAccel
//                                    + gravity.zAccel*gravity.zAccel));
//                    }
//                });
    }

    public String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    public void driveToTouch (String direction, double power, double time, double targetHeading, String touchDirection) {

        if (direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right")) { // check should be tob and bottom motors instead
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//check should do right too?
        } else {
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        }

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if (direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right")) {

            if (touchDirection.equalsIgnoreCase("front"))
                while (touchSensorFront.getState()) {//check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();
                }

            if (touchDirection.equalsIgnoreCase("left"))
                while (touchSensorLeft.getState()) {//check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();
                }

            if (touchDirection.equalsIgnoreCase("right"))
                while (touchSensorRight.getState()) {//check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();

                }

            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }

        else {

            if (touchDirection.equalsIgnoreCase("front")) {
                while (touchSensorFront.getState()) { //check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();

                }
            }

            if (touchDirection.equalsIgnoreCase("left")) {
                while (touchSensorLeft.getState()) { //check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();
                }
            }

            if (touchDirection.equalsIgnoreCase("right")) {
                while (touchSensorRight.getState()) { //check here

                    correction = this.getAngleSimple() * .082;

                    backLeftMotor.setPower(power);
                    backRightMotor.setPower(power);

                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);

                    timeTwo = this.getRuntime();
                }
            }

            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }

    public void strafeNewIMU (double distance, double time, double power, boolean forwards) {
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
//
        //Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();



        if (power > 0) {

            while (Math.abs(backLeftMotor.getCurrentPosition()) < counts && timeTwo - timeOne < time) {
                // Use gyro to drive in a straight line.
                correction = (this.getAngleSimple() + 6) * .047;

                if (!hasStrafedLoop) {
                    pause(.5);
                    hasStrafedLoop = true;
                }

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Angle", this.getAngleSimple());
                telemetry.addData("inches", inches);
                telemetry.addData("power", power);
                telemetry.addData("pos", backLeftMotor.getCurrentPosition());
                telemetry.addData("counts", counts);
                telemetry.update();

                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);

                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);
                } else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                timeTwo = this.getRuntime();

            }

        }

        else {

            while (Math.abs(backLeftMotor.getCurrentPosition()) < counts && timeTwo - timeOne < time) {

                // Use gyro to drive in a straight line.
                correction = (this.getAngleSimple() - 6) * .047;

                if (!hasStrafedLoop) {
                    pause(.5);
                    hasStrafedLoop = true;
                }

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("Angle", this.getAngleSimple());
                telemetry.addData("inches", inches);
                telemetry.addData("power", power);
                telemetry.addData("pos", backLeftMotor.getCurrentPosition());
                telemetry.addData("counts", counts);
                telemetry.update();

                backLeftMotor.setPower(power);
                backRightMotor.setPower(power);

                if (Math.abs(correction) > .08) {
                    leftMotor.setPower(correction);
                    rightMotor.setPower(-correction);
                } else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                timeTwo = this.getRuntime();


            }

            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            leftMotor.setPower(0);
            rightMotor.setPower(0);

        }
    }

    public void driveNewIMU (double distance, double time, double power, boolean forwards)
    {

        //math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of frontMotor1 to STOP_AND_RESET_ENCODER
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
//
        //Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //while (Math.abs(rightMotor.getCurrentPosition())<counts && timeTwo-timeOne<time) {

        while (Math.abs(rightMotor.getCurrentPosition())<counts && timeTwo-timeOne<time) {
            // Use gyro to drive in a straight line.
            correction = this.getAngleSimple() * .022;

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("Angle", this.getAngleSimple());
            telemetry.addData("inches", inches);
            telemetry.addData("power", power);
            telemetry.update();

            loopCounter++;

            leftMotor.setPower(power + correction);
            rightMotor.setPower(power - correction);

            timeTwo=this.getRuntime();

            if (forwards) {
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 700) && loopCounter < 100) {
                    power -= loopCounter * .01; //adjust here
                    power = Range.clip(power, .27, 1);
                }
            }
            else
            {
                if (Math.abs(rightMotor.getCurrentPosition()) > Math.abs(counts - 700) && loopCounter < 100) {
                    power += loopCounter * .01; //adjust here
                    power = Range.clip(power, -1, -.27);
                }
            }

        }

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            // stop.
            leftMotor.setPower(0);
            rightMotor.setPower(0);

    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
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

    private double getAngleSimple ()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return Double.parseDouble(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
}

