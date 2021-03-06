/**
 * Created by stevecox on 11/4/17.
 * Code for Harvey's autonomous
 */
// package declaration

package org.firstinspires.ftc.teamcode;

// import statements
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



public abstract class FunctionsForAuto extends LinearOpMode {

    /******************* MISC *******************/

    int loopCounter = 0; // Variable to count how many times a given loop has been entered

    // time variables set to current run time throughout the code, typically set to this.getRunTime()
    double timeOne;
    double timeTwo;
    double timeRunningLoop;

    String allianceColor;
    String robotStartingPosition;


    /******************* V U F O R I A *******************/

    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark;
    String vuMarkChecker = new String();
    VuforiaTrackable relicTemplate;
    VuforiaTrackables relicTrackables;


    /******************* S E N S O R S *******************/

    //TouchSensor touchSensorTop; //  declare touch sensors for grabbers
    //TouchSensor touchSensorBottom;

    ColorSensor sensorColor; // Right color feeler for balls autonomous

    // Touch Sensor variables
    //boolean touchTopPress = false;
    //boolean touchBottomPress = false;

    // The IMU sensor object
    //BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;






    /******************* D R I V I N G *******************/

    // DRIVE MOTORS
    DcMotor RightMotor;     // right drive motor front
    DcMotor LeftMotor;      // left drive motor front
    DcMotor FrontMotor;       // right drive motor back
    DcMotor BackMotor;    // left drive motor back

    // driving powers
    double rightPower;
    double leftPower;
    double topPower;
    double bottomPower;

    // encoder variables to adequately sense the lines
    final static double ENCODER_CPR = 1120;    // encoder counts per rotation (CPR)
    final static double GEAR_RATIO = 0.727;     // Gear ratio used in Harvey in 22/16, so in code we multiply by 16/22
    final static double WHEEL_DIAMETER = 4; // wheel diameter in inches

    // Driving variables
    double inches;  // Desired number of inches to drive
    double rotations;       // Wheel rotations necessary to drive the above amount of inches
    double counts;// Encoder counts necessary to drive the above amount of inches/rotations






    /******************* G R A B B E R   S E R V O S *******************/

    // Servos
    //Servo horizontalTop; // Servo that rotate's the grabber horizontally
    Servo top; // Sevo that opens and closes the two grabbers
    Servo topSuckRight; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo topSuckLeft; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo
    //Servo horizontalBottom; // Servo that rotate's the grabber horizontally
    //Servo open; // Sevo that opens and closes the two grabbers

    // grabber
    Servo feelerRaise;
    Servo feelerSwipe;

    double feelerRaiseUpPosition = 1;
    double feelerRaiseDownPosition = .25;
    double feelerSwipeNeutralPosition = .5;
    double feelerSwipeCWPosition = .25; //tentative, cw
    double feelerSwipeCCWPosition = .75; //tentative, ccw



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
        RightMotor = hardwareMap.dcMotor.get("rightMotor");
        LeftMotor = hardwareMap.dcMotor.get("leftMotor");
        FrontMotor = hardwareMap.dcMotor.get("topMotor");
        BackMotor = hardwareMap.dcMotor.get("bottomMotor");

        // Motor directions: set forward/reverse
        RightMotor.setDirection(REVERSE);
        LeftMotor.setDirection(FORWARD);
        FrontMotor.setDirection(REVERSE);
        BackMotor.setDirection(FORWARD);





        /******************* S E N S O R S *******************/

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColor.enableLed(true);


//        BNO055IMU.Parameters IMUparameters = new BNO055IMU.Parameters();
//        IMUparameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        IMUparameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        IMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        IMUparameters.loggingEnabled      = true;
//        IMUparameters.loggingTag          = "IMU";
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(IMUparameters);


        // Initialize encoder variables to 0
        inches = 0;
        rotations = 0;
        counts = 0;





        /******************* G R A B B E R   S E R V O S *******************/

        // harware map configurations
        //horizontalTop = hardwareMap.servo.get("horizontalTop");
        top = hardwareMap.servo.get("openCloseTop");
        topSuckRight = hardwareMap.servo.get("rightGrabberTop");
        topSuckLeft = hardwareMap.servo.get("leftGrabberTop");

        //horizontalBottom = hardwareMap.servo.get("horizontalBottom");
        //open = hardwareMap.servo.get("open");

        feelerRaise = hardwareMap.servo.get("feeler raise");
        feelerSwipe = hardwareMap.servo.get("feeler swipe");


        // Set servo direction orientations forward or reverse
        //horizontalTop.setDirection(Servo.Direction.FORWARD);
        top.setDirection(Servo.Direction.FORWARD);
        topSuckRight.setDirection(Servo.Direction.REVERSE);
        topSuckLeft.setDirection(Servo.Direction.FORWARD);


        //horizontalBottom.setDirection(Servo.Direction.FORWARD);
       // open.setDirection(Servo.Direction.FORWARD);

        feelerRaise.setDirection(Servo.Direction.FORWARD);
        feelerSwipe.setDirection(Servo.Direction.REVERSE);

        // Initial positions for servos
        //horizontalTop.setPosition(.486);
        top.setPosition(.5);
        topSuckRight.setPosition(.5);
        topSuckLeft.setPosition(.5);

        //horizontalBottom.setPosition(.486);
        //open.setPosition(.5);


        feelerRaise.setPosition(feelerRaiseUpPosition);
        feelerSwipe.setPosition(feelerSwipeNeutralPosition);

    }

    public void encoders ()
    {
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of FrontMotor1 to STOP_AND_RESET_ENCODER
        RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        while (this.opModeIsActive())
        {
            telemetry.addData("counts", RightMotor.getCurrentPosition());
            telemetry.update();
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



    // drive function for any direction
    public void drive( String direction, double distance, double power, double time ) {

        // math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) { // check should be tob and bottom motors instead
            FrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of LeftMotor1 to STOP_AND_RESET_ENCODER
            FrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//check should do right too?
        }
        else {
            RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of FrontMotor1 to STOP_AND_RESET_ENCODER
            RightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        }

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right")) {
            while ( Math.abs(FrontMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) { //check here
                if ( direction.equalsIgnoreCase("left") ) {
                    // Set motor powers based on paramater power
                    FrontMotor.setPower( -power );
                    BackMotor.setPower( -power );
                }
                else if ( direction.equalsIgnoreCase("right") ){
                    // Set motor powers based on paramater power
                    FrontMotor.setPower( power );
                    BackMotor.setPower( power );
                }


                // Telemetry for encoder position
                telemetry.addData("Current", FrontMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            FrontMotor.setPower( 0 );
            BackMotor.setPower( 0 );
        }
        else {
            while ( Math.abs(RightMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) { //check here
                if ( direction.equalsIgnoreCase("backwards") ) {
                    // Set motor powers based on paramater power
                    LeftMotor.setPower( power );
                    RightMotor.setPower( power );
                }
                else if ( direction.equalsIgnoreCase("forwards") ) {
                    // Set motor powers based on paramater power
                    LeftMotor.setPower( -power );
                    RightMotor.setPower( -power );
                }

                // Telemetry for encoder position
                telemetry.addData("Current", RightMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            LeftMotor.setPower(0);
            RightMotor.setPower(0);

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

    // drive function for any direction with time
    public void driveForTime (String direction, double power, double time ) {

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) { // check should be tob and bottom motors instead
            LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of LeftMotor1 to STOP_AND_RESET_ENCODER
            LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//check should do right too?
        }
        else {
            FrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of FrontMotor1 to STOP_AND_RESET_ENCODER
            FrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //check should to bottom too?
        }

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) {
            while ( (timeTwo - timeOne < time) ) { //check here
                if ( direction.equalsIgnoreCase("left") ) {
                    // Set motor powers based on paramater power
                    FrontMotor.setPower( -power );
                    BackMotor.setPower( -power );
                }
                else if ( direction.equalsIgnoreCase("right") ){
                    // Set motor powers based on paramater power
                    FrontMotor.setPower( power );
                    BackMotor.setPower( power );
                }

                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            FrontMotor.setPower( 0 );
            BackMotor.setPower( 0 );
        }
        else if (direction.equalsIgnoreCase( "forwards") || direction.equalsIgnoreCase("backwards")){
            while ( (timeTwo - timeOne < time) ) { //check here
                if ( direction.equalsIgnoreCase("backwards") ) {
                    // Set motor powers based on paramater power
                    LeftMotor.setPower( -power );
                    RightMotor.setPower( -power );
                }
                else if ( direction.equalsIgnoreCase("forwards") ) {
                    // Set motor powers based on paramater power
                    LeftMotor.setPower( power );
                    RightMotor.setPower( power );
                }

                // Telemetry for encoder position
                telemetry.addData("Current", LeftMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            LeftMotor.setPower( 0 );
            RightMotor.setPower(0);

        }

        // Execute stopDriving method
        stopDriving();

    }

    public void test(double power) {
        FrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of FrontMotor1 to STOP_AND_RESET_ENCODER
        FrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // check to change to bottom

        FrontMotor.setPower( power );
        BackMotor.setPower( power );

        pause(1);
    }

    // Sets all drive train motors to 0 power
    public void stopDriving() {
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
        FrontMotor.setPower(0);
        BackMotor.setPower(0);
    }

    public void pause( double time ) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < time) {
            timeTwo = this.getRuntime();
        }
    }

    // Execute a robot spin using both sides of the drive train and the gyro
    public void spinMove( String direction, double distance, double power, double time ) {

        // math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of LeftMotor1 to STOP_AND_RESET_ENCODER
        LeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while ( Math.abs(LeftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) )
        {
            if ( direction.equalsIgnoreCase("clockwise") || direction.equalsIgnoreCase("cw") ) {
                leftPower = power;
                rightPower = -power;
                topPower = 0;
                bottomPower = 0;
            }
            else if ( direction.equalsIgnoreCase("counterclockwise") || direction.equalsIgnoreCase("ccw") ) {
                leftPower = -power;
                rightPower = power;
                topPower = 0;
                bottomPower = 0;
            }

            // Telemetry for encoder position
            telemetry.addData("Current", LeftMotor.getCurrentPosition());
            telemetry.update();
            // Set timeTwo to this.getRuntime ()
            timeTwo = this.getRuntime();
        }

        // Safety timeout based on if the loop above executed in under 4 seconds
        // If it did not, do not execute the rest of the program
        if (timeTwo - timeOne > time) {
            while (this.opModeIsActive()) {
                stopDriving();
                timeTwo = this.getRuntime();
                // Telemetry alerting drive team of safety timeout
                telemetry.addLine("Timed out");
                telemetry.update();
            }
        }
        // Execute stopDriving method
        stopDriving();
    }

    public void dropFeelerMoveBallOnlyNewRobot (){

        feelerRaise.setPosition(feelerRaiseDownPosition);

        pause(1);


        if (allianceColor.equalsIgnoreCase("red") && sensorColor.blue() >=  sensorColor.red()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCWPosition);
            pause(.2);
            feelerRaise.setPosition(feelerRaiseUpPosition);
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("red") && sensorColor.red() >= sensorColor.blue()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCCWPosition);
            pause(.2);
            feelerRaise.setPosition(feelerRaiseUpPosition);
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.blue() >= sensorColor.red()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCCWPosition);
            pause(.2);
            feelerRaise.setPosition(feelerRaiseUpPosition);
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.red() >= sensorColor.blue()) {
            telemetry.addData( "Value of RED: ", sensorColor.red() );
            telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
            telemetry.update();
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeCWPosition);
            pause(.2);
            feelerRaise.setPosition(feelerRaiseUpPosition);
            pause(.2);
            feelerSwipe.setPosition(feelerSwipeNeutralPosition);
        }

        stopDriving();

    }
}
