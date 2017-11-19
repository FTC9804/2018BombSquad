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
import com.qualcomm.hardware.bosch.BNO055IMU;
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




    /******************* S E N S O R S *******************/

    //TouchSensor touchSensorTop; //  declare touch sensors for grabbers
    //TouchSensor touchSensorBottom;

    ColorSensor sensorColor; // Right color feeler for balls autonomous

    // Touch Sensor variables
    //boolean touchTopPress = false;
    //boolean touchBottomPress = false;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;





    /******************* D R I V I N G *******************/

    // DRIVE MOTORS
    DcMotor rightMotor;     // right drive motor front
    DcMotor leftMotor;      // left drive motor front
    DcMotor topMotor;       // right drive motor back
    DcMotor bottomMotor;    // left drive motor back

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
    Servo openCloseTop; // Sevo that opens and closes the two grabbers
    Servo rightGrabberTop; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberTop; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo
    //Servo horizontalBottom; // Servo that rotate's the grabber horizontally
    Servo openCloseBottom; // Sevo that opens and closes the two grabbers
    Servo rightGrabberBottom; // Servo that controls the grabber on the right, with a reference point looking
    // at the openClose servo
    Servo leftGrabberBottom; // Servo that controls the grabber on the left, with a reference point looking
    // at the openClose servo

    // grabber
    Servo feeler;
    double feelerRetractPosition = 0.5;
    double feelerExtendPosition = 0;



    /******************* F U N C T I O N S   F O R   A U T O *******************/

    // Configures all hardware devices, and sets them to their initial values, if necessary
    public void configure( String initialAllianceColor, String initialRobotStartingPosition ) {

        /******************* A L L I A N C E *******************/
        allianceColor = initialAllianceColor; // Options: "red" or "blue"
        robotStartingPosition = initialRobotStartingPosition; // Options: "relicSide" or "triangleSide"





        /******************* D R I V I N G *******************/

        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        topMotor = hardwareMap.dcMotor.get("topMotor");
        bottomMotor = hardwareMap.dcMotor.get("bottomMotor");

        // Motor directions: set forward/reverse
        rightMotor.setDirection(REVERSE);
        leftMotor.setDirection(FORWARD);
        topMotor.setDirection(REVERSE);
        bottomMotor.setDirection(FORWARD);





        /******************* S E N S O R S *******************/

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorColor.enableLed(true);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Initialize encoder variables to 0
        inches = 0;
        rotations = 0;
        counts = 0;





        /******************* G R A B B E R   S E R V O S *******************/

        // harware map configurations
        //horizontalTop = hardwareMap.servo.get("horizontalTop");
        openCloseTop = hardwareMap.servo.get("openCloseTop");
        rightGrabberTop = hardwareMap.servo.get("rightGrabberTop");
        leftGrabberTop = hardwareMap.servo.get("leftGrabberTop");

        //horizontalBottom = hardwareMap.servo.get("horizontalBottom");
        openCloseBottom = hardwareMap.servo.get("openCloseBottom");
        leftGrabberBottom = hardwareMap.servo.get("leftGrabberBottom");
        rightGrabberBottom = hardwareMap.servo.get("rightGrabberBottom");

        feeler = hardwareMap.servo.get("feeler");

        // Set servo direction orientations forward or reverse
        //horizontalTop.setDirection(Servo.Direction.FORWARD);
        openCloseTop.setDirection(Servo.Direction.FORWARD);
        rightGrabberTop.setDirection(Servo.Direction.REVERSE);
        leftGrabberTop.setDirection(Servo.Direction.FORWARD);

        //horizontalBottom.setDirection(Servo.Direction.FORWARD);
        openCloseBottom.setDirection(Servo.Direction.FORWARD);
        rightGrabberBottom.setDirection(Servo.Direction.REVERSE);
        leftGrabberBottom.setDirection(Servo.Direction.FORWARD);

        feeler.setDirection(Servo.Direction.FORWARD);

        // Initial positions for servos
        //horizontalTop.setPosition(.486);
        openCloseTop.setPosition(.5);
        rightGrabberTop.setPosition(.5);
        leftGrabberTop.setPosition(.5);

        //horizontalBottom.setPosition(.486);
        openCloseBottom.setPosition(.5);
        rightGrabberBottom.setPosition(.5);
        leftGrabberBottom.setPosition(.5);

        feeler.setPosition(feelerRetractPosition);

    }

    void composeIMUTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
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
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // drive function for any direction
    public void drive( String direction, double distance, double power, double time ) {

        // math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else {
            topMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
            topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        if ( direction.equalsIgnoreCase("left") || direction.equalsIgnoreCase("right") ) {
            while ( Math.abs(topMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) {
                if ( direction.equalsIgnoreCase("left") ) {
                    // Set motor powers based on paramater power
                    topMotor.setPower( -power );
                    bottomMotor.setPower( -power );
                }
                else if ( direction.equalsIgnoreCase("right") ){
                    // Set motor powers based on paramater power
                    topMotor.setPower( power );
                    bottomMotor.setPower( power );
                }


                // Telemetry for encoder position
                telemetry.addData("Current", topMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            topMotor.setPower( 0 );
            bottomMotor.setPower( 0 );
        }
        else if (direction.equalsIgnoreCase( "forwards") || direction.equalsIgnoreCase("backwards")){
            while ( Math.abs(leftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) {
                if ( direction.equalsIgnoreCase("backwards") ) {
                    // Set motor powers based on paramater power
                    leftMotor.setPower( -power );
                    rightMotor.setPower( -power );
                }
                else if ( direction.equalsIgnoreCase("forwards") ) {
                    // Set motor powers based on paramater power
                    leftMotor.setPower( power );
                    rightMotor.setPower( power );
                }

                // Telemetry for encoder position
                telemetry.addData("Current", leftMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }

            leftMotor.setPower( 0 );
            rightMotor.setPower(0);

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

    // Sets all drive train motors to 0 power
    public void stopDriving() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        topMotor.setPower(0);
        bottomMotor.setPower(0);
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

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Set run mode of leftMotor1 to STOP_AND_RESET_ENCODER
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set timeOne and timeTwo to this.getRuntime();
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while ( Math.abs(leftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) )
        {
            if ( direction.equalsIgnoreCase("clockwise") || direction.equalsIgnoreCase("cw") ) {
                leftPower = power;
                rightPower = -power;
                topPower = power;
                bottomPower = -power;
            }
            else if ( direction.equalsIgnoreCase("counterclockwise") || direction.equalsIgnoreCase("ccw") ) {
                leftPower = -power;
                rightPower = power;
                topPower = -power;
                bottomPower = power;
            }

            // Telemetry for encoder position
            telemetry.addData("Current", leftMotor.getCurrentPosition());
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

    public void dropFeelerAndMoveBasic(){

        feeler.setPosition(0);

        if ( allianceColor.equalsIgnoreCase("red") && robotStartingPosition.equalsIgnoreCase("relicSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(.8);
                    bottomMotor.setPower(.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(-.8);
                    bottomMotor.setPower(-.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE SMALLER FORWARD RIGHT DISTANCE

            }
        }
        else if ( allianceColor.equalsIgnoreCase("red") && robotStartingPosition.equalsIgnoreCase("triangleSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(.8);
                    bottomMotor.setPower(.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE

            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(-.8);
                    bottomMotor.setPower(-.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && robotStartingPosition.equalsIgnoreCase("relicSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(.8);
                    bottomMotor.setPower(.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE

            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(-.8);
                    bottomMotor.setPower(-.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE

            }
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && robotStartingPosition.equalsIgnoreCase("triangleSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(.8);
                    bottomMotor.setPower(.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE

            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                timeOne=this.getRuntime();
                timeTwo=this.getRuntime();
                while (timeTwo-timeOne<2)
                {
                    topMotor.setPower(-.8);
                    bottomMotor.setPower(-.8);
                    timeTwo = this.getRuntime();
                }
                topMotor.setPower(0);
                bottomMotor.setPower(0);
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE

            }
        }
    }


    public void dropFeelerAndMove(){

        feeler.setPosition(0);

        if ( allianceColor.equalsIgnoreCase("red") && robotStartingPosition.equalsIgnoreCase("relicSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                drive( "right", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                drive( "left", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE SMALLER FORWARD RIGHT DISTANCE
                drive( "right", 18, .3, 20 );

            }
        }
        else if ( allianceColor.equalsIgnoreCase("red") && robotStartingPosition.equalsIgnoreCase("triangleSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                drive( "right", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                drive( "left", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && robotStartingPosition.equalsIgnoreCase("relicSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                drive( "right", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                drive( "left", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && robotStartingPosition.equalsIgnoreCase("triangleSide") ){
            if ( sensorColor.blue() >= 2 && sensorColor.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                drive( "right", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
            else if ( sensorColor.red() >= 2 && sensorColor.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                drive( "left", 6, .3, 20 );
                //RETRACT SERVO
                feeler.setPosition(feelerRetractPosition);
                //DRIVE LARGER FORWARD LEFT DISTANCE
                drive( "left", 18, .3, 45 );
            }
        }
    }

    public void dropFeelerMoveBallOnly(){

        feeler.setPosition(0);

        pause( 1 );

        telemetry.addData( "Value of RED: ", sensorColor.red() );
        telemetry.addData( "Value of BLUE: ", sensorColor.blue() );
        telemetry.update ();

        pause( 1 );

        if ( allianceColor.equalsIgnoreCase("red") && sensorColor.blue() >= 14 && sensorColor.red() < 14 ) {
            drive( "right", 6, .3, 15 );
        }
        else if ( allianceColor.equalsIgnoreCase("red") && sensorColor.red() >= 14 && sensorColor.blue() < 14 ) {
            drive( "left", 6, .3, 15 );
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.blue() >= 14 && sensorColor.red() < 14) {
            drive( "left", 6, .3, 15 );
        }
        else if ( allianceColor.equalsIgnoreCase("blue") && sensorColor.red() >= 14 && sensorColor.blue() < 14 ) {
            drive( "right", 6, .3, 15);
        }

        stopDriving();
    }


}
