/**
* Created by stevecox on 11/4/17.
* Code for Harvey's autonomous
*/
// package declaration
package org.firstinspires.ftc.teamcode;

// import statements
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

    TouchSensor touchSensorTop; //  declare touch sensors for grabbers
    TouchSensor touchSensorBottom;

    ColorSensor colorSensorFeeler; // Right color feeler for balls autonomous

    // Touch Sensor variables
    boolean touchTopPress = false;
    boolean touchBottomPress = false;





    /******************* V U F O R I A *******************/

    // Variable Declarations
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;





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




    /******************* F U N C T I O N S   F O R   A U T O *******************/

    // drive function for any direction
    public void drive( String direction, double distance, double power, double time ) {

        // math to calculate total counts robot should travel
        inches = distance;
        rotations = inches / (Math.PI * WHEEL_DIAMETER);
        counts = ENCODER_CPR * rotations * GEAR_RATIO;

        if ( direction.equals("left") || direction.equals("right") ) {
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

        if ( direction.equals("left") || direction.equals("right") ) {
            while ( Math.abs(leftMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) {
                // Set motor powers based on paramater power
                leftMotor.setPower( power );
                rightMotor.setPower( power );
                topMotor.setPower( 0 );
                bottomPower.setPower( 0 );

                // Telemetry for encoder position
                telemetry.addData("Current", leftMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }
        }
        else if (direction.equals( "forwards") || direction.equals("backwards")){
            while ( Math.abs(topMotor.getCurrentPosition()) < counts && (timeTwo - timeOne < time) ) {
                // Set motor powers based on paramater power
                topMotor.setPower( power );
                bottomPower.setPower( power );
                leftMotor.setPower( 0 );
                rightMotor.setPower( 0 );

                // Telemetry for encoder position
                telemetry.addData("Current", topMotor.getCurrentPosition());
                telemetry.update();
                // Set timeTwo to this.getRuntime ()
                timeTwo = this.getRuntime();
            }
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
               if ( direction.equals("clockwise") || direction.equals("cw") ) {
                   leftPower = power;
                   rightPower = -power;
                   topPower = power;
                   bottomPower = -power;
               }
               else if ( direction.equals("counterclockwise") || direction.equals("ccw") ) {
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

    // Configures all hardware devices, and sets them to their initial values, if necessary
    public void configure( String initialAllianceColor, String initialRobotStartingPosition ) {

        /******************* A L L I A N C E *******************/
        allianceColor = initialAllianceColor; // Options: "red" or "blue"
        robotStartingPosition = initialRobotStartingPosition; // Options: "relicSide" or "triangleSide"





        /******************* V U F O R I A *******************/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AZfTpOj// // /AAAAGYCE1z7z6E5whPRKfYeRJHEN/u/+LZ7AMmBU0bBa" +
        "/7u6aTruUWfYeLur6nSFdKP0w9JPmK1gstNxVHqiaZN6iuZGxPcbnDnm" +
        "NJdoLIMtZheeNWphUMjHKoTUgsmcloZe67TG2V9duc+8jxxCLFzH5rlq" +
        "PPdcgvvtIO0orpxVcpENBunY2GChhVgP6V5T9Iby7MyM9tN+y7Egm7Xy" +
        "Iz/Tzpmlj19b3FUCW4WUDjTNQ4JoKZeB1jkhPxKGFRECoPw02jJXtQSK" +
        "zNfzmhtugA7PTOZNehc61UjOXEexTO9TRy7ZfMtW8OggcYssvIabyJ8b" +
        "DK4ePLCUP+Q4PMf7kL9lM6yDuxxKF0oqLgRglX9Axqrf";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);





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
        // gyro and setup
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); // I2C port 0
        // Set gyro variables to 0
        currentHeading = 0;
        initialHeading = 0;
        headingError = 0;
        turnSpeed = 0;

        // color sensor and setup
        I2cAddr i2cColorSensorFeeler = I2cAddr.create8bit(0x5c); // Create I2C address of colorSensorLeft
        // requires moving connection based on alliance color
        colorSensorFeeler = hardwareMap.colorSensor.get("colorSensorFeeler");     // I2C port 2
        colorSensorFeeler.setI2cAddress(i2cColorFeeler); // set I2C address of colorSensorRight
        colorSensorFeeler.enableLed(false); // Set enableLed of colorSensorRight to false

        touchSensorTop = hardwareMap.touchSensor.get("touchSensorTop");
        touchSensorBottom = hardwareMap.touchSensor.get("touchSensorBottom");

        // Initialize encoder variables to 0
        inches = 0;
        rotations = 0;
        counts = 0;





        /******************* G R A B B E R   S E R V O S *******************/

        // harware map configurations
        horizontalTop = hardwareMap.servo.get("horizontalTop");
        openCloseTop = hardwareMap.servo.get("openCloseTop");
        rightGrabberTop = hardwareMap.servo.get("rightGrabberTop");
        leftGrabberTop = hardwareMap.servo.get("leftGrabberTop");

        horizontalBottom = hardwareMap.servo.get("horizontalBottom");
        openCloseBottom = hardwareMap.servo.get("openCloseBottom");
        leftGrabberBottom = hardwareMap.servo.get("leftGrabberBottom");
        rightGrabberBottom = hardwareMap.servo.get("rightGrabberBottom");

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

    public void dropFeeler(){

        // MOVE SERVO FEELER

        if ( allianceColor.equals("red") && robotStartingPosition.equals("relicSide") ){
            if ( colorSensorFeeler.blue() >= 2 && colorSensorFeeler.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
            else if ( colorSensorFeeler.red() >= 2 && colorSensorFeeler.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE SMALLER FORWARD LEFT DISTANCE
            }
        }
        else if ( allianceColor.equals("red") && robotStartingPosition.equals("triangleSide") ){
            if ( colorSensorFeeler.blue() >= 2 && colorSensorFeeler.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
            else if ( colorSensorFeeler.red() >= 2 && colorSensorFeeler.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
        }
        else if ( allianceColor.equals("blue") && robotStartingPosition.equals("relicSide") ){
            if ( colorSensorFeeler.blue() >= 2 && colorSensorFeeler.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
            else if ( colorSensorFeeler.red() >= 2 && colorSensorFeeler.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF BLUE
                //RETRACT SERVO
                //DRIVE LARGER FORWARD RIGHT DISTANCE
            }
        }
        else if ( allianceColor.equals("blue") && robotStartingPosition.equals("triangleSide") ){
            if ( colorSensorFeeler.blue() >= 2 && colorSensorFeeler.red() < 2 ){
                // DRIVE RIGHT TO KNOCK OFF RED
                //RETRACT SERVO
                //DRIVE LARGER FORWARD RIGHT DISTANCE
            }
            else if ( colorSensorFeeler.red() >= 2 && colorSensorFeeler.blue() < 2 ){
                // DRIVE LEFT TO KNOCK OFF RED
                //RETRACT SERVO
                //DRIVE LARGER FORWARD LEFT DISTANCE
            }
        }
    }


}
