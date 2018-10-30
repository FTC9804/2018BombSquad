
/**
 * Created by WilderBuchanan on 1/13/18.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name = "Wilder=BlueGod", group = "VuforiaAuto")
public class blueAuto1Mod extends LinearOpMode{
    String vuMarkOutput = "";
    boolean single = true;
    int leftOverOne;
    int rightOverOne;
    int frontOverOne;
    int backOverOne;
    int direction;
    int over;
    int mode = 0; //end game or normal mode, init to normal
    int longArmAngle;
    int shortArmAngle;
    double linLeftPower;
    double linRightPower;
    double linFrontPower;
    double linBackPower;
    double rotLeftPower;
    double rotRightPower;
    double rotBackPower;
    double rotFrontPower;
    double testLeftPower;
    double testRightPower;
    double testBackPower;
    double testFrontPower;
    double finLeftPower;
    double finRightPower;
    double finFrontPower;
    double finBackPower;

    double strafeTime;
    //Driving Dc Motors
    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor backRightMotor;
    //DcMotor backLeftMotor;

    //Cube movement variables
    double rightTrigger; //double for the extent to which rightTrigger is pressed. No press is 0, full press is 1
    boolean rightBumper; //boolean for rightBumper. Set to true if rightBumper is pressed and set to false otherwise
    double leftTrigger; //double for the extent to which leftTrigger is pressed. No press is 0, full press is 1
    boolean leftBumper; //boolean for leftBumper. Set to true if leftBumper is pressed and set to false otherwise
    double panSpinPosition; //boolean that represents whether the pan is in intake or scoring position
    boolean dpadDownPressed; //boolean for dpadDown. Set to true if dpadDown is pressed and set to false otherwise
    boolean dpadRightPressed; //boolean for dpadRight. Set to true if dpadRight is pressed and set to false otherwise
    boolean dpadLeftPressed; //boolean for dpadLeft. Set to true if dpadLeft is pressed and set to false otherwise
    boolean dpadUpPressed; //boolean for dpadUp. Set to true if dpadUp is pressed and set to false otherwise
    boolean backPressed;
    boolean startPressed;
    double leftIntakePower;
    double rightIntakePower;
    double panLiftingPower;
    double leftStickY1;

    // Motor and servo configurations
    DcMotor rightIntakeMotor; //Dc motor that controls the right intake/right wheel of the intake
    DcMotor leftIntakeMotor; //Dc motor that controls the left intake/left wheel of the intake
    DcMotor panLifterMotor;
    DcMotor relicMotor;
    Servo leftPanSpin;
    Servo rightPanSpin;
    Servo panKick;
    Servo relicLongArm;
    Servo relicShortArm;
    Servo grab;
    Servo relicRotate;
    Servo extend;
    Servo swipe;
    double grabPosition = 0;
    double relicLongArmPosition;
    double relicShortArmPosition;
    double relicRotatePosition = 0;


    ColorSensor sensorColor;

    double timeOne;
    double timeTwo;

    DigitalChannel limitTop; //Touch sensor that tells us if we reach the top with the Pan
    DigitalChannel limitBottom; //Touch sensor that tells us if we reach the bottom with the Pan

    boolean endGame;
    boolean previousStatus;
    boolean currentStatus;
    boolean yPressed;
    boolean aPressed;


    /* Initialize standard Hardware interfaces */
    public void pause( double time ) {
        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < time) {
            timeTwo = this.getRuntime();
        }
    }

    public void runOpMode() {
        //configure("red", "relicSide");
        // Motor configurations in the hardware map
        rightMotor = hardwareMap.dcMotor.get("m1"); //RightMotor configuration
        leftMotor = hardwareMap.dcMotor.get("m2"); //LeftMotor configuration
        //backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor"); //BackLeftMotor configuration
        backRightMotor = hardwareMap.dcMotor.get("m3"); //BackRightMotor configuration
        rightIntakeMotor = hardwareMap.dcMotor.get("m4"); //rightIntakeMotor configuration
        leftIntakeMotor = hardwareMap.dcMotor.get("m5"); //leftIntakeMotor configuration
        panLifterMotor = hardwareMap.dcMotor.get("m6"); //panLifterMotor configuration
        //relicMotor = hardwareMap.dcMotor.get("m7");

        // Servo configurations in the hardware map
        leftPanSpin = hardwareMap.servo.get("s1"); //leftPanSpin configuration
        rightPanSpin = hardwareMap.servo.get("s2"); //rightPanSpin configuration
        //relicLongArm = hardwareMap.servo.get("s3");
        //relicShortArm = hardwareMap.servo.get("s4");
        //panKick = hardwareMap.servo.get("s5");
        //grab = hardwareMap.servo.get("s6");
        //relicRotate = hardwareMap.servo.get("s7");

        extend = hardwareMap.servo.get("s8");
        swipe = hardwareMap.servo.get("s9");

        // Touch sensor configuration in the hardware map
        //limitTop = hardwareMap.get(DigitalChannel.class, "d1"); //Top touchSensor configuration
        //limitBottom = hardwareMap.get(DigitalChannel.class, "d2");

        sensorColor = hardwareMap.get(ColorSensor.class, "i1");


        // Motor directions
        rightMotor.setDirection(REVERSE); //Set rightMotor to FORWARD direction
        leftMotor.setDirection(FORWARD); //Set leftMotor to REVERSE direction
        backRightMotor.setDirection(REVERSE); //Set backRightMotor to REVERSE direcion
        //backLeftMotor.setDirection(REVERSE); //Set backLeftMotor to REVERSE direction
        rightIntakeMotor.setDirection(REVERSE); //Set rightIntakeMotor to FORWARD direction
        leftIntakeMotor.setDirection(FORWARD); //Set leftIntakeMotor to FORWARD direction
        panLifterMotor.setDirection(REVERSE); //Set panLifterMotor to FORWARD direction
        //relicMotor.setDirection(FORWARD);

        // Servo directions
        leftPanSpin.setDirection(Servo.Direction.REVERSE); //Set leftPanSpin to REVERSE direction
        rightPanSpin.setDirection(Servo.Direction.FORWARD); //Set rightPanSpin to FORWARD direction
        swipe.setDirection(Servo.Direction.FORWARD);
        //panKick.setDirection(Servo.Direction.FORWARD);
        //grab.setDirection(Servo.Direction.REVERSE);
        //relicRotate.setDirection(Servo.Direction.REVERSE);

        //Init powers
        rightMotor.setPower(0); //Set rightMotor to 0 power
        leftMotor.setPower(0); //Set leftMotor to 0 power
        backRightMotor.setPower(0); //Set backRightMotor to 0 power
        //backLeftMotor.setPower(0); //Set backLeftMotor to 0 power
        rightIntakeMotor.setPower(0); //Set rightIntakeMotor to 0 power
        leftIntakeMotor.setPower(0); //Set leftIntakeMotor to 0 power
        panLifterMotor.setPower(0); //Set panLifterMotor to 0 power
        strafeTime = 1;
        //grab.setPosition(.2);
        //relicRotate.setPosition( .3 );

        leftPanSpin.setPosition(.2);
        rightPanSpin.setPosition(.2);
        swipe.setPosition(0);
        extend.setPosition(.5);


        //panKick.setPosition(1);

        //endGame = false;
        //previousStatus = false;
        //currentStatus = false;

        double swipeNeutralPosition = .5;
        double swipeCWPosition = .25; //tentative, cw
        double swipeCCWPosition = .75; //tentative, ccw
     //   boolean bLedOn = true;


        sensorColor.enableLed(true);


        waitForStart();


        sensorColor.enableLed(true);


        extend.setPosition(.5);

      /*  vuMarkOutput = detectVuMark( 5 );

        if (vuMarkOutput.equalsIgnoreCase("right"))
        {
            strafeTime = 1.5;
        }
        else if (vuMarkOutput.equalsIgnoreCase("left"))
        {
            strafeTime = 1;
        }
        else
        {
            //center condition as default
            strafeTime = 1.25;
        }
        */
        pause(.2);

        timeOne = this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne < .5)
        {
            timeTwo=this.getRuntime();
            extend.setPosition(1);
        }

        extend.setPosition(.5);

        pause (.2);

        swipe.setPosition(.65);

        pause(1);

        timeOne = this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne < 2.5)
        {
            timeTwo=this.getRuntime();
            extend.setPosition(1);
        }

        extend.setPosition(.5);

        timeOne = this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne < .1)
        {
            timeTwo=this.getRuntime();
            extend.setPosition(0);
        }

        extend.setPosition(.5);


        if(sensorColor.blue() > sensorColor.red())
        {
            swipe.setPosition(1);
            pause(1);

            timeOne = this.getRuntime();
            timeTwo=this.getRuntime();

            while (timeTwo-timeOne < 1)
            {
                timeTwo=this.getRuntime();
                extend.setPosition(0);
            }
            swipe.setPosition(0);

            telemetry.addData("red",sensorColor.red());
            telemetry.addData("Blue",sensorColor.blue());
            telemetry.update();
            pause(1);
        }
        else
        {
            swipe.setPosition(0);

            timeOne = this.getRuntime();
            timeTwo=this.getRuntime();

            while (timeTwo-timeOne < 1)
            {
                timeTwo=this.getRuntime();
                extend.setPosition(0);
            }
            telemetry.addData("red",sensorColor.red());
            telemetry.addData("Blue",sensorColor.blue());
            pause(1);
        }

        timeOne = this.getRuntime();
        timeTwo=this.getRuntime();

        while (timeTwo-timeOne < 1)
        {
            timeTwo=this.getRuntime();
            extend.setPosition(0);
        }

       pause(1);

        backRightMotor.setPower(1);
        leftMotor.setPower(.25);
        rightMotor.setPower(-.25);

        pause(1.5);

        backRightMotor.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        pause(.5);

        panLifterMotor.setPower(.5);

        pause(.4);

        panLifterMotor.setPower(0);

        leftPanSpin.setPosition(.6);
        rightPanSpin.setPosition(.6);
        pause(3);
        leftPanSpin.setPosition(0);
        rightPanSpin.setPosition(0);

        rightMotor.setPower(.5);
        leftMotor.setPower(.5);

        pause(.5);

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        rightMotor.setPower(-.5);
        leftMotor.setPower(-.5);

        pause(2);

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        pause(.5);

        rightMotor.setPower(.5);
        leftMotor.setPower(.5);

        pause(.5);

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        pause(.5);

        rightMotor.setPower(-.5);
        leftMotor.setPower(-.5);

        pause(1);

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        pause(.5);

        rightMotor.setPower(.5);
        leftMotor.setPower(.5);

        pause(.25);

        rightMotor.setPower(0);
        leftMotor.setPower(0);







    }
}
