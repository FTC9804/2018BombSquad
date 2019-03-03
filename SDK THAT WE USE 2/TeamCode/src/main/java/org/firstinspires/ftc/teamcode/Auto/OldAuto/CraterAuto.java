package org.firstinspires.ftc.teamcode.Auto.OldAuto;

//Import Statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

//Declaration for display on the driver station
//USES OLD ROTATE METHOD
@Disabled
@Autonomous(name = "Crater Auto")
public class CraterAuto extends TensorFlow {

    int Xpos;
    Recognition lowestGold;
    boolean centerBlock, rightBlock, leftBlock;

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Crater Auto...");
        telemetry.update();

        //Initializing motors and motor directions to the hardware map
        initMotors("m2", "m1", "m5", "m4", "m3", "m6");
        initMotorDirections(REVERSE, FORWARD, REVERSE, REVERSE, FORWARD, REVERSE);

        //Initializing servos and digital devices to the hardwareMap
        initServos("s1", "s2");
        initDigitals("d1", "d2", "d3", "d4");

        //Initialize the IMU
        init("i0"); //i0

        //Initializing camera for vuforia
        initVuforia();

        //Set the zero power behavior of the motors
        setZeroPow();


        //If our phone is able to, initialize the Tfod Object Detector, if it is not, print telemetry
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            telemetry.update();

        }

        //Return the size of the screen for vuforia testing purposes
        while (!isStopRequested() && !isImuCalibrated()){
            //While a stop to the op mode has not been requested and the imu is not calibrated
            sleep(50); //Sleep for 50 milliseconds
            idle(); //Mark process as swappable and lowers its priority
        }

        centerBlock = false;
        leftBlock = false;
        rightBlock = false;

        setSwapperPosition(.05);

        lowestGold = getGoldBlock(1);
        if(lowestGold == null)
            Xpos = -1;
        else
            Xpos = (int) lowestGold.getTop();

        telemetry.addLine("Init complete!");
        telemetry.addData("Xpos: ", Xpos);
        telemetry.update();

        waitForStart();

        lowestGold = getGoldBlock(1);


        if(lowestGold == null)
            Xpos = -1;
        else
            Xpos = (int) lowestGold.getTop();

        telemetry.addData("Gold X Pos: ", Xpos);
        telemetry.update();

        pause(.2);

        dropFromHang();

        //If we didn't see a gold block, or the gold block was on the very far right of our screen
        //We assume the gold block was in the right position, and turn to the right to hit it
        if(Xpos == -1 || Xpos > 900) {
            telemetry.addLine("Right");
            telemetry.update();
            rightBlock = true;
            rotate(-25, .35, 7);
        }
        //If the gold block was in the left porton of the screen,
        //we assume the block is left, and turn towards it
        else if(Xpos < 450) {
            telemetry.addLine("Left");
            telemetry.update();
            leftBlock = true;
            rotate(25, .35, 7);
        }
        //If the block was not in the very far right or far left, but we still saw it
        //we assume the block is in the center, and do not turn towards either direction
        else {
            telemetry.addLine("Center");
            telemetry.update();
            centerBlock = true;
        }

        //Drive forward to hit the block
        driveWithEncoders(35, .4, 3);

        //Make a slight adjustment
        setLeftPower(-.35);
        pause(.2);
        setLeftPower(0);

        pause(.1);

        //Drive backwards to realign with the lander
        driveWithEncoders(35, -.3, 2.3);

        //Run the opposite motor depending on which side the block was on, so that we are parralel to the lander
        if(leftBlock){
            setRightPower(-.5);
            pause(.7);
        }
        else if(rightBlock){
            setLeftPower(-.5);
            pause(.7);
        }
        setBothPower(0);

        //Drive to the corner, values change depending on which position the blockw as in
        driveWithEncoders(19,.4, 2);
        pause(.2);
        rotate(80,.35,3);
        if(rightBlock)
            driveWithEncoders(48,.35,3);
        else
            driveWithEncoders(58,.5, 3);
        pause(.1);
        rotate(27, .35, 5);
        if(rightBlock)
            driveWithEncoders(27,.4,3);
        else
            driveWithEncoders(20,.4,3);

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();


    } //Ends runOpMode method
} //Ends class