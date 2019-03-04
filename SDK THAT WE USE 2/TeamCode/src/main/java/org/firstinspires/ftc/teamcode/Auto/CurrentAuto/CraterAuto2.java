package org.firstinspires.ftc.teamcode.Auto.CurrentAuto;

//Import Statements

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

//Declaration for display on the driver station
//@Disabled

@Autonomous(name = "Crater Auto 2")
public class CraterAuto2 extends TensorFlow {

    private Recognition lowestGold;
    private boolean leftBlock = false, rightBlock = false, centerBlock = false;

    //Main OpMode method
    public void runOpMode() {

        TelemetryThread th = new TelemetryThread("TelThread");

        initAll("Crater Auto 2", "TeleopMain");//Init all motors, servos, and sensors (including gyro imu, Tfod, and vuforia)

        waitForStart();

        resetStartTime();

        lowestGold = getGoldBlock(1);

        if(lowestGold == null || ((int)lowestGold.getTop()) > 900) { //Return the l/c/r position of the gold mineral
            setGoldMineralPosTelemetry("Right");
            rightBlock = true;
        }else if(lowestGold.getTop() < 450) {
            setGoldMineralPosTelemetry("Left");
            leftBlock = true;
        }else {
            setGoldMineralPosTelemetry("Center");
            centerBlock = true;
        }telemetry.update();

        th.startThread();

        //We start hanging, so we haft to drop
        dropFromHang();

        //Run our robot to the corner to drop our marker
        driveWithEncoders(17,.4, 5, "Drive to Gap");
        rotate(80,.35,3, "Turn to wall");
        driveWithEncoders(65,.5, 3, "Drive to Wall");
        rotate(115, .35, 5, "Turn to Depot");
        driveWithEncoders(25,.4,5, "Drive to Depot");

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();

        rotate(130  ,.35,5, "Allign to wall");

        //Drive back and realign with the lander
        driveWithEncoders(37,-.4,3, "Drive away from Depot");
        pause(.1);
        rotate(95, .35, 5, "turn towards center");

        //If we didn't see a gold block, or the gold block was on the very far right of our screen
        //We assume the gold block was in the right position, and turn to the right to hit it
        if(rightBlock) {
            driveWithEncoders(70,-.5, 3, "Drive to right block");
            rotate(-5, .35, 7, "Turn towards right block");
        }
        //If the gold block was in the left porton of the screen, we assume the block is left, and turn towards it
        else if(leftBlock) {
            driveWithEncoders(40,-.5, 3, "Drive to left block");
            rotate(15, .35, 7, "Turn towards left block");
        }
        //If the block was not in the v ery far right or far left, but we still saw it
        //we assume the block is in the center, and do not turn towards either direction
        else {
            driveWithEncoders(48,-.5, 3, "Drive to center block");
            rotate(7, .35, 7, "Turn towards center block");
        }

        //Drive forward to hit the block and partially park in the crater
        driveWithEncoders(30, .4, 2, "Drive forward and hit block");
        setExtenderPower(.25);
        pause(1);
        setExtenderPower(0);
    } //Ends runOpMode method
} //Ends class

