package org.firstinspires.ftc.teamcode.Auto.CurrentAuto;

//Import Statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

//Declaration for display on the driver station
//@Disabled

@Autonomous(name = "Corner Auto 2")
public class CornerAuto2 extends TensorFlow {

    //Variable declarations
    private Recognition lowestGold;
    private boolean centerBlock = false, rightBlock = false, leftBlock = false;

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Corner Auto...");
        telemetry.update();

        initAll("Corner Auto 2", "TeleopMain");

        waitForStart();

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

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander
        dropFromHang();

        //If we didn't see a gold block, or the gold block was on the very far right of our screen
        //We assume the gold block was in the right position, and turn to the right to hit it
        //If the block is right, we back up turn clockwise so our intake faces our own team's crater,
        //and drive backwards until we hit the wall. We then turn towards counter-clockwise untill our intake faces our depot
        //and drive forwards towards it.
        //We move like this in order to abide by safepaths and ensure we do not hit our alliance partner.

        if(rightBlock) {
            rotate(-10, .35, 7, "Turn towards right block");
            driveWithEncoders(35, .4, 3, "Drive and hit right block");
            driveWithEncoders(10,-.4,3, "Back up");
            rotate(-83, .35, 5, "Turn towards wall");
            driveWithEncoders(60,-.4,5, "Drive towards wall");
            rotate(-40,.34,5, "Turn towards depot");
            driveWithEncoders(20,.4,5, "Drive towards depot:");
        }
        //If the gold block was in the left porton of the screen, we assume the block is left, and turn towards it
        else if(leftBlock) {
            rotate(30, .35, 7, "Turn towards left block");
            driveWithEncoders(40,.4,3, "Drive and hit block");
            rotate(-35,.5,3, "Turn towards depot");
            driveWithEncoders(20,.4,2, "Drive towards depot");
        }
        //If the block was not in the very far right or far left, but we still saw it
        //we assume the block is in the center, and do not turn towards either direction,
        //instead we just drive straight forward, hitting the block and lining us up with the depot
        else {
            driveWithEncoders(45, .4, 3, "Drive and hit center block");
        }

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();


        //If the block was on the left, we have to turn towards the crater opposing team's crater
        if(leftBlock)
            rotate(-27, .4, 3, "Turn towards wall");
        else if(centerBlock){//If the block is in the center, we back up, turn towards  the opponenets crater, and drive towards it
            driveWithEncoders(30, -.4, 3,"Drive backwards");
            rotate(-83, .35, 5, "Turn towards wall");
            driveWithEncoders(45, -.4, 3, "Drive towards wall");
        }

        rotate(-50,.35,3, "Turn towards crater");

        //Then we drive backwards towards the opposing team's crater
        driveWithEncoders(75, -.5, 4, "Drive towards crater");

    } //Ends runOpMode method
} //Ends class