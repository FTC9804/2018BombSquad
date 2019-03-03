package org.firstinspires.ftc.teamcode.Auto.OldAuto;

//Import Statements
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Auto.HelperClasses.TensorFlow;

//Declaration for display on the driver station
//USESES OLD ROTATE METHOD
@Disabled
@Autonomous(name = "Corner Auto")
public class CornerAuto extends TensorFlow {

    //Variable declarations
    int Xpos;
    Recognition lowestGold;
    boolean centerBlock, rightBlock, leftBlock;

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Corner Auto...");
        telemetry.update();

        initAll("Corner Auto", "TeleopMain");

        waitForStart();

        lowestGold = getGoldBlock(1);

        if(lowestGold == null)
            Xpos = -1;
        else
            Xpos = (int) lowestGold.getTop();

        telemetry.addData("Gold X Pos: ", Xpos);
        telemetry.update();

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander
        dropFromHang();

        //If we didn't see a gold block, or the gold block was on the very far right of our screen
        //We assume the gold block was in the right position, and turn to the right to hit it
        if(Xpos == -1 || Xpos > 900) {
            telemetry.addLine("Right");
            telemetry.update();
            rightBlock = true;
            rotate(-18, .35, 7);
        }
        //If the gold block was in the left porton of the screen, we assume the block is left, and turn towards it
        else if(Xpos < 450) {
            telemetry.addLine("Left");
            telemetry.update();
            leftBlock = true;
            rotate(30, .35, 7);
        }
        //If the block was not in the very far right or far left, but we still saw it
        //we assume the block is in the center, and do not turn towards either direction
        else {
            telemetry.addLine("Center");
            telemetry.update();
            centerBlock = true;
        }

        if(leftBlock)
            driveWithEncoders(40,.4,3);
        else
            driveWithEncoders(35, .4, 3);


        //Drive forward to hit the blocks

        pause(.1);

        //Turn towards the depot, or dont turn if the block was in the center
        if(leftBlock){
            rotate(-35,.4,3);
        }
        else if(rightBlock){
            rotate(35,.4,3);
        }

        //Drive forward towards the depot
        driveWithEncoders(20,.4, 2);

        pause(.1);

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();

        //If we are on the right side, we run backwards, turn towards the opposing team's crater,
        //and drive forward, making a slight adjustment towards the end as to make sure we hit the crater
        //and not the wall to the right of the crater
        if(rightBlock){
            driveWithEncoders(27,-.4, 2);
            rotate(70,.4,3);
            driveWithEncoders(75,.5, 3);
            rotate(13,.35,4);
            driveWithEncoders(20,.4,3);
        }
        //If we are on the left side, we only have to turn and drive straight towards the crater
        else if(leftBlock){
            rotate(-27,.4,3);
            driveWithEncoders(-75,.5, 3);
        }
        //Else, we are center, so we drive backwards and turn towards the crater, making a slight adjustment
        //at the end as to make sure we hit the crater and no the wall to the right of the crater
        else{
            driveWithEncoders(30,-.4, 2);
            rotate(90,.4,3);
            driveWithEncoders(50,.5, 3);
            rotate(17,.35,4);
            driveWithEncoders(20,.4,3);
        }

    } //Ends runOpMode method
} //Ends class