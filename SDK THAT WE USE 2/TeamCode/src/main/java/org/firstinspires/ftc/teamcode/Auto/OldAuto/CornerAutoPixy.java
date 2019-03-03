package org.firstinspires.ftc.teamcode.Auto.OldAuto;

//Import Statements

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.HelperClasses.PixyCam;

//Declaration for display on the driver station
//USES OLD ROTATE

@Disabled
@Autonomous(name = "Corner Auto Pixy")
public class CornerAutoPixy extends PixyCam {

    //Main OpMode method
    public void runOpMode() {

        telemetry.addLine("Running Corner Auto 2..."); telemetry.update();

        initAll("Corner Auto Pixy", "TeleopMain");//Init all motors, servos, and sensors except Vuforia
        //because we do not use it in this class, also automatically transition to TeleopMain when we finish this auto

        waitForStart();

        //We start hanging, so we call the method dropFromHang(), which pulls out the lock,
        //lowers us down, and unlaches us from the lander, followed by an imu turn to make us
        //parallel to the lander. This method does not run if we chose not to drop.
        dropFromHang();

        //Use our front pixyCam to see the gold block, face towards it exactly, and drive through it
        faceAndHitWithPixy(15);

        //Rotate either left, right, or not at all depending on the mineral position
        //mineral pos = -1 for left, 0 for center, and 1 for right
        rotate(getMineralPos()*35, .35, 4);


        driveWithEncoders(20,.4, 2);//Drive forward towards the depot and drop our marker

        //Call the method dropMarker(), which extends our intake and runs the intake outwards,
        //which pushes the marker out of our robot. It then retracts the intake and runs the extender
        //at a constant -.2 power, so it doesn't fall down again on the feild, messing us up and damaging the intake
        dropMarker();

        //If we are on the right side, we run backwards, turn towards the opposing team's crater,
        //and drive forward, making a slight adjustment towards the end as to make sure we hit the crater
        //and not the wall to the right of the crater
        if(getMineralPos() == 1){
            driveWithEncoders(27,-.4, 2);
            rotate(70,.4,3);
            driveWithEncoders(75,.5, 3);
            rotate(13,.35,4);
            driveWithEncoders(20,.4,3);
        }
        //If we are on the left side, we only have to turn and drive straight towards the crater
        else if(getMineralPos() == -1){
            rotate(-27,.4,3);
            driveWithEncoders(-75,.5, 3);
        }
        //Else, we are center, so we drive backwards and turn towards the crater, making a slight adjustment
        //at the end as to make sure we hit the crater and no the wall to the right of the crater
        else {
            driveWithEncoders(30,-.4, 2);
            rotate(90,.4,3);
            driveWithEncoders(50,.5, 3);
            rotate(17,.35,4);
            driveWithEncoders(20,.4,3);
        }
    } //Ends runOpMode method
} //Ends class