package org.firstinspires.ftc.teamcode;

/**
 * Created by MarcusLeher on 18/11/2017.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "encoders", group = "AutoWithFunctions")

//@Disabled
public class EncoderTestTurn extends FunctionsForAuto {

    public void runOpMode() {
        //Configure motors, servos and sensors
        configure("blue", "triangleSide");

        //Wait until play button is pressed
        waitForStart();

        encoders();
    }
}