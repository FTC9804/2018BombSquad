

package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 12/1/17.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "VuMarkAutoMethodTest", group = "VuforiaAuto")
//@Disabled

public class VuMarkAutoMethodTest extends FunctionsForAuto {

    String vuMarkOutput = "";

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        configure("red", "relicSide");

        //Wait until play button is pressed
        waitForStart();

        vuMarkOutput = detectVuMark( 30 );


    } // end run op mode
} // end V1RedRelicRecoverySide
