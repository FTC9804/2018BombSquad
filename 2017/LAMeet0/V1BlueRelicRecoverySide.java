
package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 11/4/17.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueRelicSide", group = "AutoWithFunctions")

//@Disabled
public class V1BlueRelicRecoverySide extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        configure( "blue", "relicSide" );

        //Wait until play button is pressed
        waitForStart();

        pause( 2 );

        dropFeelerMoveBallOnly();

        stopDriving ();


    } // end run op mode
} // end V1RedRelicRecoverySide
