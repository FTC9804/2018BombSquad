
package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 12/1/17.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2RedRelicSideVuforia", group = "VuforiaAuto")
//@Disabled

public class V2RedRelicSideVuforia extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        configure("blue", "relicSide");

        //Wait until play button is pressed
        waitForStart();

        pause( 2 );

        /**
         * Order of Operations
         *      Drop color sensor and then hit ball
         *      detect vumark
         *      move off ramp
         *      move slowly to touch ramp and re-position
         *      move sideways to a certain spot
         *      push in
         *      pull out
         *      push in
         *      pull out
         */

        //needs to be checked to drop feeler before checking color
        dropFeelerMoveBallOnlyNewRobot();

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo - timeOne < 5) {
            timeTwo  = this.getRuntime();

            detectVuMark();
        }

        drive( "right", 15, .75, 15 );
        driveForTime( "left", .2, 5);

        if (detectVuMark().equalsIgnoreCase("right"))
        {
            drive( "right", 12, .4, 15 );
        }
        else if (detectVuMark().equalsIgnoreCase("left"))
        {
            drive( "right", 28, .4, 15 );
        }
        else
        {
            //center condition as default
            drive( "right", 20, .4, 15 );
        }

        stopDriving ();

        drive( "backwards", 12, .4, 15);
        drive( "forwards", 6, .2, 15);
        drive( "backwards", 6, .2, 15);
        drive( "forwards", 12, .4, 15);

    } // end run op mode
} // end V1RedRelicRecoverySide
