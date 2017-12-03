

package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 12/1/17.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "V2RedRelicSideVuforia", group = "VuforiaAuto")
//@Disabled

public class V2RedRelicSideVuforia extends FunctionsForAuto {

    String vuMarkOutput = "";

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        configure("red", "relicSide");

        //Wait until play button is pressed
        waitForStart();

        pause(.2);

        /**
         * Order of Operations
         *      Drop color sensor and then hit ball
         *      detect vumark
         *      move off ramp
         *          //ramp adjsuted/moved???
         *      spin 180º
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

            vuMarkOutput = detectVuMark( 5 );

        drive( "right", 15, .75, 15 );
        spinMove ("clockwise", 26.24, .5, 100);
//        driveForTime( "right", .2, 5);
//
//        if (vuMarkOutput.equalsIgnoreCase("right"))
//        {
//            drive( "left", 12, .4, 15 );
//        }
//        else if (vuMarkOutput.equalsIgnoreCase("left"))
//        {
//            drive( "left", 28, .4, 15 );
//        }
//        else
//        {
//            //center condition as default
//            drive( "left", 20, .4, 15 );
//        }
//
//        stopDriving ();
//
//        drive( "forward", 12, .4, 15);
//        drive( "backwards", 6, .2, 15);
//        drive( "forward", 6, .2, 15);
//        drive( "backwards", 12, .4, 15);

    } // end run op mode
} // end V1RedRelicRecoverySide
