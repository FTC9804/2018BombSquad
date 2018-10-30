

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

        //pause(.2);

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

        vuMarkOutput = detectVuMark( 5 );

        grabAndLiftBlock(.5, .5);

        drive("right", 25, .5, 15);

        pause(0.5);

        spin180(.3, 10);

        if (vuMarkOutput.equalsIgnoreCase("right"))
        {
            drive( "left", 8, .4, 15 );
        }
        else if (vuMarkOutput.equalsIgnoreCase("left"))
        {
            drive( "left", 24, .4, 15 );
        }
        else
        {
            //center condition as default
            drive( "left", 16, .4, 15 );
        }

        pause(1.0);

        driveForTime("forwards", .4, 2);

        pause(0.5);

        lowerAndReleaseBlock(.2, 5);

        pause(2.0 );

        driveForTime("backwards", .4, .5);
        driveForTime("forwards", .4, 1);
        driveForTime("backwards", .4, .5);

    } // end run op mode
} // end V1RedRelicRecoverySide
