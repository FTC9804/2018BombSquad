

package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 1/12/18.
 */

//import statement
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Red Corner M3", group = "VuforiaAuto")
@Disabled

public class RedCornerMeet3 extends FunctionsForAuto {

    String vuMarkOutput = "";

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        configure("red", "relicSide");

        //Wait until play button is pressed
        waitForStart();

        /**
         * 1. Vuforia check with the balls (jewel mech)
         * 2. Strafe to the right for 20 inches, 28 inches, or 36 (pending results)
         * 3. Backup 3 inches
         * 4. Score function
         * 5. Backup for 1 second 50% power
         * 6. Forward 7 inches
         * 7. Backup for 2 seconds 50% power
         * 8. Forward 6 inches --> DONE
         */

        relicMotorForTime( .5, .5 );
        dropFeelerMoveBallOnlyNewRobot();

        vuMarkOutput = detectVuMark( 5 );

        if (vuMarkOutput.equalsIgnoreCase("right"))
        {
            strafeNewIMU( 20, 7, .5, true );
        }
        else if (vuMarkOutput.equalsIgnoreCase("left"))
        {
            strafeNewIMU( 36, 7, .5, true );
        }
        else
        {
            //center condition as default
            strafeNewIMU( 28, 7, .5, true );
        }

        driveNewIMU( 3, 3, .5, false );

        scoreBlock( true );

        driveForTime( .5, 1, false );

        driveNewIMU( 7, 5, .5, true );

        driveForTime( .5, 2, false );

        driveNewIMU( 6, 5, .5, true );


    } // end run op mode
} // end V1RedRelicRecoverySide
