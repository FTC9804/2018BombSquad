package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "redredred", group = "VuforiaAuto")
//@Disabled


public class REDAUTO extends FunctionsForAuto {
    String vuMarkOutput = "";

    public void runOpMode() throws InterruptedException {

        configure("red", "relicSide");

        pause(.1);

        calibrateGyro();

        telemetry.addData("calibrated", true);
        telemetry.update();

        pause(.1);

        introduceAngle();

        pause(.1);

        waitForStart();

        pause(.1);

        telemetry.addData("here", true);
        telemetry.update();

        dropFeelerMoveBallOnlyNewRobot();

        pause(.1);

        vuMarkOutput = detectVuMark(2);

        pause(.1);

        driveNewIMU(10.2, 10, .25, true, false, 0);

        pause(.1);

        spinMove (88);

        if (vuMarkOutput.equalsIgnoreCase("left"))
        {
            strafeNewIMU(17, 10, .5, true, 90);
        }
        else if (vuMarkOutput.equalsIgnoreCase("right"))
        {

        }
        else
        {
            strafeNewIMU(9, 10, .5, true, 90);
        }

        pause(.1);

        driveNewIMU (12, 4, -.3, false, false, 90);

        pause(.1);

        driveNewIMU(2.25, 5, .3, true, false, 90);

        pause(.1);

        scoreBlock(true);

        pause(.1);

        driveNewIMU (3, 1.2, -.6, false, false, 90);

        pause(.1);

        driveNewIMU(5.25, 5, .3, true, false, 90);

        pause(.1);

        if (vuMarkOutput.equalsIgnoreCase("left"))
        {
            strafeNewIMU(9, 10, -.5, false, 90);
        }
        else if (vuMarkOutput.equalsIgnoreCase("right"))
        {
            strafeNewIMU(9, 10, .5, true, 90);
        }
        else
        {

        }

        getBlocks(60);

        pause(.1);

        driveNewIMU (88, 6, -.3, false, false, 90);

        pause(.1);

        driveNewIMU(2.25, 5, .3, true, false, 90);

        pause(.1);

        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        while (timeTwo-timeOne < .06)
        {
            rightMotor.setPower(.2);
            leftMotor.setPower(-.2);
            timeTwo= this.getRuntime();
        }

        pause(.1);

        scoreBlock(true);

        pause(.1);

        driveNewIMU (9, 1.2, -.6, false, false, 90);

        pause(.1);

        driveNewIMU(2.25, 5, .3, true, false, 90);
    }
}
