package org.firstinspires.ftc.teamcode;

/**
 * Created by stevecox on 11/4/17.
 */

//import statement

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedRelicSide", group = "AutoWithFunctions")

//@Disabled
public class V1RedRelicRecoverySide extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException {

        //Configure motors, servos and sensors
        Configure();

        //Wait until play button is pressed
        waitForStart();

        //Set timeOne and timeTwo to this.getRuntime()
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();

        //For 1 second, bring down leftSideWheels
        //and rightSideWheels to .95 position
        while (timeTwo - timeOne < 1) {
            timeTwo = this.getRuntime();
        }

        //public void calibrateGyro()
        calibrateGyro();



    }

}
