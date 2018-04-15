//Package declaration
package org.firstinspires.ftc.teamcode;

//Import statements
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by MarcusLeher on 30/12/2017.
 */

@Autonomous(name = "redNOTcorner", group = "VuforiaAuto")
//@Disabled

//Autonomous to score ball and a block from the red non-corner position
public class REDAUTONOTCORNER extends FunctionsForAuto {

    public void runOpMode() throws InterruptedException { //runOpMode() method

        configure("red", "notcorner"); //Configure with parameters red and relicSide

        pause(.05); //pause 

        calibrateGyro(); //Calibrate the gyro

        //Telemetry
        telemetry.addData("calibrated", true);
        telemetry.update();

        pause(.05); //pause 

        introduceAngle(); //Introduce the angle

        pause(.05); //pause

        waitForStart(); //Wait for start

        pause(.05); //pause

        //Telemetry
        telemetry.addData("here", true);
        telemetry.update();

        dropFeelerMoveBallOnlyNewRobot(); //Run dropFeelerMoveBallOnlyNewRobot to score the ball

        pause(.05); //pause

        driveNewIMU(6, 10, .25, true, 0); //Drive forward

        frontBarDown(); //Lower the front bar to prevent faulty glyphs from intaking

        pause(.01); //pause 

        if (vuMarkReturn.equalsIgnoreCase("left")) //If vuforia reads left
        {
            spinMove(225, false, 5, true); //Spin to orient robot for glyph scoring

            pause(.01); //pause 

            driveNewIMU(8, 3.2, -.3, false, 225);

            pause(.01); //pause 

            release();

            pause(.7); //pause 

            lowerPan();

            pause(.01); //pause 

            driveNewIMU(8.5, 1.6, -.3, false, 225);

            pause(.01); //pause 

            driveNewIMU(6.1, 5, .3, true, 230);
            
            pause(.01); //pause 

            spinMove(136, false, 5, false);
            
            pause(.01); //pause 

            driveNewIMU(24, 3, .86, true, 125);
            
            pause(.01); //pause 

            spinMove(170, false, 5, false);
            
            pause(.01); //pause 

            getBlockOne(); 

            pause(.01); //pause 

            driveNewIMU(6, 3, -.4, false, 180);

            pause(.01); //pause 

            spinMove(160, false, 1, false);

            pause(.01); //pause 

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                getBlockTwo();
            }

            spinMove(150, false, 1, false);

            grab();

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8))
            {
                driveNewIMU(76, 3.6, -.5, false, 145);

                pause(.01); //pause 

                driveNewIMU(4.9, 1, .5, true, 145);

            }
            else {

                driveNewIMU(76, 3.6, -.5, false, 145);

                raisePan();

                if (this.getRuntime() - threeGlyphTimeOne > 25)
                {
                    release();
                }

                driveNewIMU(4.9, 1, .5, true, 145);

                pause(.2); //pause 
                
                release();

                if (this.getRuntime() - threeGlyphTimeOne < 25) {

                    pause(.6); //pause 

                    lowerPan();

                    driveNewIMU(10, 1.6, -.5, false, 140);

                    pause(.01); //pause 

                    driveNewIMU(6, 3, .5, true, 160);
                }
                
            }
            
        }
        else if (vuMarkReturn.equalsIgnoreCase("right")) //If vuforia reads right
        {
            raisePan();

            pause(.01); //pause 

            pivot(-160, 4);

            pause(.01); //pause 

            release();

            pause(.7); //pause 

            lowerPan();

            pause(.01); //pause 

            driveNewIMU(12, 1.6, -.3, false, -160);

            pause(.01); //pause 

            driveNewIMU(6.1, 5, .3, true, -160);

            pause(.01); //pause 

            spinMove(-80, false, 5, false);
            
            pause(.01); //pause 

            driveNewIMU(29, 3, -1, false, -70);

            pause(.01); //pause 

            spinMove(-180, false, 5, false);

            getBlockOne();

            pause(.01); //pause 

            driveNewIMU(6, 3, -.4, false, -180);

            pause(.01); //pause 

            spinMove(-170, false, 1, false);

            pause(.01); //pause 

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                getBlockTwo();
            }

            spinMove(-195, false, 1, false);

            grab();

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8))
            {
                driveNewIMU(76, 3.6, -.5, false, -195);

                pause(.01); //pause 

                driveNewIMU(4.9, 1, .5, true, -195);
            }
            else {

                driveNewIMU(76, 3.6, -.5, false, -195);

                pause(.01); //pause 

                raisePan();
                
                pause(.01); //pause 

                if (this.getRuntime() - threeGlyphTimeOne > 25)
                {
                    release();
                }

                driveNewIMU(5.1, 1, .5, true, -195);

                pause(.01); //pause 
                
                release();

                if (this.getRuntime() - threeGlyphTimeOne < 25) {

                    pause(.6); //pause 

                    lowerPan();
                    
                    pause(.01); //pause 

                    driveNewIMU(10, 1.6, -.5, false, -170);

                    pause(.01); //pause 

                    driveNewIMU(6, 3, .5, true, -190);
                }
            }

        }
        else //else
        {
            spinMove(211, false, 5, true);

            pause(.01);

            release();

            pause(.6);

            lowerPan();

            driveNewIMU(12.5, 1.6, -.3, false, 211);

            pause(.01);

            driveNewIMU(6.1, 5, .3, true, 211);
            
            pause(.01);

            spinMove(125, false, 5, false);
            
            pause(.01);

            driveNewIMU(33, 3, .86, true, 125);
            
            pause(.01);

            spinMove(170, false, 5, false);

            getBlockOne();

            pause(.01);

            driveNewIMU(6, 3, -.4, false, 180);

            pause(.01);

            spinMove(160, false, 1, false);

            pause(.01);

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) || !(sensorC.getDistance(DistanceUnit.CM) < 8)) {
                getBlockTwo();
            }

            spinMove(150, false, 1, false);

            grab();

            if (!(sensorB.getDistance(DistanceUnit.CM) < 8) && !(sensorC.getDistance(DistanceUnit.CM) < 8))
            {
                driveNewIMU(76, 5, -.5, false, 145);

                pause(.01);

                driveNewIMU(4.9, 1, .5, true, 145);

            }
            else {

                driveNewIMU(76, 3.6, -.5, false, 145);
                
                pause(.01);
                
                raisePan();
                
                pause(.01);

                if (this.getRuntime() - threeGlyphTimeOne > 25)
                {
                    release();
                }
                
                driveNewIMU(5.1, 1, .5, true, 145);

                pause(.01);

                release();

                if (this.getRuntime() - threeGlyphTimeOne < 25) {

                    pause(.6);

                    lowerPan();
                    
                    driveNewIMU(10, 1, -.5, false, 170);

                    pause(.01);

                    driveNewIMU(6, 3, .5, true, 190);
                    
                }
            }
        }



    }

      

            
    

}




