package org.firstinspires.ftc.teamcode.Auto.OldAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;

/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
        */
@Disabled
@Autonomous(name = "PixyExample")
public class PixyExample extends LinearOpMode {
    I2cDeviceSynch pixy;
    Servo pixys;

    //our Pixy device
    @Override
    public void runOpMode() throws InterruptedException {
        //setting up Pixy to the hardware map
        pixy = hardwareMap.i2cDeviceSynch.get("p1");
        pixys = hardwareMap.servo.get("pixys");
        pixys.setPosition(.5);


        //setting Pixy's I2C Address
        pixy.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the
        // SDK Documentation to learn more
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,I2cDeviceSynch.ReadMode.REPEAT);
        pixy.setReadWindow(readWindow);

        //required to "turn on" the device
        pixy.engage();
        waitForStart();

        while(opModeIsActive()) {
            //send every byte of data that we can to the phone screen
            byte[][] stuff1 = new byte[14][];
            for(int o = 0; o < 14; o++){
                 stuff1[o] = pixy.read(0x51,o);
            }
            for(byte[] bytes : stuff1){
                for(byte byt1 : bytes){
                    telemetry.addData("Stuff: ", byt1);
                }
            }

            telemetry.addData("Byte 0", pixy.read(0x51,0));
            telemetry.addData("Byte 1", pixy.read(0x51,1));
            telemetry.addData("Byte 2", pixy.read(0x51,2));
            telemetry.addData("Byte 3", pixy.read(0x51,3));
            telemetry.addData("Byte 4", pixy.read(0x51,4));
            telemetry.addData("Byte 5", pixy.read(0x51,5));
            telemetry.addData("Byte 6", pixy.read(0x51,6));
            telemetry.addData("Byte 7", pixy.read(0x51,7));
            telemetry.addData("Byte 8", pixy.read(0x51,8));
            telemetry.addData("Byte 9", pixy.read(0x51,9));
            telemetry.addData("Byte 10", pixy.read(0x51,10));
            telemetry.addData("Byte 11", pixy.read(0x51,11));
            telemetry.addData("Byte 12", pixy.read(0x51,12));
            telemetry.addData("Byte 13", pixy.read(0x51,13));
            telemetry.addData("Byte 0", pixy.read8(0));
            telemetry.addData("Byte 1", pixy.read8(1));
            telemetry.addData("Byte 2", pixy.read8(2));
            telemetry.addData("Byte 3", pixy.read8(3));
            telemetry.addData("Byte 4", pixy.read8(4));
            telemetry.addData("Byte 5", pixy.read8(5));
            telemetry.addData("Byte 6", pixy.read8(6));
            telemetry.addData("Byte 7", pixy.read8(7));
            telemetry.addData("Byte 8", pixy.read8(8));
            telemetry.addData("Byte 9", pixy.read8(9));
            telemetry.addData("Byte 10", pixy.read8(10));
            telemetry.addData("Byte 11", pixy.read8(11));
            telemetry.addData("Byte 12", pixy.read8(12));
            telemetry.addData("Byte 13", pixy.read8(13));
            telemetry.update();
        }
    }
}