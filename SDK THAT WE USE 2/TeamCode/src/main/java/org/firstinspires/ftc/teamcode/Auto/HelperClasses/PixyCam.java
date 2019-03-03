package org.firstinspires.ftc.teamcode.Auto.HelperClasses;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public abstract class PixyCam extends FunctionsForAuto {
    //Pixycam variables
    private I2cDeviceSynch pixySide;
    private byte[] pixyData;
    private int mineralPos;

    public void initAll(String name, String chosenOpMode){
        engagePixy();
        telemetry.addLine("PixyCam initialized");
        telemetry.update();
        super.initAll(name,chosenOpMode);
    }

    public void engagePixy(){
        //setting up Pixy to the hardware map
        mineralPos = 0;
        pixySide = hardwareMap.i2cDeviceSynch.get("p1");

        //setting Pixy's I2C Address
        pixySide.setI2cAddress(I2cAddr.create7bit(0x54));

        //setting Pixy's read window. You'll want these exact parameters, and you can reference the
        //SDK Documentation to learn more
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 26,
                I2cDeviceSynch.ReadMode.REPEAT);
        pixySide.setReadWindow(readWindow);

        //required to "turn on" the device
        pixySide.engage();
    }

    public void turnRunAndHitWithPixy(int driveTime){
        setBothPower(-.3);
        timeOne = this.getRuntime();
        timeTwo = this.getRuntime();
        pixyData = pixySide.read(0x51,5);
        while((pixyData[1] < 100 || pixyData[1] > 150) && timeTwo - timeOne < driveTime){
            timeTwo = this.getRuntime();
            telemetry.addData("Num of Blocks that match signature:", pixyData[0]);
            telemetry.addData("X value:", pixyData[1]);
            telemetry.addData("X value:", pixySide.read(0x51,5)[1]);
            telemetry.addData("Y value:", pixyData[2]);
            telemetry.addData("Width:", pixyData[3]);
            telemetry.addData("Height:", pixyData[4]);
            telemetry.update();
            pixyData = pixySide.read(0x51,5);
        }
        setBothPower(0);
        telemetry.addLine("Final Data:");
        telemetry.addData("Num of Blocks that match signature:", pixyData[0]);
        telemetry.addData("X value:", pixyData[1]);
        telemetry.addData("Y value:", pixyData[2]);
        telemetry.addData("Width:", pixyData[3]);
        telemetry.addData("Height:", pixyData[4]);
        telemetry.update();
        rotate(-80,.35,5);
        driveWithEncoders(30, .4, 2);//Drive forward to hit the block
    }

    public void faceAndHitWithPixy(int freedom){

        //Give the pixyCam time to see everything and set a byte array to the values the front pixyCam sees
        pause(.5);
        pixyData = pixySide.read(0x51, 5);

        //Determine if the mineral is left, right, or center for later code
        if (pixyData[1] > 128 + freedom)//If the mineral is more than freedom to the right of the center of the camera
            mineralPos = 1; //Set mineral position to 1 (right)
        else if (pixyData[1] < 128 - freedom)//If the mineral is more than freedom to the left of the center of the camera
            mineralPos = -1; //Set mineral position to -1 (left)
        else //Else, the gold mineral is within freedom distance from the center of the camera
            mineralPos = 0; //Set mineral position to 0 (center)

        //Add telemetry so we know what we're seeing
        telemetry.addData("Num of Blocks that match signature:", pixyData[0]);
        telemetry.addData("X value:", pixyData[1]);
        telemetry.addData("Y value:", pixyData[2]);
        telemetry.addData("Width:", pixyData[3]);
        telemetry.addData("Height:", pixyData[4]);
        telemetry.addData("Mineral Pos:", mineralPos);
        telemetry.addData("Turn degrees:", (128 - pixyData[1]) / 2);
        telemetry.update();

        pause(1);

        //Turn and face the block depending on the block's distance from the center of the camera
        rotate((128 - pixyData[1])/2, .3, 4);

        driveWithEncoders(40,.4,5);//Drive forward and hit the block
    }

    //Getters
    public int getMineralPos(){return mineralPos;}
    public byte[] getPixyData(){return pixyData;}
    public I2cDeviceSynch getPixySide(){return pixySide;}
}