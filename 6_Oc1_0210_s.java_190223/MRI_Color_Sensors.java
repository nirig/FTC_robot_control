package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Color Sensors", group = "MRI")
// @Autonomous(...) is the other common choice
@Disabled
public class MRI_Color_Sensors extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] color1cache;


    I2cDevice color1;

    I2cDeviceSynch color1reader;


    @Override
    public void runOpMode() throws InterruptedException {

        color1 = hardwareMap.i2cDevice.get("m4");


        color1reader = new I2cDeviceSynchImpl(color1, I2cAddr.create8bit(0x26), false);


        color1reader.engage();

        waitForStart();
        color1reader.write8(3, 0);


        while (opModeIsActive()) {
            color1cache = color1reader.read(0x04, 1);


//            telemetry.addData("1 #A", color1cache[0] & 0xFF);
//
//
//            telemetry.addData("3 A", color1reader.getI2cAddress().get8Bit());

        }
    }
}