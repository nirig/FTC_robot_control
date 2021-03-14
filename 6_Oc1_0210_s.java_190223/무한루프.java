package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Time;

@Autonomous(name="Auto1", group="teamcode")
//@Disabled

public class 무한루프  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a = 0;
    int b = 0;
    int r = 0;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    DcMotor motor5 = null;
    DcMotor motor6 = null;

    ModernRoboticsI2cRangeSensor r1 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        {

            c1 = hardwareMap.colorSensor.get("c1");
            c1.setI2cAddress(I2cAddr.create7bit(0x26));

            r1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r1");

            motor1 = hardwareMap.dcMotor.get("m1");
            motor2 = hardwareMap.dcMotor.get("m2");
            motor3 = hardwareMap.dcMotor.get("m3");
            motor4 = hardwareMap.dcMotor.get("m4");
            motor5 = hardwareMap.dcMotor.get("m5");
            motor6 = hardwareMap.dcMotor.get("m6");

            waitForStart();
            c1.enableLed(true);

            while (opModeIsActive()) {

                if (r1.getDistance(DistanceUnit.CM) < 5) {     //랜딩
                    motor5.setPower(0.0);
                    motor6.setPower(0.0);
                    r = 1;
                } else if (r1.getDistance(DistanceUnit.CM) >= 5) {
                    motor5.setPower(0.1);
                    motor6.setPower(-0.1);
                    ElapsedTime wowTime = new ElapsedTime();
                    wowTime.reset();
                    while (wowTime.time() < 0.1) {
                    }
                }
            }
        }
    }
}