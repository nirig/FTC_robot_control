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
@Disabled

public class 무한루프2  extends LinearOpMode {

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

            while (opModeIsActive())
            {
                if (r1.getDistance(DistanceUnit.CM) < 5) //랜딩
                {
                    motor5.setPower(0.0);
                    motor6.setPower(0.0);
                    r = 1;
                    if (r == 1)
                    {
                        motor1.setPower(0.3);//오른쪽 수직이동
                        motor2.setPower(-0.3);
                        motor3.setPower(0.3);
                        motor4.setPower(-0.3);
                        ElapsedTime aTime = new ElapsedTime();
                        aTime.reset();
                        while (aTime.time() < 1.1) { }
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime bTime = new ElapsedTime();
                        bTime.reset();
                        while (bTime.time() < 0.2) { }
                        ///////////////////////////////////////////////////
                        motor1.setPower(1.0);//사선이동(공,큐브앞까지)
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(-1.0);
                        ElapsedTime cTime = new ElapsedTime();
                        cTime.reset();
                        while (cTime.time() < 1.1) { }
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime dTime = new ElapsedTime();
                        dTime.reset();
                        while (dTime.time() < 0.2) { }
                        dTime.reset();
                        ////////////////////////////////////////////////
                        while (dTime.time() < 10 && a == 0) // 컬러센서세팅
                        {
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            red1 = c1.red();
                            blue1 = c1.blue();
                            green1 = c1.green();
                            ElapsedTime eTime = new ElapsedTime();
                            eTime.reset();
                            while (eTime.time() < 1) {}

                            if (green1 > blue1 && red1 > blue1)  //노란색이면 전진
                            {
                                motor1.setPower(0.5);
                                motor2.setPower(0.5);
                                motor3.setPower(-0.5);
                                motor4.setPower(-0.5);
                                ElapsedTime fTime = new ElapsedTime();
                                fTime.reset();
                                while (fTime.time() < 1) { }
                                motor1.setPower(0.0);
                                motor2.setPower(0.0);
                                motor3.setPower(0.0);
                                motor4.setPower(0.0);
                                ElapsedTime gTime = new ElapsedTime();
                                gTime.reset();
                                while (gTime.time() < 1) {}
                            }
                        }
                    }
                    else if (r1.getDistance(DistanceUnit.CM) >= 5)
                    {
                        motor5.setPower(1.0);
                        motor6.setPower(-1.0);
                        ElapsedTime wowTime = new ElapsedTime();
                        wowTime.reset();
                        while (wowTime.time() < 0.1) { }
                    }
                }
            }
        }
    }
}