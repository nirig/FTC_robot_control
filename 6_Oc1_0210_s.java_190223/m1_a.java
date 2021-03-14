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

@Autonomous(name="Auto1", group="teamcode")
@Disabled

public class m1_a  extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a = 0;
    int b = 0;
    int r = 0;

    DcMotor motor1 = null;


    ModernRoboticsI2cRangeSensor r1 = null;

    void letsmove1(int motorTicks, double motorPower) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorTicks);
        motor1.setPower(motorPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        c1 = hardwareMap.colorSensor.get("c1");
        c1.setI2cAddress(I2cAddr.create7bit(0x26));

        r1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r1");

        motor1 = hardwareMap.dcMotor.get("m1");


        waitForStart();
        c1.enableLed(true);

        while (opModeIsActive()) {

            if (r1.getDistance(DistanceUnit.CM) > 5) {     //랜딩
                motor1.setPower(0.5);
            }
            else {
                motor1.setPower(0.0);
                ElapsedTime aTime = new ElapsedTime();
                aTime.reset();
                while (aTime.time() < 1) {}
                r++;

                if (r > 0 && a==0 && b==0) {
                    letsmove1(2240, -0.1);// 옆으로 이동 2
                    ElapsedTime zTime = new ElapsedTime();
                    zTime.reset();
                    while (zTime.time() < 5) {}

                    motor1.setPower(0.0);
                    ElapsedTime bTime = new ElapsedTime();
                    bTime.reset();
                    while (bTime.time() < 1) {}

                    letsmove1(2240, -1.0);//앞으로 2바퀴
                    ElapsedTime yTime = new ElapsedTime();
                    yTime.reset();
                    while (yTime.time() < 5) {}

                    motor1.setPower(0.0);
                    ElapsedTime cTime = new ElapsedTime();
                    cTime.reset();
                    while (cTime.time() < 1) {}

                    letsmove1(4480, -0.1);//공, 큐브 사선이동
                    ElapsedTime xTime = new ElapsedTime();
                    xTime.reset();
                    while (xTime.time() < 5) {}

                    motor1.setPower(0.0);
                    ElapsedTime dTime = new ElapsedTime();
                    dTime.reset();
                    while (dTime.time() < 3) {}

                    if (red1 > blue1 && green1 >blue1 && a == 0) {
                        letsmove1(11200, -0.1);//노란큐브치기_작은 전진 10바퀴
                        ElapsedTime wTime = new ElapsedTime();
                        wTime.reset();
                        while (wTime.time() < 5) {}

                        motor1.setPower(0.0);
                       break;
                    } else {
                        letsmove1(2240, 0.1);// 옆으로 이동 2
                        ElapsedTime uTime = new ElapsedTime();
                        uTime.reset();
                        while (uTime.time() < 5) {}

                        motor1.setPower(0.0);
                        ElapsedTime fTime = new ElapsedTime();
                        fTime.reset();
                        while (fTime.time() < 1) {}
                        b++;
                    }
                    motor1.setPower(0.0);
                }
                motor1.setPower(0.0);
            }
            motor1.setPower(0.0);
        }
        motor1.setPower(0.0);
    }
}