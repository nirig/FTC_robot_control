package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;



@TeleOp(name = "Color", group = "MRI")
@Disabled
public class Oc1_0210_s extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;

    @Override
    public void runOpMode() throws InterruptedException {

        c1 = hardwareMap.colorSensor.get("c1");
        c1.setI2cAddress(I2cAddr.create7bit(0x26));

        motor1 = hardwareMap.dcMotor.get("m1");
        motor2 = hardwareMap.dcMotor.get("m2");
        motor3 = hardwareMap.dcMotor.get("m3");
        motor4 = hardwareMap.dcMotor.get("m4");


        waitForStart();
        c1.enableLed(true);
        red1=c1.red();
        blue1=c1.blue();
        green1=c1.green();


        while (opModeIsActive()) {

                ElapsedTime aTime = new ElapsedTime();//(랜딩후) 공, 큐브 3개가 나란히 놓여있는 곳으로 이동_사선이동
                aTime.reset();
                while(aTime.time()<2){}
                motor1.setPower(0.15);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(-0.15);
                if (green1>blue1) {//(초록색이 파란색보다 많으면) 노란색이면 멈추고 앞으로 이동하여 쳐냄.
                    ElapsedTime bTime = new ElapsedTime();
                    bTime.reset();
                    while(bTime.time()<1){}
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    motor4.setPower(0.0);
                    ElapsedTime cTime = new ElapsedTime();
                    cTime.reset();
                    while(cTime.time()<1){}
                    motor1.setPower(0.15 * 0.5);
                    motor2.setPower(0.15 * 0.5);
                    motor3.setPower(-0.15 * 0.5);
                    motor4.setPower(-0.15 * 0.5);
                }
                else if (blue1>green1) {//노란색이 아니면 계속 옆으로 이동
                    motor1.setPower(0.15 * 0.5);
                    motor2.setPower(-0.15 * 0.5);
                    motor3.setPower(0.15 * 0.5);
                    motor4.setPower(-0.15 * 0.5);
                }
            }
            }
        }
