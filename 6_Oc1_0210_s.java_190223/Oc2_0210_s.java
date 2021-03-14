package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;



@Autonomous(name = "Color", group = "MRI")
@Disabled
public class Oc2_0210_s extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a=0;

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

        while (opModeIsActive()) {
            if (a == 0) {
                motor1.setPower(1.0);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(-1.0);
                ElapsedTime aTime = new ElapsedTime();
                aTime.reset();
                while (aTime.time() < 2) {
                }
            }
            motor1.setPower(0.0);
            motor2.setPower(0.0);
            motor3.setPower(0.0);
            motor4.setPower(0.0);
            ElapsedTime aTime = new ElapsedTime();
            aTime.reset();
            while(aTime.time()<3) {}

            while(aTime.time()<10 && a==0) {
                motor1.setPower(0.0);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(0.0);
                red1 = c1.red();
                blue1 = c1.blue();
                green1 = c1.green();
                ElapsedTime bTime = new ElapsedTime();
                bTime.reset();
                while (bTime.time() < 1) {
                }

                if (green1 > blue1 && red1 > blue1) {
                    motor1.setPower(0.1);
                    motor2.setPower(0.1);
                    motor3.setPower(-0.1);
                    motor4.setPower(-0.1);
                    ElapsedTime cTime = new ElapsedTime();
                    cTime.reset();
                    while (cTime.time() < 1) {
                    }
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    motor4.setPower(0.0);
                    a=1;
                    break;
                }
                else {
                    motor1.setPower(0.5);
                    motor2.setPower(-0.5);
                    motor3.setPower(0.5);
                    motor4.setPower(-0.5);
                    ElapsedTime cTime = new ElapsedTime();
                    cTime.reset();
                    while (cTime.time() < 1) {
                    }
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    motor4.setPower(0.0);
                    if (a==1) {
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        break;
                    }
                }
            }

        }
        if (a==1) {
            motor1.setPower(0.0);
            motor2.setPower(0.0);
            motor3.setPower(0.0);
            motor4.setPower(0.0);
        }
    }
}
