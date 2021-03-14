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

public class 랜딩이_없다면2  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a = 0;
    int r = 0;
    int b = 0;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    DcMotor motor5 = null;
    DcMotor motor6 = null;
    DcMotor motor7 = null;
    DcMotor motor8 = null;

    ModernRoboticsI2cRangeSensor r1 = null;

    void letsmove1(int motorTicks, double motorPower) {
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorTicks);
        motor1.setPower(motorPower);
    }

    void letsmove2(int motorTicks, double motorPower) {
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(motorTicks);
        motor2.setPower(motorPower);
    }

    void letsmove3(int motorTicks, double motorPower) {
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setTargetPosition(motorTicks);
        motor3.setPower(motorPower);
    }

    void letsmove4(int motorTicks, double motorPower) {
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setTargetPosition(motorTicks);
        motor4.setPower(motorPower);
    }

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
            motor7 = hardwareMap.dcMotor.get("m7");
            motor8 = hardwareMap.dcMotor.get("m8");

            waitForStart();
            c1.enableLed(true);

            while (opModeIsActive()) {

                if(opModeIsActive())
                    //공,큐브 앞으로 왼쪽 사선이동
                    //  letsmove1(0, 1.0);
                    // letsmove2(4354, 0.0);
                    //letsmove3(-5406, 0.0);
                    //letsmove4(0, -1.0);
                    motor1.setPower(0.0);
                    motor2.setPower(1.0);
                    motor3.setPower(-1.0);
                    motor4.setPower(0.0);
                    ElapsedTime aTime = new ElapsedTime();
                    aTime.reset();
                    while(aTime.time()<1.2) {}
                    motor1.setPower(0.0);
                    motor2.setPower(0.0);
                    motor3.setPower(0.0);
                    motor4.setPower(0.0);
                    ElapsedTime bTime = new ElapsedTime();
                    bTime.reset();
                    while (bTime.time() < 1) {}
                    bTime.reset();

                    while (bTime.time() < 10 && a == 0) {
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        red1 = c1.red();
                        blue1 = c1.blue();
                        green1 = c1.green();
                        ElapsedTime cTime = new ElapsedTime();
                        cTime.reset();
                        while (cTime.time() < 1) {}

                        if (green1 > blue1 && red1 > blue1) {            //노란색이면 전진
                        //letsmove1(4480, 0.1);
                        //letsmove2(4480, 0.1);
                        //letsmove3(4480, -0.1);
                        //letsmove4(4480, -0.1);
                            motor1.setPower(0.3);
                            motor2.setPower(0.3);
                            motor3.setPower(-0.3);
                            motor4.setPower(-0.3);
                            ElapsedTime dTime = new ElapsedTime();
                            dTime.reset();
                            while (dTime.time() < 1) {}//////////////확인!!!!!
                            b=1;
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime eTime = new ElapsedTime();
                            eTime.reset();
                            while (eTime.time() < 1) {}

                            if(b==1&&a==0&&green1 > blue1 && red1 > blue1){
                                motor1.setPower(0.3);
                                motor2.setPower(0.3);
                                motor3.setPower(0.3);
                                motor4.setPower(0.3);
                                ElapsedTime fTime = new ElapsedTime();
                                fTime.reset();
                                while (fTime.time() < 0.3) {}
                                motor7.setPower(0.0);

                            }
                        }

                        else {
                            //  letsmove1(2240, 0.5);  //노란색 아니면 오른쪽수직이동(2바퀴)
                            // letsmove2(2240, -0.5);
                            // letsmove3(2240, 0.5);
                            // letsmove4(2240, -0.5);
                            motor1.setPower(0.3);
                            motor2.setPower(-0.3);
                            motor3.setPower(0.3);
                            motor4.setPower(-0.3);
                            ElapsedTime gTime = new ElapsedTime();
                            gTime.reset();
                            while (gTime.time() < 0.8) {
                            }
                            a++;
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime hTime = new ElapsedTime();
                            hTime.reset();
                            while (hTime.time() < 1) {
                            }
                        }
                    }
                }

            }
            motor1.setPower(0.0);
            motor2.setPower(0.0);
            motor3.setPower(0.0);
            motor4.setPower(0.0);
        }
    }
