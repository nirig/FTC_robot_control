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

public class mission1  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a = 0;
    int r = 0;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    DcMotor motor5 = null;
    DcMotor motor6 = null;

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

    void letsmove5(int motorTicks, double motorPower) {
        motor5.setDirection(DcMotor.Direction.REVERSE);
        motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor5.setTargetPosition(motorTicks);
        motor5.setPower(motorPower);
    }

    void letsmove6(int motorTicks, double motorPower) {
        motor6.setDirection(DcMotor.Direction.REVERSE);
        motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor6.setTargetPosition(motorTicks);
        motor6.setPower(motorPower);
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

            waitForStart();
            c1.enableLed(true);

            while (opModeIsActive()) {

                if (r1.getDistance(DistanceUnit.CM) > 5) {     //랜딩
                    motor5.setPower(1.0);
                    motor6.setPower(-1.0);
                } else {
                    motor5.setPower(0.0);
                    motor6.setPower(0.0);
                    r = 1;
                    ElapsedTime wowTime = new ElapsedTime();
                    wowTime.reset();
                    while (wowTime.time() < 1) {
                    }

                    if (r == 1) {
                        letsmove1(2240, 0.5);  //손잡이 빼고(오른쪽 수직이동_2바퀴)
                        letsmove2(2240, -0.5);
                        letsmove3(2240, 0.5);
                        letsmove4(2240, -0.5);
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime aTime = new ElapsedTime();
                        aTime.reset();
                        while (aTime.time() < 1) {
                        }

                        letsmove1(2240, 0.1);  //조금 앞으로 나오기(앞으로 직진_2바퀴)
                        letsmove2(2240, 0.1);
                        letsmove3(2240, -0.1);
                        letsmove4(2240, -0.1);
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime bTime = new ElapsedTime();
                        bTime.reset();
                        while (bTime.time() < 1) {
                        }


                        //공,큐브 앞으로 왼쪽 사선이동
                        letsmove1(0, 1.0);
                        letsmove2(4354, 0.0);
                        letsmove3(-5406, 0.0);
                        letsmove4(0, -1.0);

                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime dTime = new ElapsedTime();
                        dTime.reset();
                        while (dTime.time() < 3) {
                        }
                        dTime.reset();

                        while (dTime.time() < 10 && a == 0) {
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            red1 = c1.red();
                            blue1 = c1.blue();
                            green1 = c1.green();
                            ElapsedTime eTime = new ElapsedTime();
                            eTime.reset();
                            while (eTime.time() < 1) {
                            }

                            if (green1 > blue1 && red1 > blue1) {            //노란색이면 전진
                                letsmove1(4480, 0.1);
                                letsmove2(4480, 0.1);
                                letsmove3(4480, -0.1);
                                letsmove4(4480, -0.1);
                                motor1.setPower(0.0);
                                motor2.setPower(0.0);
                                motor3.setPower(0.0);
                                motor4.setPower(0.0);
                                ElapsedTime fTime = new ElapsedTime();
                                fTime.reset();
                                while (fTime.time() < 1) {
                                }
                                letsmove1(6720, 0.1);  //검정 울타리에 걸치기_6바퀴전진
                                letsmove2(6720, 0.1);
                                letsmove3(6720, -0.1);
                                letsmove4(6720, -0.1);
                                a++;
                                motor1.setPower(0.0);
                                motor2.setPower(0.0);
                                motor3.setPower(0.0);
                                motor4.setPower(0.0);
                            } else {
                                letsmove1(2240, 0.5);  //노란색 아니면 오른쪽수직이동(2바퀴)
                                letsmove2(2240, -0.5);
                                letsmove3(2240, 0.5);
                                letsmove4(2240, -0.5);
                                motor1.setPower(0.0);
                                motor2.setPower(0.0);
                                motor3.setPower(0.0);
                                motor4.setPower(0.0);
                                ElapsedTime gTime = new ElapsedTime();
                                gTime.reset();
                                while (gTime.time() < 1) {
                                }

                                if (a > 0) {
                                    motor1.setPower(0.0);
                                    motor2.setPower(0.0);
                                    motor3.setPower(0.0);
                                    motor4.setPower(0.0);
                                    break;
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
        }
    }
}
