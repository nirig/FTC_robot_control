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

@Autonomous
@Disabled

public class mission2  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor c1;
    int red1;
    int blue1;
    int green1;
    int a = 0;
    int b = 0;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    DcMotor motor5 = null;
    DcMotor motor6 = null;
    DcMotor motor8 = null;

    ModernRoboticsI2cRangeSensor r1 = null;

    void letsmove1(int motorTicks, double motorPower) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorTicks);
        motor1.setPower(motorPower);
    }
    void letsmove2(int motorTicks, double motorPower) {
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(motorTicks);
        motor2.setPower(motorPower);
    }
    void letsmove3(int motorTicks, double motorPower) {
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setTargetPosition(motorTicks);
        motor3.setPower(motorPower);
    }
    void letsmove4(int motorTicks, double motorPower) {
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setTargetPosition(motorTicks);
        motor4.setPower(motorPower);
    }
    void letsmove5(int motorTicks, double motorPower) {
        motor5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor5.setTargetPosition(motorTicks);
        motor5.setPower(motorPower);
    }
    void letsmove6(int motorTicks, double motorPower) {
        motor6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor6.setTargetPosition(motorTicks);
        motor6.setPower(motorPower);
    }
    void letsmove8(int motorTicks, double motorPower) {
        motor8.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor8.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor8.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor8.setTargetPosition(motorTicks);
        motor8.setPower(motorPower);
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
            motor8 = hardwareMap.dcMotor.get("m8");

            waitForStart();
            c1.enableLed(true);

            while (opModeIsActive()) {

                if (r1.getDistance(DistanceUnit.CM)>5 ) {     //랜딩
                    motor5.setPower(1.0);
                    motor6.setPower(-1.0);
                }
                else {
                    motor5.setPower(0.0);
                    motor6.setPower(0.0);
                }

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
                while (aTime.time() < 1) {}

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
                while (bTime.time() < 1) {}

                letsmove5(6720, -0.5);  //팔리니어 내리기(6바퀴)
                letsmove6(6720, 0.5);
                motor5.setPower(0.0);
                motor6.setPower(0.0);

////////////////////////////////////////////////////////////////////////////

                if (a == 0 && b == 1) {                                  //공,큐브 앞으로 왼쪽 사선이동
                    letsmove1(4480,1.0);
                    letsmove2(4480,0.0);
                    letsmove3(4480,0.0);
                    letsmove4(4480,-1.0);
                    //motor1.setPower(1.0);
                    //motor2.setPower(0.0);
                    //motor3.setPower(0.0);
                    //motor4.setPower(-1.0);
                    //ElapsedTime cTime = new ElapsedTime();
                    //cTime.reset();
                    //while (cTime.time() < 2) {}
                }

                motor1.setPower(0.0);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(0.0);
                ElapsedTime dTime = new ElapsedTime();
                dTime.reset();
                while(dTime.time()<3) {}

                while(dTime.time()<10 && a==0 && b == 1) {
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

                    if (green1 > blue1 && red1 > blue1) {            //노란색이면 전진
                        letsmove1(4480,0.1);
                        letsmove2(4480,0.1);
                        letsmove3(4480,-0.1);
                        letsmove4(4480,-0.1);
                        //motor1.setPower(0.1);
                        //motor2.setPower(0.1);
                        //motor3.setPower(-0.1);
                        //motor4.setPower(-0.1);
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime fTime = new ElapsedTime();
                        fTime.reset();
                        while (fTime.time() < 1) {}
                        a++;
                        b = 1;

                        if (b == 1) {
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime hTime = new ElapsedTime();
                            hTime.reset();
                            while (hTime.time() < 1) {}
                            break;
                        }
//////////////////////////////////////////////////////////////////////////////////////////////////
                        if (b == 1 && a ==1) {
                            letsmove1(2240, 0.5);  //우제자리회전(2바퀴)
                            letsmove2(2240, 0.5);
                            letsmove3(2240, 0.5);
                            letsmove4(2240, 0.5);
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime iTime = new ElapsedTime();
                            iTime.reset();
                            while (iTime.time() < 1) {}
                            letsmove1(2240, 0.5);  //전진(2바퀴)
                            letsmove2(2240, 0.5);
                            letsmove3(2240, -0.5);
                            letsmove4(2240, -0.5);
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime jTime = new ElapsedTime();
                            jTime.reset();
                            while (jTime.time() < 1) {}
                            letsmove8(7840,1.0);//팀표기물 놓기_돌돌이 7바퀴
                            motor8.setPower(0.0);
                            break;
                        }
                        else if (b == 1 && a ==2) {
                            letsmove1(2240, 0.5);  //전진(2바퀴)
                            letsmove2(2240, 0.5);
                            letsmove3(2240, -0.5);
                            letsmove4(2240, -0.5);
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime iTime = new ElapsedTime();
                            iTime.reset();
                            while (iTime.time() < 1) {}
                            letsmove8(7840,1.0);//팀표기물 놓기_돌돌이 7바퀴
                            motor8.setPower(0.0);
                            break;
                        }
                        else if (b == 1 && a ==3) {
                            letsmove1(2240, -0.5);  //좌제자리회전(2바퀴)
                            letsmove2(2240, -0.5);
                            letsmove3(2240, -0.5);
                            letsmove4(2240, -0.5);
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime iTime = new ElapsedTime();
                            iTime.reset();
                            while (iTime.time() < 1) {}
                            letsmove1(2240, 0.5);  //전진(2바퀴)
                            letsmove2(2240, 0.5);
                            letsmove3(2240, -0.5);
                            letsmove4(2240, -0.5);
                            motor1.setPower(0.0);
                            motor2.setPower(0.0);
                            motor3.setPower(0.0);
                            motor4.setPower(0.0);
                            ElapsedTime jTime = new ElapsedTime();
                            jTime.reset();
                            while (jTime.time() < 1) {}
                            letsmove8(7840,1.0);//팀표기물 놓기_돌돌이 7바퀴
                            motor8.setPower(0.0);
                            break;
                        }
                    }
///////////////////////////////////////////////////////////////////////////////
                    else {
                        letsmove1(2240, 0.5);  //노란색 아니면 오른쪽수직이동(2바퀴)
                        letsmove2(2240, -0.5);
                        letsmove3(2240, 0.5);
                        letsmove4(2240, -0.5);
                        //motor1.setPower(0.5);
                        //motor2.setPower(-0.5);
                        //motor3.setPower(0.5);
                        //motor4.setPower(-0.5);
                        motor1.setPower(0.0);
                        motor2.setPower(0.0);
                        motor3.setPower(0.0);
                        motor4.setPower(0.0);
                        ElapsedTime gTime = new ElapsedTime();
                        gTime.reset();
                        while (gTime.time() < 1) {}
                        a++;
                    }
                }
            }
        }
            if (b == 1) {
                motor1.setPower(0.0);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(0.0);
                ElapsedTime hTime = new ElapsedTime();
                hTime.reset();
                while (hTime.time() < 1) {}
            }
    }
}

