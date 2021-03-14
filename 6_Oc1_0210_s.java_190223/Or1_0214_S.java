package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Range+Encoder", group = "MRI")
@Disabled
public class Or1_0214_S extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;
    DcMotor motor5 = null;
    DcMotor motor6 = null;

    ModernRoboticsI2cRangeSensor r1 = null;

    void letsmove1(int motorTicks,double motorPower) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void letsmove2(int motorTicks,double motorPower) {
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void letsmove3(int motorTicks,double motorPower) {
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void letsmove4(int motorTicks,double motorPower) {
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        r1 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r1");

        motor1 = hardwareMap.dcMotor.get("m1");
        motor2 = hardwareMap.dcMotor.get("m2");
        motor3 = hardwareMap.dcMotor.get("m3");
        motor4 = hardwareMap.dcMotor.get("m4");
        motor5 = hardwareMap.dcMotor.get("m5");
        motor6 = hardwareMap.dcMotor.get("m6");



        waitForStart();

        while (opModeIsActive()) {

            if (r1.getDistance(DistanceUnit.CM)>5 ) {
                motor5.setPower(1.0);
                motor6.setPower(-1.0);
            }
            else {
                motor5.setPower(0.0);
                motor6.setPower(-0.0);
            }
            letsmove1(2240, 0.5);
            letsmove2(2240, -0.5);
            letsmove3(2240, 0.5);
            letsmove4(2240, -0.5);
            motor1.setPower(0.0);
            motor2.setPower(0.0);
            motor3.setPower(0.0);
            motor4.setPower(0.0);
            ElapsedTime cTime = new ElapsedTime();
            cTime.reset();
            while (cTime.time() < 1) {}

        }
    }
}
