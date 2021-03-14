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

@Autonomous(name="encoder", group="teamcode")
@Disabled

public class encoder  extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motor1 = null;

    void letsmove1(int motorTicks, double motorPower) {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorTicks);
        motor1.setPower(motorPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        {


            motor1 = hardwareMap.dcMotor.get("m1");

            waitForStart();


            while (opModeIsActive()) {
                letsmove1(1120, 1.0);
                motor1.setPower(0.0);
            }
        }
    }
}
