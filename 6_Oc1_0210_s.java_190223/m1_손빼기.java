package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="mission1a", group="teamcode")
@Disabled

public class m1_손빼기 extends LinearOpMode{
    private ElapsedTime runtime =new ElapsedTime();

    int a=0;

    DcMotor motor1 = null;
    DcMotor motor2 = null;
    DcMotor motor3 = null;
    DcMotor motor4 = null;


    public void movemotor1 (int motorsTicks,double motorPower){
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorsTicks);
        motor1.setPower(motorPower);
    }

    public void movemotor2 (int motorsTicks,double motorPower){
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setTargetPosition(motorsTicks);
        motor2.setPower(motorPower);
    }

    public void movemotor3 (int motorsTicks,double motorPower){
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setTargetPosition(motorsTicks);
        motor3.setPower(motorPower);
    }
    public void movemotor4 (int motorsTicks,double motorPower){
        motor4.setDirection(DcMotor.Direction.REVERSE);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setTargetPosition(motorsTicks);
        motor4.setPower(motorPower);
    }

    @Override
    public void runOpMode(){
        motor1=hardwareMap.dcMotor.get("m1");
        motor2=hardwareMap.dcMotor.get("m2");
        motor3=hardwareMap.dcMotor.get("m3");
        motor4=hardwareMap.dcMotor.get("m4");


        telemetry.addData("Path0", "%7d :&7d :&7d :&7d",
                motor1.getCurrentPosition(),
                motor2.getCurrentPosition(),
                motor3.getCurrentPosition(),
                motor4.getCurrentPosition());
        telemetry.update();

        waitForStart();
        telemetry.addData("status", "Program Started");
        telemetry.update();

        while (opModeIsActive() && a==0){
            telemetry.addData("Now Running at", "%7d :%7d",
                    motor1.getCurrentPosition(),
                    motor2.getCurrentPosition(),
                    motor3.getCurrentPosition(),
                    motor4.getCurrentPosition());
            telemetry.update();
            a=1;
            if(a==1){
                 motor1.setPower(0.7);
                motor2.setPower(-0.7);
                motor3.setPower(0.7);
                motor4.setPower(-0.7);
                ElapsedTime dTime = new ElapsedTime();
                dTime.reset();
                while(dTime.time()<1.2) {}
                telemetry.addData("Now Running at", "%7d :%7d :%7d :%7d",
                        motor1.getCurrentPosition(),
                        motor2.getCurrentPosition(),
                        motor3.getCurrentPosition(),
                        motor4.getCurrentPosition());
                telemetry.update();
                motor1.setPower(0.0);
                motor2.setPower(0.0);
                motor3.setPower(0.0);
                motor4.setPower(0.0);
            }
        }

    }
}
