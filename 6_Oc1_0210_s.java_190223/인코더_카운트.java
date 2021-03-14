package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="encoder", group="teamcode")
@Disabled
public class 인코더_카운트 extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motor1 = null;
    int a = 0;
    public void movemotor1 (int motorsTicks,double motorPower){
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(motorsTicks);
        motor1.setPower(motorPower);
    }
    @Override
    public void runOpMode(){

    motor1 = hardwareMap.dcMotor.get("m1");
    telemetry.addData("Path0", "%7d", motor1.getCurrentPosition());
    telemetry.update();



    waitForStart();
    telemetry.addData("status", "Program Started");
    telemetry.update();
    while (opModeIsActive()&&a==0){
        telemetry.addData("Now Running at", "%7d :%7d",motor1.getCurrentPosition(),
                motor1.getCurrentPosition());
        telemetry.update();
        a=1;
        if(a==1){
            motor1.setPower(1.0);
            ElapsedTime dTime = new ElapsedTime();
            dTime.reset();
            while(dTime.time()<4) {}
            telemetry.addData("Now Running at", "%7d :%7d", motor1.getCurrentPosition(),motor1.getCurrentPosition());
            telemetry.update();
            ElapsedTime aTime = new ElapsedTime();
            aTime.reset();
            while(aTime.time()<10) {}
            break;
        }
        else{
            motor1.setPower(0.0);
        }
        motor1.setPower(0.0);

        telemetry.addData("Now Running at", "%7d :%7d", motor1.getCurrentPosition(),motor1.getCurrentPosition());
        telemetry.update();
    }
}
}
