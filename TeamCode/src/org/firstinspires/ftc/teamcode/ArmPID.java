package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Arm PID")
public class ArmPID extends PIDclass{
    //This class is only applicable for Arm Bot

    double kP = 1;  //Proportional gain
    double kI = 0;  //Integral gain
    double kD = -0.1;  //Derivative gain
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double setPoint = 1500;
    double processVariable = 0;
    double time = 0;    //Set it equal to getRunTime() later while (opModeIsActive)
    double error = 0;
    double previousError = 0;
    double currentPosition = 0;
    double output = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");

        waitForStart();

        while(opModeIsActive()){
            time = getRuntime();
            currentPosition = arm.getCurrentPosition();


            error = setPoint - currentPosition;
            proportional = error;
            integral = integral + error * time;
            derivative = (error - previousError) / time;
            output =  kP * proportional + kI * integral + kD * derivative;

            telemetry.addData("time:", time);
            telemetry.addData("arm position:", currentPosition);
            telemetry.addData("error:", error);
            telemetry.addData("proportional:", proportional);
            telemetry.addData("integral:", integral);
            telemetry.addData("derivative:", derivative);
            telemetry.addData("output:", output);
            telemetry.update();

            telemetry.update();

            if (gamepad1.left_bumper){
                while (arm.getCurrentPosition() < output){
                    arm.setPower(1);
                }
                arm.setPower(0);
            }

        }

    }
}
