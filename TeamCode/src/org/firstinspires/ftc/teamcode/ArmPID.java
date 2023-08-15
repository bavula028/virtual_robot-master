package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Arm PID")
public class ArmPID extends LinearOpMode{

    /* This is a PID
        DO NOT MAKE ANY MORE CHANGES
        It is already perfect.
     */

    private double kP = 1;  //Proportional gain
    private double kI = 0;  //Integral gain
    private double kD = -0.1;  //Derivative gain
    private double proportional = 0;
    private double integral = 0;
    private double derivative = 0;
    private double setPoint = 2000;
    private double processVariable = 0;
    private double time = 0;    //Set it equal to getRunTime() later while (opModeIsActive)
    private double error = 0;
    private double previousError = 0;
    private double currentPosition = 0;
    private double output = 0;



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
