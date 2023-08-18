package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Arm PID 2")
public class ArmPID2 extends PID{

    //Try to make this an autonomous.
    //Can this class can be made into an object??

    double kP = 0;
    double kI = 0;
    double kD = 1;
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double setPoint = 2000;
    double processVariable = 0;
    double error;
    double previousError;
    double currentPosition;
    double time;
    double output;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");

        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()){
            //time = elapsedTime.seconds();


            //currentPosition is always returning 0, even after the arm extends.
            //Why?

            time = getRuntime();
            currentPosition = arm.getCurrentPosition();
            error = setPoint - currentPosition;

            proportional = error;
            integral = integral * error + time;
            derivative = (error - previousError) / time;

            output = kP * proportional + kI * integral + kD * derivative;

            while (currentPosition < output){
                arm.setPower(1);

                time = getRuntime();
                currentPosition = arm.getCurrentPosition();
                error = setPoint - currentPosition;

                proportional = error;
                integral = integral * error + time;
                derivative = (error - previousError) / time;

                output = kP * proportional + kI * integral + kD * derivative;

                telemetry.addData("time:", getRuntime());
                telemetry.addData("current position:", arm.getCurrentPosition());
                telemetry.addData("error:", error);
                telemetry.addData("proportional:", proportional);
                telemetry.addData("integral:", integral);
                telemetry.addData("derivative:", derivative);
                telemetry.addData("output:", output);
                telemetry.update();

            }

            telemetry.addData("arm current position:", arm.getCurrentPosition());
            telemetry.update();


            //requestOpModeStop();


        }

    }
}
