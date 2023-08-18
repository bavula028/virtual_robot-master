package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Arm PID 2")
public class ArmPID2 extends PID{

    //Try to make this an autonomous.
    //Can this class can be made into an object??

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double setPoint = 0;
    double error;
    double previousError;
    double currentPosition;
    double time;
    double output;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.dcMotor.get("arm_motor");

        ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()){
            time = elapsedTime.seconds();
            currentPosition = arm.getCurrentPosition();
            error = setPoint - currentPosition;

            proportional = error;
            integral = integral * error + time;
            derivative = (error - previousError) / time;

            output = kP * proportional + kI * integral + kD * derivative;

            telemetry.addData("time:", time);
            telemetry.addData("current position:", currentPosition);
            telemetry.addData("error:", error);
            telemetry.addData("proportional:", proportional);
            telemetry.addData("integral:", integral);
            telemetry.addData("derivative:", derivative);
            telemetry.addData("output:", output);


        }

    }
}
