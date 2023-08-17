package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

@TeleOp(name = "Rotation PID")
public class RotationPID extends PID{
    //This class is applicable for Arm Bot and Mecanum Bot.

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double setPoint = 90;
    double processVariable = 0;
    double time = 0;
    double error = 0;
    double currentAngle = 0;
    double previousError = 0;

    double output = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = " ";
        parameters.loggingEnabled = true;
        parameters.loggingTag = " ";

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            time = getRuntime();

            double getAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

            currentAngle = getAngle * 180 / Math.PI;
            //Why is it showing an error?
            //When running, it terminates because of this line.
            //Returns an exception


            error = setPoint -  currentAngle;


            System.out.println(currentAngle);


            proportional = error;
            integral = integral + error * time;
            derivative = (error - previousError) / time;
            output = kP + proportional + kI * integral + kD * derivative;

            telemetry.addData("time:", time);
            telemetry.addData("current angle:", currentAngle);
            telemetry.addData("error:", error);
            telemetry.addData("proportional:", proportional);
            telemetry.addData("integral:", integral);
            telemetry.addData("derivative:", derivative);
            telemetry.addData("output:", output);
            telemetry.update();

            if (gamepad1.left_bumper) {

                while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI < output){
                    bl.setPower(-1);
                    br.setPower(1);
                    fl.setPower(-1);
                    fr.setPower(1);
                }
            }
        }





    }
}
