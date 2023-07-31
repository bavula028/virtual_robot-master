package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.simple_rr_example.MecBot;


@Autonomous(name = "AutonomousSquare")
public class AutoOpSquare extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //Call Motors
        DcMotor backLeft = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor backRight = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor frontRight = hardwareMap.dcMotor.get("front_right_motor");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //WHY ARE THESE MOTORS REVERSED???

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu"); //New IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //New Parameters
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "hello";
        imu.initialize(parameters);
        DistanceSensor back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");


        waitForStart();

        while (opModeIsActive()) {
            while (back_distance.getDistance(DistanceUnit.CM) >= 65) {
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            double length = backLeft.getCurrentPosition();

            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180.0 / Math.PI <= 90) {
                backLeft.setPower(-1);
                frontLeft.setPower(-1);
                backRight.setPower(1);
                frontRight.setPower(1);

                telemetry.addData("imu value: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();

            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //THIS WAS ALL WRITTEN FROM ORIGINAL CODE

            while (backLeft.getCurrentPosition() >= length) {
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI <= 175) {
                //Compares the current angle of the robot to the initial angle from which it started

                backLeft.setPower(-1);
                frontLeft.setPower(-1);
                backRight.setPower(1);
                frontRight.setPower(1);
                //TURNS RIGHT

                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData("", length);
                telemetry.update();
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (backLeft.getCurrentPosition() >= length * 2) {
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
            } //Makes it stop, I guess.

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI <= -85 ||
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI >= -90) {
                backLeft.setPower(-1);
                frontLeft.setPower(-1);
                backRight.setPower(1);
                frontRight.setPower(1);

                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();

            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (backLeft.getCurrentPosition() <= 2 * length) {
                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontLeft.setPower(-1);
                frontRight.setPower(-1);
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI >= -180 ||
                    imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI <= -170){
                backLeft.setPower(-1);
                frontLeft.setPower(-1);
                backRight.setPower(1);
                frontRight.setPower(1);

                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();

            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

}