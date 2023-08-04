package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Practice Autonomous Mecanum")
public class PracticeAutonomousMecanum extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        //Call Motors
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        //Call IMU
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Call Parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = " ";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "hello";

        imu.initialize(parameters);
        DistanceSensor front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            while(front_distance.getDistance(DistanceUnit.CM) >= 68.5){
                bl.setPower(1);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(1);
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            double length = bl.getCurrentPosition();

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI < 90){

                //right wheels go forward, left wheels go back
                //turning left
                bl.setPower(-1);
                br.setPower(1);
                fl.setPower(-1);
                fr.setPower(1);

                //Add that data
                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();

            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while(front_distance.getDistance(DistanceUnit.CM) >= 68.5){
                bl.setPower(1);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(1);
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI <= 177){
                bl.setPower(-1);
                br.setPower(1);
                fl.setPower(-1);
                fr.setPower(1);

                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (front_distance.getDistance(DistanceUnit.CM) > 68.5){
                bl.setPower(1);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(1);
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI < -90 ||
            imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI > -85){
                bl.setPower(-1);
                br.setPower(1);
                fl.setPower(-1);
                fr.setPower(1);

                telemetry.addData("imu value:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle * 180 / Math.PI);
                telemetry.addData(" ", length);
                telemetry.update();
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (back_distance.getDistance(DistanceUnit.CM) >= 2){
                bl.setPower(-1);
                br.setPower(-1);
                fl.setPower(-1);
                fr.setPower(-1);
            }

            requestOpModeStop();

        }



    }
}