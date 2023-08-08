package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Practice Autonomous Mecanum")
public class ChallengeAutonomous extends LinearOpMode{

        //This is supposed to move forward, go diagonally right forward, and then go diagonally back left.

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");

        bl.setDirection()

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

        public void forwardRightDiagonal(){
            while (front_distance.getDistance(DistanceUnit.CM) <= 50) {
                bl.setPower(1);
                br.setPower(0);
                fl.setPower(0);
                fr.setPower(1);
            }
        }

        public void backLeftDiagonal(){
            while(front_distance.getDistance(DistanceUnit.CM) >= 50){
                bl.setPower(0);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(0);
            }

        }

        while (opModeIsActive()){

            while (front_distance.getDistance)



        }
    }

}
