package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

//Try out the servos
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.Servo;


import com.qualcomm.robotcore.hardware.ServoImpl;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Practice Autonomous Arm Bot")
public class PracticeAutonomousArmBot extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Call Motors
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor am = hardwareMap.dcMotor.get("arm_motor");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = " ";
        parameters.loggingEnabled = false;
        parameters.loggingTag = " ";
        imu.initialize(parameters);

        DistanceSensor front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");
        DistanceSensor left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");

        //Do I want to code hand servos?
        ServoImpl hand_servo = hardwareMap.get(ServoImpl.class, "hand_servo");

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

        waitForStart();
        while (opModeIsActive()){

            //Robot starts sideways.
            //Strafes right.
            //Hopefully.


            while(left_distance.getDistance(DistanceUnit.CM) <= 140){
                bl.setPower(-1);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(-1);

                telemetry.addData("length from left wall:", left_distance.getDistance(DistanceUnit.CM));
                telemetry.update();

            }  //Keeps returning 820.0. Why?

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            while (front_distance.getDistance(DistanceUnit.CM) > 10){
                bl.setPower(1);
                br.setPower(1);
                fl.setPower(1);
                fr.setPower(1);

                telemetry.addData("length from left wall:", left_distance.getDistance(DistanceUnit.CM));
                telemetry.addData("front distance:", front_distance.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);

            //HOW DO I CODE THE SERVOS?

            hand_servo.setPosition(1); // means open

            while (front_distance.getDistance(DistanceUnit.CM) < 65){
                bl.setPower(-1);
                br.setPower(-1);
                fl.setPower(-1);
                fr.setPower(-1);
            }
            bl.setPower(0);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(0);




            requestOpModeStop();






        }

    }
}

