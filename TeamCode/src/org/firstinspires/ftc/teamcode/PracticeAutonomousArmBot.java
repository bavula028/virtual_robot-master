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


        waitForStart();
        while (opModeIsActive()){



        }

    }
}
