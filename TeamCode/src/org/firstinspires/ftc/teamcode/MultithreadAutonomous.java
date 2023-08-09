package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImpl;

@Autonomous(name = "Arm Bot Autonomous Multi-Thread")
public class MultithreadAutonomous extends LinearOpMode implements Runnable {
    @Override
    public void runOpMode() throws InterruptedException {

        //Wheel Motors
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        //Arm and Hand Motors
        DcMotor armMotor = hardwareMap.dcMotor.get("arm_motor");
        ServoImpl hand_servo = hardwareMap.get(ServoImpl.class ,"hand_servo");

        //Distance Sensors
        DistanceSensor front_distance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor back_distance = hardwareMap.get(DistanceSensor.class, "back_distance");
        DistanceSensor left_distance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor right_distance = hardwareMap.get(DistanceSensor.class, "right_distance");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = " ";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "loggingTag";

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




        }

    }

    @Override
    public void run() {
        //Think of the motors called above are private motors.
        //Call new motors in this function.
        //Set the motors called above equal to the motors called in this function.

        //How do we make the thing work without a "public static void main" statement?
        //We could try to make it work through the actual runOpMode thing.

    }

}
