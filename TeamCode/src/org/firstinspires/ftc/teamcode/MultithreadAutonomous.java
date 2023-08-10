package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.ThreadFunctions;

@Autonomous(name = "Arm Bot Autonomous Multi-Thread")
public class MultithreadAutonomous extends LinearOpMode implements Runnable {
    @Override
    public void runOpMode() throws InterruptedException {

       waitForStart();


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
