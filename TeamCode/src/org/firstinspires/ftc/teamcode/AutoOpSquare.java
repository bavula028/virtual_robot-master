package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import virtual_robot.controller.AutonomousExampleMethods;
import com.qualcomm.robotcore.hardware.DcMotorImpl;


import java.lang.annotation.Annotation;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

@Autonomous(name = "AutonomousSquare")
public class AutoOpSquare extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //DcMotorImpl frontLeft = new DcMotorImpl(hardwareMap.dcMotor.get("front_left_motor"));
        //DcMotorImpl backLeft = hardwareMap.dcMotor.get("back_left_motor");
        //DcMotorImpl frontRight = hardwareMap.dcMotor.get("front_right_motor");
        //DcMotorImpl backRight = hardwareMap.dcMotor.get("back_right_motor");

        //Use DcMotorImpl's for the wheels. Use GoBuilda 194's. Found in the enum called MotorType.

        //frontRight.setDirection(DcMotor.Direction.FORWARD);
        //backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){

            //THIS IS ONLY A ROUGH DRAFT
            //TRYING TO MAKE THE ROBOT MOVE


            //frontLeft.getCurrentPosition();

            //SET TO MOVE FORWARD
            //frontLeft.setTargetPosition(210000);
            //backLeft.setTargetPosition(210000);
            //frontRight.setTargetPosition(210000);
            //backRight.setTargetPosition(210000);

            //frontLeft.setPower(13);
            //backLeft.setPower(13);       //POWER MUST BE BETWEEN -1 AND 1
            //frontRight.setPower(13);
            //backRight.setPower(13);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //TURN RIGHT
            //frontLeft.setTargetPosition(1200);
            //backLeft.setTargetPosition(1200);
            //frontRight.setTargetPosition(0);
            //backRight.setTargetPosition(0);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(0.0);
            //backRight.setPower(0.0);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //MOVE FORWARD
            //frontLeft.setTargetPosition(5000);
            //backLeft.setTargetPosition(5000);
            //frontRight.setTargetPosition(5000);
            //backRight.setTargetPosition(5000);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(1.5);
            //backRight.setPower(1.5);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //TURN RIGHT
            //frontLeft.setTargetPosition(1200);
            //backLeft.setTargetPosition(1200);
            //frontRight.setTargetPosition(0);
            //backRight.setTargetPosition(0);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(0.0);
            //backRight.setPower(0.0);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //MOVE FORWARD
            //frontLeft.setTargetPosition(5000);
            //backLeft.setTargetPosition(5000);
            //frontRight.setTargetPosition(5000);
            //backRight.setTargetPosition(5000);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(1.5);
            //backRight.setPower(1.5);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //TURN RIGHT
            //frontLeft.setTargetPosition(1200);
            //backLeft.setTargetPosition(1200);
            //frontRight.setTargetPosition(0);
            //backRight.setTargetPosition(0);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(0.0);
            //backRight.setPower(0.0);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //MOVE FORWARD
            //frontLeft.setTargetPosition(5000);
            //backLeft.setTargetPosition(5000);
            //frontRight.setTargetPosition(5000);
            //backRight.setTargetPosition(5000);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(1.5);
            //backRight.setPower(1.5);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //TURN RIGHT
            //frontLeft.setTargetPosition(1200);
            //backLeft.setTargetPosition(1200);
            //frontRight.setTargetPosition(0);
            //backRight.setTargetPosition(0);

            //frontLeft.setPower(1.5);
            //backLeft.setPower(1.5);
            //frontRight.setPower(0.0);
            //backRight.setPower(0.0);

            //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }

    }
}