package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;

import static com.qualcomm.robotcore.hardware.configuration.MotorType.Gobilda192;

public class AutonomousExampleMethods {

    private int ticks;
    private DcMotorImpl frontLeft = new DcMotorImpl(Gobilda192, true, true);
    private DcMotorImpl frontRight = new DcMotorImpl(Gobilda192, true, true);
    private DcMotorImpl backLeft = new DcMotorImpl(Gobilda192, true, true);
    private DcMotorImpl backRight = new DcMotorImpl(Gobilda192, true, true);


    public void runFor(int ticksInput){
        //How to implement setInertia and/or setVelocity?
        ticks = ticksInput;

        frontLeft.setInertia(0.99);
        frontRight.setInertia(0.99);
        backLeft.setInertia(0.99);
        backRight.setInertia(0.99);

        frontLeft.setPower(1.0);
        frontRight.setPower(1.0);
        backLeft.setPower(1.0);
        backLeft.setPower(1.0);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }




}
