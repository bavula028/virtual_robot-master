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
    private DcMotor frontRight = new DcMotorImpl(Gobilda192, true, true);
    private DcMotor backLeft = new DcMotorImpl(Gobilda192, true, true);
    private DcMotor backRight = new DcMotorImpl(Gobilda192, true, true);



    public void runFor(int ticksInput){
        //How to implement setInertia and/or setVelocity?
        //frontLeft.setInertia();




    }

}
