package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class PID extends LinearOpMode {

    public double kP;
    public double kI;
    public double kD;
    public double proportional;
    public double integral;
    public double setPoint;
    public double processVariable;
    public double time;
    public double error;
    public double previousError;
    public double currentPosition;
    public double output;

    public abstract void runOpMode() throws InterruptedException;

}
