package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

public class WheelsPID extends PID{
    //This class is applicable for Arm Bot and Mecanum Bot.

    double kP = 0;
    double kI = 0;
    double kD = 0;
    double proportional = 0;
    double integral = 0;
    double derivative = 0;
    double setPoint = 0;
    double processVariable = 0;
    double time = 0;
    double error = 0;
    double previousError = 0;
    double currentPosition = 0;
    double output = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");






    }
}
