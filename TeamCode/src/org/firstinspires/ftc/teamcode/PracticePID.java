package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "Practice PID")
public class PracticePID extends LinearOpMode{

    private double kP = 0;  //Difference between setPoint and processVariable
    private double kI = 0;  //Error over time
    private double kD = 0;  //Proportional to the rate of change of the processVariable
    private double setPoint = 1500;
    private double processVariable = 0;
    private double time = 0;    //Set it equal to getRunTime() later while (opModeIsActive)




    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();

        while(opModeIsActive()){

        }

    }
}
