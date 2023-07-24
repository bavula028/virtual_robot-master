package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

@TeleOp(name = "BasicOpMode")
public class BasicTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            float leftPower = gamepad1.left_stick_y;
            float rightPower = gamepad1.right_stick_y;

            br.setPower(rightPower); //right joystick controls right wheels
            bl.setPower(leftPower); //left joystick controls left wheels
            fr.setPower(rightPower);
            fl.setPower(leftPower);

        }

    }
}
