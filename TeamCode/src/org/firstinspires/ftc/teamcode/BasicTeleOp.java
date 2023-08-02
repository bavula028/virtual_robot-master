package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "BasicOpMode")
public class BasicTeleOp extends LinearOpMode {


    @Override
    public  void runOpMode() throws InterruptedException {

        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");

        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            float wheelPower = gamepad1.left_stick_y;
            float wheelPower2 = gamepad1.right_stick_y;

            br.setPower(wheelPower2);
            bl.setPower(wheelPower);
            fr.setPower(wheelPower2);
            fl.setPower(wheelPower);

        }


    }
}
