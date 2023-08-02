package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class FieldCentric extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize IMU and Motors
            BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
            DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
            DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
            DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
            DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");

            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            fl.setDirection(DcMotorSimple.Direction.REVERSE);

        //STEP ONE: Get Joystick Angle
        double x, y, joystickAngle, magnitude;

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;

        joystickAngle = Math.atan(y/x);
        magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        //STEP TWO: Get Robot Angle
        double robotAngle;
        robotAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle * 180 / Math.PI;

        //STEP THREE: Normalize Joystick Angle (Part 1)
        if (joystickAngle > 270){
                joystickAngle = joystickAngle * -1 + 450;
                //Ex. joystickAngle = 300, joystickAngle = 150
        }
        else if (joystickAngle < 270){
                joystickAngle = joystickAngle * -1 + 90;
                //Ex. joystickAngle = 100, joystickAngle = -10
        }







    }
}
