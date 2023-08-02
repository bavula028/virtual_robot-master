package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric")
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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //New parameters
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = " ";
        parameters.loggingEnabled = true;
        parameters.loggingTag = " ";
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()){
            //STEP ONE: Get Joystick Angle
            double x, y, joystickAngle, magnitude;

            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;

            joystickAngle = Math.atan(y / x);
            magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

            //STEP TWO: Get Robot Angle
            double robotHeading;
            robotHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle;

            //STEP THREE: Normalize Joystick Angle (Part 1)
            if (joystickAngle > 270) {
                joystickAngle = joystickAngle * -1 + 450;
                //Ex. joystickAngle = 300, joystickAngle = 150
            } else if (joystickAngle < 270) {
                joystickAngle = joystickAngle * -1 + 90;
                //Ex. joystickAngle = 100, joystickAngle = -10
            }

            //STEP THREE: Normalize Robot and Joystick Angle (Part 2)
            if (joystickAngle < 0) {
                joystickAngle = joystickAngle + 360;
            }

            //STEP FOUR: Get Lateral Travel Angle

            double lateralTravelAngle = 0;

            if (robotHeading > joystickAngle) {
                lateralTravelAngle = (360 - robotHeading) + joystickAngle;
            } else if (robotHeading < joystickAngle) {
                lateralTravelAngle = joystickAngle - robotHeading;
            }

            //STEP FIVE: Travel Equations

            double travelAngleVariable = (lateralTravelAngle + (Math.PI / 4)) * magnitude;

            bl.setPower(Math.sin(travelAngleVariable));
            fr.setPower(Math.sin(travelAngleVariable));
            br.setPower(Math.cos(travelAngleVariable));
            fl.setPower(Math.cos(travelAngleVariable));

            //STEP SIX: Adding Rotation
            double rotation = gamepad1.right_stick_x;

            bl.setPower(Math.sin(travelAngleVariable) - rotation);
            fr.setPower(Math.sin(travelAngleVariable) - rotation);
            br.setPower(Math.cos(travelAngleVariable) + rotation);
            fl.setPower(Math.cos(travelAngleVariable) + rotation);

            //STEP SEVEN: Adding Data to the Telemetry
            telemetry.addData("joystick angle:", joystickAngle);
            telemetry.addData("magnitude:", magnitude);
            telemetry.addData("robot heading:", robotHeading);
            telemetry.addData("lateral travel angle:", lateralTravelAngle);
            telemetry.addData("travel angle:", travelAngleVariable);
            telemetry.update();

            if (gamepad1.a)
            {
                requestOpModeStop();
            }
        }

    }




}