package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.ServoImpl;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Thread Functions")
public class ThreadFunctions extends LinearOpMode implements Runnable {

    private DcMotor bl;
    private DcMotor br;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor arm;
    private ServoImpl hand_servo;
    private DistanceSensor back = null;
    private DistanceSensor front = null;
    private DistanceSensor left = null;
    private DistanceSensor right = null;
    private BNO055IMU imu = null;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private HardwareMap hardwareMap;

    public void runOpModeThread(DcMotor bl, DcMotor br, DcMotor fl, DcMotor fr, DcMotor arm, ServoImpl hand_servo, DistanceSensor back, DistanceSensor front, DistanceSensor left, DistanceSensor right, BNO055IMU imu, BNO055IMU.Parameters params){

        //INITIALIZE EVERYTHING
        bl = this.bl;
        br = this.br;
        fl = this.fl;
        fr = this.fr;

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        arm = this.arm;

        left = this.left;
        right = this.right;
        front = this.front;
        back = this.back;

        left = hardwareMap.get(DistanceSensor.class, "left_distance_sensor");
        right = hardwareMap.get(DistanceSensor.class, "right_distance_sensor");
        front = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");
        back = hardwareMap.get(DistanceSensor.class, "back_distance_sensor");

        hand_servo = this.hand_servo;

        imu = this.imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        params = parameters;
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.calibrationData = null;
        params.calibrationDataFile = " ";
        params.loggingEnabled = true;
        params.loggingTag = " ";
        imu.initialize(params);

        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");
        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");

        arm = hardwareMap.dcMotor.get("arm_motor");

        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void forwardLeftDiagonal(){
        while (front.getDistance(DistanceUnit.CM) >= 20){  //Unknown Error on this line
            bl.setPower(1);
            br.setPower(0);
            fl.setPower(0);
            fr.setPower(1);
        }
    }

    public void forwardRightDiagonal(){
        while (front.getDistance(DistanceUnit.CM) >= 20){
            bl.setPower(0);
            br.setPower(1);
            fl.setPower(1);
            fr.setPower(0);
        }
    }

    public void runOpMode() throws InterruptedException{

        waitForStart();

        forwardLeftDiagonal();  //Affected by the error on line 87
    }

    @Override
    public void run() {
        while(front.getDistance(DistanceUnit.CM) >= 65){
            bl.setPower(1);
            br.setPower(1);
            fl.setPower(1);
            fr.setPower(1);
        }
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
        fr.setPower(0);


    }
}
