package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous
public class MoveLeft extends LinearOpMode {
    double flywheelVel = 0;
    double targetFlywheelVel = 1600;
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotorEx flywheel;
    CRServo right_launch_servo;
    CRServo left_launch_servo;

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_launch_servo = hardwareMap.get(CRServo.class, "rightServo");
        left_launch_servo = hardwareMap.get(CRServo.class, "leftServo");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        //setting direction of each motor
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Calibrating...");
        telemetry.update();

        ElapsedTime shootTimer = new ElapsedTime();

        boolean shooting = false;

        double power = 0.7;
        imu.resetYaw();
        waitForStart();

        move(1, 0, 75, .75);
        sleep(75);
    }

    public void move(double x, double y, long time, double power) throws InterruptedException {

        time *= (1/power);

        double leftfrontPower = (y-x) * power;
        double rightfrontPower = (y+x) * power;
        double leftbackPower = (y+x) * power;
        double rightbackPower = (y-x) * power;
        frontleft.setPower(leftfrontPower);
        frontright.setPower(rightfrontPower);
        backleft.setPower(leftbackPower);
        backright.setPower(rightbackPower);

        sleep(time);

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

    }
}
