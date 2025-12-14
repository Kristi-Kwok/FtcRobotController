package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class bluetop extends LinearOpMode {
    int counter = 0;
    int counter_loop = 0;
    double flywheelVel = 0;
    double targetFlywheelVel = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backright = hardwareMap.get(DcMotor.class, "backright");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        CRServo right_launch_servo = hardwareMap.get(CRServo.class, "rightServo");
        CRServo left_launch_servo = hardwareMap.get(CRServo.class, "leftServo");
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

        boolean shooting = false;

        double power = 0.7;
        imu.resetYaw();
        waitForStart();

        while (opModeIsActive()){
            shooting = true;

            if (shooting == true){
                double motorSpeedTowardsTarget;

                if(flywheelVel < targetFlywheelVel) {
                    if (flywheelVel > (targetFlywheelVel / 2)) {
                        double normalVel = flywheelVel / targetFlywheelVel;
                        motorSpeedTowardsTarget = 1.3 - (2 * (normalVel - 0.5));
                        if(motorSpeedTowardsTarget < 0)
                            motorSpeedTowardsTarget = 0;
                    } else {
                        motorSpeedTowardsTarget = 1;
                    }
                    flywheel.setPower(motorSpeedTowardsTarget);
                } else {
                    flywheel.setPower(0);
                }

                counter += 1;

                if (counter == 200) {
                    right_launch_servo.setPower(-1);
                    left_launch_servo.setPower(1);
                }
                if (counter == 275) {
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                }
                if (counter == 325) {
                    right_launch_servo.setPower(-1);
                    left_launch_servo.setPower(1);
                    counter = 201;
                }
            } else {
                counter = 0;
                flywheel.setPower(0);
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
            sleep(10000);
            //backwards .5 seconds
            frontleft.setPower(-.7);
            frontright.setPower(-.7);
            backleft.setPower(-.7);
            backright.setPower(-.7);
            sleep(500);
            //straf right
            frontleft.setPower(-.7);
            frontright.setPower(-.7);
            backleft.setPower(.7);
            backright.setPower(.7);
            sleep(600);
            frontleft.setPower(-.7);
            frontright.setPower(-.7);
            backleft.setPower(.7);
            backright.setPower(.7);
            sleep(500);
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);




            }
        }
    }
