package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "kristiMecanum")
public class kristiMecanum extends LinearOpMode {
    int counter = 0;
    double power = 1;
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

        // You don't HAVE to do this, but it makes things clear
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Calibrating...");
        telemetry.update();

        imu.resetYaw();
        waitForStart();

        while (opModeIsActive()) {
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            flywheelVel = flywheel.getVelocity();


            double leftfrontPower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            double rightfrontPower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            double leftbackPower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            double rightbackPower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            frontleft.setPower(leftfrontPower);
            frontright.setPower(rightfrontPower);
            backleft.setPower(leftbackPower);
            backright.setPower(rightbackPower);


            telemetry.addData("Heading (Z)", heading);

            telemetry.addData("front left motor", leftfrontPower);
            telemetry.addData("front right motor", rightfrontPower);
            telemetry.addData("back left motor", leftbackPower);
            telemetry.addData("back right motor", rightbackPower);
            telemetry.update();

            if (gamepad1.dpad_left == true){
                frontleft.setPower(1);
                frontright.setPower(-1);
                backleft.setPower(-1);
                backright.setPower(1);
            }

            if (gamepad1.dpad_right == true){
                frontleft.setPower(-1);
                frontright.setPower(1);
                backleft.setPower(1);
                backright.setPower(-1);
            }

            if(gamepad1.right_bumper){

                double motorSpeedTowardsTarget;

                if(flywheelVel < targetFlywheelVel) {
                    if (flywheelVel > (targetFlywheelVel / 2)) {
                        double normalVel = flywheelVel / targetFlywheelVel;
                        motorSpeedTowardsTarget = 1 - (2 * (normalVel - 0.5));
                        if(motorSpeedTowardsTarget < 0)
                            motorSpeedTowardsTarget = 0;
                    } else {
                        motorSpeedTowardsTarget = 1;
                    }
                    flywheel.setPower(motorSpeedTowardsTarget);
                }
                else {
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
        }
        }
    }