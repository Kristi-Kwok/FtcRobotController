package org.firstinspires.ftc.teamcode.opmodes;
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "kristiMecanum")
public class kristiMecanum extends LinearOpMode {
    int counter = 0;
    double power = 1;
    double flywheelVel = 0;
    double targetFlywheelVel = 1700;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean shooting = false;
    double offset = 0;
    int sortAmt = 0;
    int sortCounter = 0;
    double rotOffset = 0;

    double moveSpeed = 1;

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
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);

        //Make the motors brake whenever their power is zero
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

            if(!shooting){
                shootTimer.reset();
                shooting = true;
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            flywheelVel = flywheel.getVelocity();

            if (gamepad1.aWasPressed() || gamepad1.dpadUpWasPressed()){
                imu.resetYaw();
            }


            if(gamepad1.yWasPressed() || gamepad1.leftStickButtonWasPressed()){
                if(moveSpeed == 1){
                    moveSpeed = 0.2;
                } else{
                    moveSpeed = 1;
                }
            }


            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            double moveAmnt = (Math.abs(x) + Math.abs(y)) * moveSpeed;

            //field centric code

            double weight = 0;
            int xPolarity = 0;
            int yPolarityX = 0;
            int yPolarity = 0;
            int xPolarityY = 0;

            //change data from IMU so 0 degrees is forward
            double curDirection = heading + rotOffset;

            if(curDirection > 180){
                curDirection -= 360;
            }else if(curDirection < -180){
                curDirection += 360;
            }

            //use direction to "rotate" movement inputs
            if(0 <= curDirection && curDirection <= 90){
                xPolarity = 1;
                yPolarity = 1;
                xPolarityY = 1;
                yPolarityX = -1;

                weight = 1 - (curDirection / 90);
            }else if(90 <= curDirection && curDirection <= 180){
                xPolarity = -1;
                yPolarity = -1;
                xPolarityY = 1;
                yPolarityX = -1;

                weight = ((curDirection - 90) / 90);
            }else if(-90 <= curDirection && curDirection <= 0){
                xPolarity = 1;
                yPolarity = 1;
                xPolarityY = -1;
                yPolarityX = 1;

                weight = 1 - ((curDirection*-1) / 90);
            }else if(-180 <= curDirection && curDirection <= -90){
                xPolarity = -1;
                yPolarity = -1;
                xPolarityY = -1;
                yPolarityX = 1;

                weight = (((curDirection*-1)-90) / 90);
            }


            double oldX = x;

            x = (x * weight * xPolarity) + (y * (1 - weight) * yPolarityX);
            y = (y * weight * yPolarity) + (oldX * (1 - weight) * xPolarityY);

            double normalizeVector = 1 / (Math.abs(x) + Math.abs(y));

            x *= normalizeVector;
            y *= normalizeVector;


            if(Double.isNaN(x)){
                x = 0;
            }

            if(Double.isNaN(y)){
                y = 0;
            }
            
            x *= moveAmnt;
            y *= moveAmnt;
            rot *= moveSpeed;

            //assign power to motors

            double leftfrontPower = y - x - rot;
            double rightfrontPower = y + x + rot;
            double leftbackPower = y + x - rot;
            double rightbackPower = y - x + rot;

            frontleft.setPower(leftfrontPower);
            frontright.setPower(rightfrontPower);
            backleft.setPower(leftbackPower);
            backright.setPower(rightbackPower);


            telemetry.addData("Heading (Z)", heading);

            telemetry.addData("front left motor", leftfrontPower);
            telemetry.addData("front right motor", rightfrontPower);
            telemetry.addData("back left motor", leftbackPower);
            telemetry.addData("back right motor", rightbackPower);
            telemetry.addData("x input", x);
            telemetry.addData("y input", y);
            telemetry.addData("rotation input", rot);
            telemetry.addData("Sort Que:", sortAmt);
            telemetry.update();

            if(gamepad1.xWasPressed()){
                sortAmt += 1;
            }

            if(gamepad1.b) {
                flywheel.setPower(-0.4);
                right_launch_servo.setPower(1);
                left_launch_servo.setPower(-1);
            }else if(gamepad1.right_bumper){
                double motorSpeedTowardsTarget;
                targetFlywheelVel = 1650;
                if(gamepad1.left_bumper){
                    targetFlywheelVel = 2600;
                }

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
                    if (counter == 215) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    if (counter == 300) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                        counter = 201;
                    }
                } else if(sortAmt > 0){ //sorting system
                double motorSpeedTowardsTarget;
                targetFlywheelVel = 525;

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


                    sortCounter += 1;

                    if (sortCounter == 200) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (sortCounter == 215) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                    }
                    if (sortCounter == 300) {
                        sortCounter = 180;
                        sortAmt -= 1;
                    }

                }else {
                    counter = 0;
                    sortCounter = 0;
                    flywheel.setPower(0);
                    right_launch_servo.setPower(0);
                    left_launch_servo.setPower(0);
                }
            }

        }
    }
