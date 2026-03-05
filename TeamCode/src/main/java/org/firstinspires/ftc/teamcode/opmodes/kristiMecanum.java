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
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean shooting = false;
    double offset = 0;
    int sortAmt = 0;
    int sortCounter = 0;
    double rotOffset = 0;

    double moveSpeed = 1;
    double defaultFlywheelVel = 0;
    double oldFlywheelVel;
    int waitTimeNotPrerun = 300;
    int waitTimePrerun = 0;
    int waitTimeServoForward = 100;
    int waitTimeServoWait = 100;
    String curChanging = "";

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

            if (!shooting) {
                shootTimer.reset();
                shooting = true;
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            flywheelVel = flywheel.getVelocity();

            if (gamepad1.aWasPressed()) {
                imu.resetYaw();
                rotOffset = 0;
            }


            if (gamepad1.yWasPressed()) {
                if (moveSpeed == 1) {
                    moveSpeed = 0.2;
                } else {
                    moveSpeed = 1;
                }
            }

            //Press after loading
            if (gamepad1.dpad_right || gamepad1.left_bumper || gamepad2.a) {
                //stopped waiting, too much work :P
                defaultFlywheelVel = 500;
            }

            //MOVEMENT

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

            if (curDirection > 180) {
                curDirection -= 360;
            } else if (curDirection < -180) {
                curDirection += 360;
            }

            //use direction to "rotate" movement inputs
            if (0 <= curDirection && curDirection <= 90) {
                xPolarity = 1;
                yPolarity = 1;
                xPolarityY = 1;
                yPolarityX = -1;

                weight = 1 - (curDirection / 90);
            } else if (90 <= curDirection && curDirection <= 180) {
                xPolarity = -1;
                yPolarity = -1;
                xPolarityY = 1;
                yPolarityX = -1;

                weight = ((curDirection - 90) / 90);
            } else if (-90 <= curDirection && curDirection <= 0) {
                xPolarity = 1;
                yPolarity = 1;
                xPolarityY = -1;
                yPolarityX = 1;

                weight = 1 - ((curDirection * -1) / 90);
            } else if (-180 <= curDirection && curDirection <= -90) {
                xPolarity = -1;
                yPolarity = -1;
                xPolarityY = -1;
                yPolarityX = 1;

                weight = (((curDirection * -1) - 90) / 90);
            }


            double oldX = x;

            x = (x * weight * xPolarity) + (y * (1 - weight) * yPolarityX);
            y = (y * weight * yPolarity) + (oldX * (1 - weight) * xPolarityY);

            double normalizeVector = 1 / (Math.abs(x) + Math.abs(y));

            x *= normalizeVector;
            y *= normalizeVector;


            if (Double.isNaN(x)) {
                x = 0;
            }

            if (Double.isNaN(y)) {
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
            telemetry.addData("Sort Que:", sortAmt);
            telemetry.addData("Flywheel Velocity", flywheelVel);
            telemetry.addData("Target Flywheel Velocity", targetFlywheelVel);
            telemetry.addData("Left Servo", left_launch_servo.getPower());
            telemetry.addData("Right Servo", right_launch_servo.getPower());


            //testing values



            telemetry.addData("Currently Changing", curChanging);
            telemetry.addData("no charge wait time", waitTimeNotPrerun);
            telemetry.addData("servo forward time", waitTimeServoForward);
            telemetry.addData("servo wait time", waitTimeServoWait);

            telemetry.update();

            //FLYWHEEL (SHOOTING, SORTING, REVERSE)

            if (gamepad1.xWasPressed()) {
                sortAmt += 1;
            }

            if (gamepad1.b) {
                flywheel.setPower(-0.4);
                right_launch_servo.setPower(1);
                left_launch_servo.setPower(-1);
                sortAmt = 0;
            } else if (gamepad1.right_bumper) {
                if(gamepad1.rightBumperWasPressed()){
                    shootTimer.reset();
                    oldFlywheelVel = defaultFlywheelVel;
                }
                defaultFlywheelVel = 0;
                sortAmt = 0;
                //regular shot speed
                if (targetFlywheelVel < 1000)
                    targetFlywheelVel = 1490;


                //counter += 1;

                if(oldFlywheelVel > 0){
                    if (shootTimer.milliseconds() < 100) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (100 > shootTimer.milliseconds() && shootTimer.milliseconds() < 200) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                        if (targetFlywheelVel > 1470) {
                            targetFlywheelVel -= 40;
                        } else{
                            targetFlywheelVel += 40;
                        }
                    }
                    if (300 < shootTimer.milliseconds()) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                } else {
                    if (shootTimer.milliseconds() < 300) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (300 > shootTimer.milliseconds() && shootTimer.milliseconds() < 400) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                        if (targetFlywheelVel > 1470) {
                            targetFlywheelVel -= 40;
                        } else{
                            targetFlywheelVel += 40;
                        }
                    }
                    if (400 < shootTimer.milliseconds() && shootTimer.milliseconds() < 500) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                        counter = 11;
                    }
                }



            } else if(gamepad1.right_trigger > 0.5f){
                double oldFlywheelVel = defaultFlywheelVel;
                sortAmt = 0;
                defaultFlywheelVel = 0;
                //regular shot speed
                if (targetFlywheelVel < 1000)
                    targetFlywheelVel = 2350;


                counter += 1;

                if(oldFlywheelVel > 0) {
                    if (counter == 50) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (counter == 65) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                        if (targetFlywheelVel > 1470)
                            targetFlywheelVel -= 40;
                    }
                    if (counter == 77) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                        counter = 51;
                    }
                } else {
                    if (counter == 100) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                    }
                    if (counter == 115) {
                        right_launch_servo.setPower(0);
                        left_launch_servo.setPower(0);
                        if (targetFlywheelVel > 1470)
                            targetFlywheelVel -= 40;
                    }
                    if (counter == 127) {
                        right_launch_servo.setPower(-1);
                        left_launch_servo.setPower(1);
                        counter = 101;
                    }
                }
            }else if (sortAmt > 0) { //sorting system
                defaultFlywheelVel = 0;
                targetFlywheelVel = 760;

                sortCounter += 1;

                if (sortCounter == 100 && sortAmt < 50) {
                    right_launch_servo.setPower(-1);
                    left_launch_servo.setPower(1);
                }
                if (sortCounter == 130 && sortAmt < 50) {
                    right_launch_servo.setPower(1);
                    left_launch_servo.setPower(-1);
                    if(targetFlywheelVel > 700)
                        targetFlywheelVel -=65;
                }
                if (sortCounter == 150) {
                    right_launch_servo.setPower(-1);
                    left_launch_servo.setPower(1);
                    sortCounter = 101;
                    sortAmt -= 1;
                    if(sortAmt == 0){
                        sortAmt = 1000;
                        sortCounter = 0;
                        flywheel.setPower(-0.4);

                        right_launch_servo.setPower(1);
                        left_launch_servo.setPower(-1);
                    }
                }

                if(sortAmt > 50 && sortCounter == 130){
                    sortAmt = 0;
                }

            } else {
                counter = 0;
                sortCounter = 0;
                targetFlywheelVel = defaultFlywheelVel;
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }


            if (!gamepad1.b && sortAmt < 50) {
                double slowThreshold = 2;
                double motorSpeedTowardsTarget;
                if (flywheelVel < targetFlywheelVel) {
                    if (flywheelVel > (targetFlywheelVel / slowThreshold) * (slowThreshold - 1)) {
                        double normalVel = flywheelVel / targetFlywheelVel;
                        motorSpeedTowardsTarget = 1 - (2 * (normalVel - 0.5));
                        if (motorSpeedTowardsTarget < 0)
                            motorSpeedTowardsTarget = 0;
                    } else {
                        motorSpeedTowardsTarget = 1;
                    }
                    flywheel.setPower(motorSpeedTowardsTarget);
                } else {
                    flywheel.setPower(0);
                }
            }

        }
    }

}
