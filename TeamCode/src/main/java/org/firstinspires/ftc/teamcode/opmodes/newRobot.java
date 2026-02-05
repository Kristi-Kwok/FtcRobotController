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

@TeleOp(name = "newRobot")
public class newRobot extends LinearOpMode {
    double rotOffset = 0;
    double moveSpeed = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backright = hardwareMap.get(DcMotor.class, "backright");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // You don't HAVE to do this, but it makes things clear
        frontleft.setDirection(DcMotor.Direction.REVERSE );
        frontright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);

        //Make the motors brake whenever their power is zero
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            if (gamepad1.aWasPressed() || gamepad1.dpadUpWasPressed()) {
                imu.resetYaw();
                rotOffset = 0;
            }


            if (gamepad1.yWasPressed() || gamepad1.leftStickButtonWasPressed()) {
                if (moveSpeed == 1) {
                    moveSpeed = 0.2;
                } else {
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

            if(false){
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


            //intake
            if(gamepad1.b){
                intake.setPower(-1);
            } else if(y > Math.abs(x) || gamepad1.x){
                intake.setPower(1);
            }else{
                intake.setPower(0);
            }

            //LIMELIGHT AIMING
            //test values, plug in apriltag values
            double distFromTag = 20; //range
            double angleToTag = 30; //bearing
            double tagAngleToFlat = 0; //flat -> flat for the camera (straight on) also called yaw
            //check negative values


            //We get these values from the apriltag reading, but want them to be accurate here
            double opp1 = distFromTag * Math.sin(Math.toRadians(angleToTag)) * xPolarity; //x
            double adj1 = distFromTag * Math.cos(Math.toRadians(angleToTag)); //y

            //Legs of the tag-to-corner hypotenuse
            double opp2 = 18.3 * Math.sin(Math.toRadians(tagAngleToFlat)) * -xPolarity;
            double adj2 = 18.3 * Math.cos(Math.toRadians(tagAngleToFlat));

            //robot-to-corner triangle
            double opp3 = opp1 + opp2;
            double adj3 = adj1 + adj2;
            double dist = Math.sqrt((opp3 * opp3) + (adj3 * adj3));
            double angleToCorner = Math.toDegrees(Math.atan2(opp3, adj3));




            telemetry.addData("Corner Distance", dist);
            telemetry.addData("Angle To Corner", angleToCorner);


            telemetry.addData("front left motor", leftfrontPower);
            telemetry.addData("front right motor", rightfrontPower);
            telemetry.addData("back left motor", leftbackPower);
            telemetry.addData("back right motor", rightbackPower);
            telemetry.update();
        }
    }
}