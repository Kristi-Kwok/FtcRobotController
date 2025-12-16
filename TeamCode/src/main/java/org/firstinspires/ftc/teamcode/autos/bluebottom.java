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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class bluebottom extends LinearOpMode {
    double flywheelVel = 0;
    double targetFlywheelVel = 2000;
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotorEx flywheel;
    CRServo right_launch_servo;
    CRServo left_launch_servo;
    ElapsedTime shootTimer = new ElapsedTime();

    boolean shooting = false;

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


        double power = 0.7;
        imu.resetYaw();
        waitForStart();


        move(0,1,900);
        sleep(900);
        rotate(-1, 80);
        sleep(80);
        move(0,1,500);
        sleep(500);
        shoot();
        sleep(13000);
    }
    //Move in a certain direction for a certain amount of time
    //Make sure x + y = 1
    public void move(double x, double y, long time) throws InterruptedException {

        y = -y;
        x= -x;



        double leftfrontPower = y-x;
        double rightfrontPower = y+x;
        double leftbackPower = y+x;
        double rightbackPower = y-x;
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

    //rotate the robot
    //direction = 1 for clockwise, direction = -1 for counterclockwise (intended, but doublecheck)
    public void rotate(int direction, long time) throws InterruptedException {
        frontleft.setPower(1 * direction);
        frontright.setPower(-1 * direction);
        backleft.setPower(1 * direction);
        backright.setPower(-1 * direction);

        sleep(time);

        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    //wait 13 seconds after calling the function
    public void shoot(){

        shooting = true;
        shootTimer.reset();

        while (shooting == true){
            flywheelVel = flywheel.getVelocity();
            double motorSpeedTowardsTarget = 0;


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
            } else {
                flywheel.setPower(0);

            }

            telemetry.addData("Timer", shootTimer.seconds());
            telemetry.addData("Target Velocity", targetFlywheelVel);
            telemetry.addData("Motor Speed", motorSpeedTowardsTarget);
            telemetry.addData("Velocity of Motor", flywheelVel);
            telemetry.update();

            if (shootTimer.seconds() < 3 && shootTimer.seconds() > 2) {
                right_launch_servo.setPower(-1);
                left_launch_servo.setPower(1);
            }
            if (shootTimer.seconds() < 5 && shootTimer.seconds() > 3) {
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }
            if (shootTimer.seconds() < 7 && shootTimer.seconds() > 6) {
                right_launch_servo.setPower(-1);
                left_launch_servo.setPower(1);
            }
            if (shootTimer.seconds() < 9 && shootTimer.seconds() > 7) {
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }
            if (shootTimer.seconds() < 10 && shootTimer.seconds() > 8) {
                right_launch_servo.setPower(-1);
                left_launch_servo.setPower(1);

            }
            if (shootTimer.seconds() > 13){
                shooting = false;
                flywheel.setPower(0);
                right_launch_servo.setPower(0);
                left_launch_servo.setPower(0);
            }
            sleep(1);
        }
    }
}




