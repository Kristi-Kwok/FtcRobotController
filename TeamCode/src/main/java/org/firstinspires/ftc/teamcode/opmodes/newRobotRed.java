package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareCode.PIDConfig;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.FileOutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.io.File;
import java.io.InputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import android.os.Environment;



@TeleOp(name = "newRobotRed")
@Config
public class newRobotRed extends LinearOpMode {
    double rotOffset = 0;
    double moveSpeed = 1;
    double flywheelVel;
    double targetFlywheelVel = 0;
    Limelight3A limelight;
    boolean isTargeting;
    double distToTarget;
    double angleFromFlatToTarget;
    double angleToTarget;
    double xPos;
    double yPos;
    Servo gate;
    int artifactCount;
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime shootTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean isShooting;
    public static int flywheelTestingVelocity = 1420;
    boolean GateOpen = false;
    double oldRot = 0;
    double oldTime = 0;
    double oldFlywheelDist = 0;
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001};
    //Index to select the current step size from the array
    int stepIndex = 2;
    public static double r_kD = 0.006, r_kI, r_kP = .03;
    public static double f_kD = 0.002, f_kI = 0.000001, f_kP = .008;
    PIDConfig RotPID = new PIDConfig(0.5, 0, 0);
    Telemetry dashboardTelemetry  = FtcDashboard.getInstance().getTelemetry();
    double r_integralSum, f_integralSum = 0;
    int lastFileSaved = 0;
    int downloadFrameCountdown = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor frontright = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backleft = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor backright = hardwareMap.get(DcMotor.class, "backright");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gate = hardwareMap.get(Servo.class, "gate");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //set up limelight
        limelight.setPollRateHz(90); // gets data from limelight 90 times per second
        limelight.start(); //start detection
        limelight.pipelineSwitch(0);



        // You don't HAVE to do this, but it makes things clear
        intake.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        //Make the motors brake whenever their power is zero
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ImuOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP);

        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        telemetry.addData("Status", "IMU Calibrating...");
        telemetry.update();

        imu.resetYaw();
        waitForStart();

        runtime.reset();


        while (opModeIsActive()) {

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if(gamepad2.dpadUpWasPressed()){
                flywheelTestingVelocity += 10;
            } else if(gamepad2.dpadDownWasPressed()){
                flywheelTestingVelocity -= 10;
            }

            if (gamepad1.aWasPressed()) {
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

            if(downloadFrameCountdown > 0){
                if(downloadFrameCountdown == 1){
                    downloadSnapshot();
                }
                downloadFrameCountdown--;
            }

            if(gamepad2.rightBumperWasPressed() && !gamepad1.right_bumper){
                limelight.captureSnapshot("snapshot");
                downloadFrameCountdown = 9;
            }

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;

            double moveAmnt = (Math.abs(x) + Math.abs(y)) * moveSpeed;

            flywheelVel = flywheel.getVelocity();


            //keep track of how many artifacts we have
            if(gamepad2.aWasPressed()){
                if(artifactCount < 3){
                    artifactCount++;
                }
            } else if(gamepad2.bWasPressed()){
                if(artifactCount > 0){
                    artifactCount++;
                }
            }


    //      ---------------------intake---------------------

            if(gamepad1.b){
                intake.setPower(-1);
            } else if(gamepad1.x){
                if(artifactCount != 3){
                    intake.setPower(1);
                }
            }else{
                intake.setPower(0);
            }



//------------------------LIMELIGHT AIMING------------------------

            limelight.updateRobotOrientation(heading+90);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {


                xPos = result.getBotpose_MT2().getPosition().x * 39.37;
                yPos = result.getBotpose_MT2().getPosition().y * 39.37;
                double llRot = result.getBotpose_MT2().getOrientation().getYaw();



                //xPos = result.getBotpose().getPosition().x * 39.37;
                //yPos = result.getBotpose().getPosition().y * 39.37;
                //double llRot = result.getBotpose().getOrientation().getYaw();


                double targetX = -70;
                double targetY = 70;

                double xDistToCorner = Math.abs(targetX - xPos);
                double yDistToCorner = targetY - yPos;

                distToTarget = Math.sqrt((xDistToCorner * xDistToCorner) + (yDistToCorner * yDistToCorner));
                angleFromFlatToTarget = Math.toDegrees(Math.atan2(xDistToCorner, yDistToCorner));
                angleToTarget = -(angleFromFlatToTarget - (heading));
                telemetry.addData("Angle From Flat", angleFromFlatToTarget);

                isTargeting = true;

                telemetry.addData("Limelight", "Targeting");
            } else {
                telemetry.addData("Limelight", "No Targets");
                isTargeting = false;
            }


    //      ---------------------flywheel power calculations---------------------

            double height = 16.5;
            double targetHeight = 40;

            //testing values, we need to test
            double flywheelVelPerDist = flywheelTestingVelocity; //Flywheel velocity for when target is 100 in away, constant
            double velModifier_perDist = 71.4285714286; //velModifier for when target is 100 in away, constant (calc in desmos)

            double velModifier = -(distToTarget*distToTarget)/((targetHeight - height - distToTarget) * 2.0);
            double neededVelocity = flywheelVelPerDist * (velModifier / velModifier_perDist);

    //      ---------------------modifying rotation PID at runtime (only for driver hub control)---------------------



            //'B' cycles through the different step sizes for tuning precision
            if (gamepad2.aWasPressed()) {
                stepIndex = (stepIndex +1) % stepSizes.length; //Modulo wraps the index back to 0
            }

            //Used gamepad2 to not override any of the controls already stated
            //You may still need to test it but you can change the P and D while running the opmode
            if (gamepad2.dpadLeftWasPressed()) {
                RotPID.setkP(RotPID.getkP() - stepSizes[stepIndex]);
            }

            if (gamepad2.dpadRightWasPressed()) {
                RotPID.setkP(RotPID.getkP() + stepSizes[stepIndex]);
            }

            // D-pad up/down adjust the D gain
            if (gamepad2.dpadUpWasPressed()) {
                RotPID.setkD(RotPID.getkD() + stepSizes[stepIndex]);
            }

            if (gamepad2.dpadDownWasPressed()) {
                RotPID.setkD(RotPID.getkD() - stepSizes[stepIndex]);
            }

            // D-pad up/down adjust the D gain
            if (gamepad2.xWasPressed()) {
                RotPID.setkI(RotPID.getkI() + stepSizes[stepIndex]);
            }

            if (gamepad2.yWasPressed()) {
                RotPID.setkI(RotPID.getkI() - stepSizes[stepIndex]);
            }

    //      ---------------------actually shooting---------------------

            if(gamepad1.left_bumper){
                targetFlywheelVel = flywheelTestingVelocity;
            } else {
                targetFlywheelVel = 0;
            }

            if(gamepad1.right_bumper){
                if(isTargeting){
                    rot = pid(r_kP, r_kI, r_kD, oldRot, oldTime, angleToTarget, runtime.milliseconds(), "r");
                } else {
                    if (heading < -125 || heading > 55) {
                        rot = 0.5;
                    }else if(heading > 54 && heading < 55){
                        rot = 0;
                    }else {
                        rot = -0.5;
                    }

                    r_integralSum = 0;
                }

                targetFlywheelVel = flywheelTestingVelocity;
                GateOpen = true;


                if(Math.abs(angleToTarget) < 3.5 && !isShooting && Math.abs(targetFlywheelVel - flywheelVel) <= 20){
                    isShooting = true;
                    shootTimer.reset();
                }

                if (isShooting) {
                    intake.setPower(1);
                    if(shootTimer.milliseconds() > 130){
                        targetFlywheelVel += 5;
                    }
                }
            } else {
                targetFlywheelVel = 0;
                isShooting = false;
                r_integralSum = 0;
                GateOpen = false;
            }

            //offwall sorting
            //if(gamepad1.dpad_left){
            //    if(xPos > 0){
            //        double xDist = 58.5 - xPos;
            //        x = xDist;
            //        double rotDist = -90 - heading;
            //        rot = rotDist/5;
            //    }else{
            //
            //    }
            //
            //}



            if(gamepad1.dpad_left){
                GateOpen = true;
            }
            else if(gamepad1.dpad_right){
                GateOpen = false;
            }

            if(GateOpen){
                gate.setPosition(0.35);
            } else {
                gate.setPosition(0.15);
            }

    //      ---------------------field centric---------------------
    //      (should prob put in a mecanum general class)

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
            //double oldY = y;

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

    //      ----------------------telemetry--------------------


            telemetry.addData("Corner Distance", distToTarget);
            telemetry.addData("Angle To Corner", angleToTarget);
            telemetry.addData("Needed Velocity", neededVelocity);

            telemetry.addData("X position", xPos);
            telemetry.addData("Y position", yPos);
            telemetry.addData("Heading (Z)", heading);

            telemetry.addData("Flywheel Testing Velocity", flywheelTestingVelocity);

            telemetry.addData("Tuning P", "%.7f (D-Pad L/R)", RotPID.getkP());
            telemetry.addData("Tuning D", "%.7f (D-Pad U/D)", RotPID.getkD());
            telemetry.addData("Tuning I", "%.7f (X/Y)", RotPID.getkI());
            telemetry.addData("Step Size", "%.7f (A button)", stepSizes[stepIndex]);

            telemetry.update();

            //dashboard telemetry
            if(true){
                dashboardTelemetry.addData("Corner Distance", distToTarget);
                dashboardTelemetry.addData("Angle To Corner", Math.abs(angleToTarget));
                dashboardTelemetry.addData("Needed Velocity", neededVelocity);

                dashboardTelemetry.addData("Heading (Z)", heading);

                dashboardTelemetry.addData("Target Flywheel Vel", targetFlywheelVel);
                dashboardTelemetry.addData("Actual Flywheel Velocity", flywheel.getVelocity());

                dashboardTelemetry.addData("Last Snapshot Save #", lastFileSaved);



                dashboardTelemetry.update();

            }
            //Tells it to update the field view
            drawRobot(xPos, yPos, heading);



    //      ---------------------assign flywheel velocity---------------------
            if (!gamepad1.b && false) {
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

            if(targetFlywheelVel != 0) {
                flywheel.setPower(Math.max(pid(f_kP, f_kI, f_kD, oldFlywheelDist, oldTime, targetFlywheelVel - flywheelVel, runtime.milliseconds(), "f"), -0.5));
            } else {
                f_integralSum = 0;
                flywheel.setPower(0);
            }
    //      ---------------------assign power to drivetrain motors---------------------

            double leftfrontPower = y - x - rot;
            double rightfrontPower = y + x + rot;
            double leftbackPower = y + x - rot;
            double rightbackPower = y - x + rot;

            frontleft.setPower(leftfrontPower);
            frontright.setPower(rightfrontPower);
            backleft.setPower(leftbackPower);
            backright.setPower(rightbackPower);



            //remember last frame's values for pid
            oldRot = angleToTarget;
            oldTime = runtime.milliseconds();
            oldFlywheelDist = targetFlywheelVel - flywheelVel;
        }
    }

    public void drawRobot(double x, double y, double heading) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        //draw square for robot lol

        double theta = Math.toRadians(heading);

        double[] xCords = {9, 9, -9, -9, 0, 0, 0};
        double[] yCords = {9, -9, -9, 9, 9, 12, 9};

        double cos = Math.cos(theta);
        double sin = Math.sin(theta);


        for (int i = 0; i < 7; i++) {
            double xCord = xCords[i];
            double yCord = yCords[i];

            // Apply rotation
            double rotatedX = xCord * cos - yCord * sin;
            double rotatedY = xCord * sin + yCord * cos;

            // Translate back to center (cx, cy)
            rotatedX += x;
            rotatedY += y;

            xCords[i] = rotatedX;
            yCords[i] = rotatedY;

        }
        fieldOverlay.strokePolygon(xCords, yCords);

        fieldOverlay.strokeLine(x, y, -70, 70); //draw shot distance line

        dashboard.sendTelemetryPacket(packet);
    }

    public double pid(double proportion, double integral, double dampening, double distOld, double tOld, double distNew, double tNew, String letterCode){

        double p = distNew * proportion;

        double i = 0;

        if(letterCode == "r"){
            r_integralSum += (tNew-tOld)*((distOld+distNew)/2);
            i = r_integralSum * integral;
        }
        else if(letterCode == "f"){
            f_integralSum += (tNew-tOld)*((distOld+distNew)/2);
            i = f_integralSum * integral;
        }


        double vel = (distNew - distOld)/(tNew - tOld);
        double d = -vel * dampening;

        return p + i + d;
    }


    public void downloadSnapshot() { //downloads most recent limelight snapshot into microSD in CH
        try {
            //reads the filenames
            URL fileNamesUrl = new URL("http://172.29.0.1:5801/snapshots/snapshotmanifest.txt");
            BufferedReader reader = new BufferedReader(
                    new InputStreamReader(fileNamesUrl.openStream())
            );

            StringBuilder content = new StringBuilder();
            String line;

            //cycle through filenames (stored in string)
            while ((line = reader.readLine()) != null) {
                content.append(line);
            }

            reader.close();

            //isolate last file
            String result = content.toString();

            result = result.replace("[", "")
                    .replace("]", "")
                    .replace("\"", "");

            String[] parts = result.split(",");

            String last = parts[parts.length - 1];
            dashboardTelemetry.addLine(last);

            URL url = new URL("http://172.29.0.1:5801/snapshots/min_" + last);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection();
            connection.connect();

            InputStream input = connection.getInputStream();

            lastFileSaved++;
             File file = new File(Environment.getExternalStorageDirectory().getPath(), "FIRST/snapshots/snapshot" + String.valueOf(lastFileSaved) + ".png");

             FileOutputStream output = new FileOutputStream(file);

             byte[] buffer = new byte[4096];
             int bytesRead;

             while ((bytesRead = input.read(buffer)) != -1) {
                 output.write(buffer, 0, bytesRead);
             }

             output.close();
             input.close();




        } catch (Exception e) {
            e.printStackTrace();

            dashboardTelemetry.addLine("ERROR: " + e.toString());
        }
    }
}