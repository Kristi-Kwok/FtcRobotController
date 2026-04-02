package org.firstinspires.ftc.teamcode.HardwareCode;

public class PIDConfig {
    private double kP = 0.0001;
    private double kD = 0.0000;
    private double kI = 0;
    private double goalX = 0;


    public PIDConfig(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    public void setkP(double newKP) {
        kP = kP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }
}
