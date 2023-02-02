package org.firstinspires.ftc.teamcode;//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class CustomDifferentialDrive extends RobotDrive {
    public static final double kDefaultRightSideMultiplier = -1.0D;
    private Motor[] motors;
    private double leftSideMultiplier = 1.0D;
    private double rightSideMultiplier = -1.0D;

    public CustomDifferentialDrive(Motor... myMotors) {
        this.motors = myMotors;
        this.setRightSideInverted(true);
    }

    public CustomDifferentialDrive(boolean autoInvert, Motor... myMotors) {
        this.motors = myMotors;
        this.setRightSideInverted(autoInvert);
    }

    public boolean isRightSideInverted() {
        return this.rightSideMultiplier == -1.0D;
    }

    public void setRightSideInverted(boolean isInverted) {
        this.rightSideMultiplier = isInverted ? -1.0D : 1.0D;
    }

    public void setLeftSideMultiplier(double value) {
        this.leftSideMultiplier = value;
    }

    public void stop() {
        Motor[] var1 = this.motors;
        int var2 = var1.length;

        for(int var3 = 0; var3 < var2; ++var3) {
            Motor x = var1[var3];
            x.stopMotor();
        }

    }

    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        forwardSpeed = this.clipRange(forwardSpeed);
        turnSpeed = this.clipRange(turnSpeed);
        double[] wheelSpeeds = new double[2];
        wheelSpeeds[MotorType.kLeft.value] = forwardSpeed + turnSpeed;
        wheelSpeeds[MotorType.kRight.value] = forwardSpeed - turnSpeed;
        this.normalize(wheelSpeeds);
        this.motors[MotorType.kLeft.value].set(this.maxOutput * wheelSpeeds[0]);
        this.motors[MotorType.kRight.value].set(this.rightSideMultiplier * this.maxOutput * wheelSpeeds[1]);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed, boolean squareInputs) {
        forwardSpeed = squareInputs ? this.clipRange(this.squareInput(forwardSpeed)) : this.clipRange(forwardSpeed);
        turnSpeed = squareInputs ? this.clipRange(this.squareInput(turnSpeed)) : this.clipRange(turnSpeed);
        this.arcadeDrive(forwardSpeed, turnSpeed);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed = this.clipRange(leftSpeed);
        rightSpeed = this.clipRange(rightSpeed);
        double[] wheelSpeeds = new double[2];
        wheelSpeeds[MotorType.kLeft.value] = leftSpeed;
        wheelSpeeds[MotorType.kRight.value] = rightSpeed;
        this.normalize(wheelSpeeds);
        this.motors[MotorType.kLeft.value].set(wheelSpeeds[0] * this.leftSideMultiplier * this.maxOutput);
        this.motors[MotorType.kRight.value].set(wheelSpeeds[1] * this.rightSideMultiplier * this.maxOutput);
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean squareInputs) {
        leftSpeed = squareInputs ? this.clipRange(this.squareInput(leftSpeed)) : this.clipRange(leftSpeed);
        rightSpeed = squareInputs ? this.clipRange(this.squareInput(rightSpeed)) : this.clipRange(rightSpeed);
        this.tankDrive(leftSpeed, rightSpeed);
    }
}
