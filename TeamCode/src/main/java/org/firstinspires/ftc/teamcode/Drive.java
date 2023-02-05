package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Drive {
    private Telemetry telemetry;
    private PIDController pidController;

    private CustomDifferentialDrive differentialDrive;

    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;
    private BooleanSupplier slowModeInput;

    public MotorGroup leftMotors;
    public MotorGroup rightMotors;

    // creates a new Drive
    public Drive(HardwareMap hardwareMap,
                 Telemetry telemetry,
                 DoubleSupplier forwardInput,
                 DoubleSupplier rotationInput,
                 BooleanSupplier slowModeInput) {
        this.telemetry = telemetry;
        this.forwardInput = forwardInput;
        this.rotationInput = rotationInput;
        this.slowModeInput = slowModeInput;

        this.leftMotors = new MotorGroup(
                new Motor(hardwareMap, "FrontLeftMotor"),
                new Motor(hardwareMap, "BackLeftMotor"));
        this.rightMotors = new MotorGroup(
                new Motor(hardwareMap, "FrontRightMotor"),
                new Motor(hardwareMap, "BackRightMotor"));

        this.differentialDrive = new CustomDifferentialDrive(leftMotors, rightMotors);
    }

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null, null);
    }

//    public void motorForward(int ticks) {
//        // set the run mode
//        leftMotors.setRunMode(Motor.RunMode.PositionControl);
//        rightMotors.setRunMode(Motor.RunMode.PositionControl);
//        telemetry.addData("left run mode", leftMotors.get)
//
//        // set and get the position coefficient
//        leftMotors.setPositionCoefficient(0.05);
//        rightMotors.setPositionCoefficient(0.05);
//        //double kP = motorGroup.getPositionCoefficient();
//
//        // set the target position
//        leftMotors.setTargetPosition(ticks);      // an integer representing
//        rightMotors.setTargetPosition(ticks);      // an integer representing
//        // desired tick count
//
//        leftMotors.set(0);
//        rightMotors.set(0);
//
//        // set the tolerance
//        leftMotors.setPositionTolerance(13.6);   // allowed maximum error
//        rightMotors.setPositionTolerance(13.6);   // allowed maximum error
//
//        // perform the control loop
//        while (!leftMotors.atTargetPosition()) {
//            leftMotors.set(0.75);
//            rightMotors.set(0.75);
//            telemetry.addData("front left position", leftMotors.getPositions().get(0));
//            telemetry.addData("front right position", rightMotors.getPositions().get(0));
//            telemetry.update();
//        }
//        leftMotors.stopMotor(); // stop the motor
//        rightMotors.stopMotor(); // stop the motor
//    }
//
//    public void goForwardTicks(int ticks){
//        motorForward(ticks);
//    }

    public void autoInit() {
//        goForwardTicks(1000);
    }

    public void autoPeriodic() throws InterruptedException {
        differentialDrive.arcadeDrive(1, 0);
        differentialDrive.arcadeDrive(0, 0);
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        double sign = forwardInput.getAsDouble() < 0  ? -1 : 1;
        double sign2 = rotationInput.getAsDouble() < 0  ? -1 : 1;
        double forwardValue = sign * Math.pow(forwardInput.getAsDouble(),2);
        double rotationValue = sign2 * Math.pow(rotationInput.getAsDouble(),2);

        if (slowModeInput.getAsBoolean()) {
            forwardValue /= 2;
            rotationValue /= 2;
        }

        // controls the drivetrain according to the doubles supplied by forward and rotation
        differentialDrive.arcadeDrive(forwardValue, rotationValue);

        telemetry.addData("Left Drive Power", leftMotors.get());
        telemetry.addData("Right Drive Power", rightMotors.get());
    }
}
