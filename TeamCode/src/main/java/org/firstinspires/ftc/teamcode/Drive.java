package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class Drive {
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;

    private Telemetry telemetry;

    private DifferentialDrive differentialDrive;

    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;
    private BooleanSupplier slowModeInput;

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

        this.differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null, null);
    }

    public void autoInit() {

    }

    public void autoPeriodic() throws InterruptedException {
//        differentialDrive.arcadeDrive(1, 0);
//
//        differentialDrive.arcadeDrive(0, 0);
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
