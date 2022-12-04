package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;


public class Drive {
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;

    private Telemetry telemetry;

    private DifferentialDrive differentialDrive;

    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;

    // creates a new Drive
    public Drive(HardwareMap hardwareMap, Telemetry telemetry, DoubleSupplier forwardInput, DoubleSupplier rotationInput) {
        this.telemetry = telemetry;
        this.forwardInput = forwardInput;
        this.rotationInput = rotationInput;

        this.leftMotors = new MotorGroup(
                new Motor(hardwareMap, "FrontLeftMotor"),
                new Motor(hardwareMap, "BackLeftMotor"));
        this.rightMotors = new MotorGroup(
                new Motor(hardwareMap, "FrontRightMotor"),
                new Motor(hardwareMap, "BackRightMotor"));

        this.differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {


    }

    public void teleopPeriodic() {
        // controls the drivetrain according to the doubles supplied by forward and rotation
        differentialDrive.arcadeDrive(forwardInput.getAsDouble(), rotationInput.getAsDouble());

        telemetry.addData("Left Drive Power", leftMotors.get());
        telemetry.addData("Right Drive Power", rightMotors.get());
    }
}
