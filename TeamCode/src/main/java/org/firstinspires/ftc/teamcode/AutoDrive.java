package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class AutoDrive {
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;

    private DcMotor leftMotor1;
    private DcMotor leftMotor2;
    private DcMotor rightMotor1;
    private DcMotor rightMotor2;

    private Telemetry telemetry;
//
//    private DifferentialDrive differentialDrive;

    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;
    private BooleanSupplier slowModeInput;

    // creates a new Drive
    public AutoDrive(HardwareMap hardwareMap,
                     Telemetry telemetry,
                     DoubleSupplier forwardInput,
                     DoubleSupplier rotationInput,
                     BooleanSupplier slowModeInput) {
        this.telemetry = telemetry;
        this.forwardInput = forwardInput;
        this.rotationInput = rotationInput;
        this.slowModeInput = slowModeInput;
//
//        this.leftMotors = new MotorGroup(
//                new Motor(hardwareMap, "FrontLeftMotor"),
//                new Motor(hardwareMap, "BackLeftMotor"));
//        this.rightMotors = new MotorGroup(
//                new Motor(hardwareMap, "FrontRightMotor"),
//                new Motor(hardwareMap, "BackRightMotor"));

        leftMotor1 = hardwareMap.dcMotor.get("FrontLeftMotor");
        leftMotor2 = hardwareMap.dcMotor.get("BackLeftMotor");
        rightMotor1 = hardwareMap.dcMotor.get("FrontRightMotor");
        rightMotor2 = hardwareMap.dcMotor.get("BackRightMotor");

        // reset encoder counts kept by motors.
        leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        this.differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    public AutoDrive(HardwareMap hardwareMap, Telemetry telemetry) {
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



        telemetry.addData("Left Drive Power", leftMotors.get());
        telemetry.addData("Right Drive Power", rightMotors.get());
    }

    private void setLeftPower(double power) {
        leftMotor1.setPower(power);
        leftMotor2.setPower(power);
    }

    private void setRightPower(double power) {
        rightMotor1.setPower(power);
        rightMotor2.setPower(power);
    }
}

