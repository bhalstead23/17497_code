package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class Elevator {
    private DcMotor LeftElevator;
    private DcMotor RightElevator;
    private Motor rightElevator;
    private Motor leftElevator;
    private MotorGroup elevatorMotors;

    private Telemetry telemetry;
    private DoubleSupplier upInput;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry, DoubleSupplier upInput) {
        this.telemetry = telemetry;
        this.upInput = upInput;

        rightElevator = new Motor(hardwareMap, "RightElevator");
        leftElevator = new Motor(hardwareMap, "LeftElevator");
        rightElevator.setInverted(true);
        elevatorMotors = new MotorGroup(rightElevator, leftElevator);
    }

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    public void setHeight(float height) {

    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
    }

    public void teleopPeriodic() {
        elevatorMotors.set(upInput.getAsDouble());

        telemetry.addData("elevatorMotorPower", elevatorMotors.get());
    }
}
