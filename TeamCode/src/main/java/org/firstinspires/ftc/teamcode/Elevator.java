package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Elevator {
    private Motor upFwdMotor;
    private Motor upRevMotor;
    private MotorGroup elevatorMotors;

    private Telemetry telemetry;
    private DoubleSupplier upInput;
    private DoubleSupplier downInput;
    private BooleanSupplier encoderResetInput;

    private double position;

    public Elevator(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            DoubleSupplier upInput,
            DoubleSupplier downInput,
            BooleanSupplier encoderResetInput
    ) {
        this.telemetry = telemetry;
        this.upInput = upInput;
        this.downInput = downInput;
        this.encoderResetInput = encoderResetInput;

        upFwdMotor = new Motor(hardwareMap, "RightElevator");
        upRevMotor = new Motor(hardwareMap, "LeftElevator");
        upRevMotor.setInverted(true);
        elevatorMotors = new MotorGroup(upFwdMotor, upRevMotor);
    }

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null, null);
    }

    public void setHeight(float height) {

    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        elevatorMotors.resetEncoder();
    }

    public void teleopPeriodic() {
        position = elevatorMotors.getPositions().get(0);
        double output;
        if (encoderResetInput.getAsBoolean()) {
            output = upInput.getAsDouble() - downInput.getAsDouble();
            elevatorMotors.resetEncoder();
        } else {
            if (position < 0) {
                output = upInput.getAsDouble();
            } else {
                output = upInput.getAsDouble() - downInput.getAsDouble();
            }
        }
        
        elevatorMotors.set(output);
        telemetry.addData("Elevator Motor Power", output);
    }
}
