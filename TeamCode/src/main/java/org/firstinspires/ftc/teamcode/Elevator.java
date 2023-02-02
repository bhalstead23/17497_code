package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class Elevator {
    private Motor upFwdMotor;
    private Motor upRevMotor;
    private MotorGroup elevatorMotors;

    private Telemetry telemetry;
    private DoubleSupplier upInput;
    private DoubleSupplier downInput;
    private BooleanSupplier encoderResetInput;
    private BooleanSupplier autoHighInput;
    private BooleanSupplier autoZeroInput;

    public static double kp = 0.01;
    public static double ki = 0;
    public static double kd = 0;

    public static double ffWeight = 0.1;
    public static double stepSize = 5;
    public static double highPosition = 2500;

    public static double ks = 0; //static gain
    public static double kg = 0; //gravity gain
    public static double kv = 0; //velocity gain
//    public static double ka = 0; //acceleration gain

    private PIDController pidController;
    private ElevatorFeedforward feedforward;

    private double target;
    private double output;
    private double pidOutput;
    private double ffOutput;

    private double position;

    public Elevator(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            DoubleSupplier upInput,
            DoubleSupplier downInput,
            BooleanSupplier encoderResetInput,
            BooleanSupplier autoHighInput,
            BooleanSupplier autoZeroInput
    ) {
        this.telemetry = telemetry;
        this.upInput = upInput;
        this.downInput = downInput;
        this.encoderResetInput = encoderResetInput;
        this.autoHighInput = autoHighInput;
        this.autoZeroInput = autoZeroInput;

        upFwdMotor = new Motor(hardwareMap, "RightElevator");
        upRevMotor = new Motor(hardwareMap, "LeftElevator");
        upRevMotor.setInverted(true);
        elevatorMotors = new MotorGroup(upFwdMotor, upRevMotor);

        target = 0;

        pidController = new PIDController(kp, ki, kd);
        feedforward = new ElevatorFeedforward(ks, kg, kv);
    }

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null, null, null, null);
    }

    public void setTarget(double target) {
        this.target = target;
        pidController.setSetPoint(target);
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
        double input;
        if (position < 0) {
            input = upInput.getAsDouble();
        } else {
            input = upInput.getAsDouble() - downInput.getAsDouble();
        }

        input = upInput.getAsDouble() - downInput.getAsDouble();

        if (encoderResetInput.getAsBoolean()) {
            elevatorMotors.resetEncoder();
        }

        // How often does this loop?
        double positionChange = stepSize * input; //Math.signum(input) * Math.pow(input, 2);

        if(autoHighInput.getAsBoolean()){
            setTarget(highPosition);
        } else if(autoZeroInput.getAsBoolean()){
            setTarget(0);
        } else {
            setTarget(target + positionChange); // do we want something more intricate?
        }

//        boolean atSetPoint = pidController.atSetPoint();
        //if (!atSetPoint) { // only runs if arm is out of position
        pidOutput = pidController.calculate(position, target);
        // "output" must be rads/sec
        ffOutput = ffWeight * feedforward.calculate(input);
        output = pidOutput + ffOutput; // should this be scaled somehow? it can exceed 1 right now (?)
        elevatorMotors.set(output);
        // }

        handleTelemetry();
        telemetry.addData("Elevator Motor Power", output);
    }

    public void handleTelemetry() {
        telemetry.addData("Elevator Target", target);
        telemetry.addData("Elevator output", output);
        telemetry.addData("Elevator Output * 1000", output * 1000);
        telemetry.addData("Elevator pidOutput * 1000", pidOutput * 1000);
        telemetry.addData("Elevator ffOutput * 1000", ffOutput * 1000);
        telemetry.addData("Elevator Position", upFwdMotor.getCurrentPosition());
    }
}
