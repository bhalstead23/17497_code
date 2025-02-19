package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class Arm {
    public Motor rightArm;
    public Motor leftArm;
    private MotorGroup armMotors;

    private Telemetry telemetry;

    private DoubleSupplier rotationInput;
    private BooleanSupplier autoHighInput;
    private BooleanSupplier autoZeroInput;

    private PIDController pidController;
    private ArmFeedforward feedforward;

    private static double AUTO_INIT_ANGLE = Math.PI / 4;
    private static double highPosition = 595;

    public static double kp = 0.02;
    public static double ki = 0.001;
    public static double kd = 0;

    public static double ffWeight = 0.0;

    public static double ks = 0.1; //static gain
    public static double kcos = 0; //gravity gain
    public static double kv = 0.1; //velocity gain
//    public static double ka = 0; //acceleration gain

    private double target;
    private double output;
    private double pidOutput;
    private double ffOutput;
    private double angle;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry, DoubleSupplier rotationInput, BooleanSupplier autoHighInput, BooleanSupplier autoZeroInput) {
        this.rotationInput = rotationInput;
        this.autoHighInput = autoHighInput;
        this.autoZeroInput = autoZeroInput;
        this.telemetry = telemetry;

        rightArm = new Motor(hardwareMap, "RightArm");
        leftArm = new Motor(hardwareMap, "LeftArm");
        rightArm.setInverted(true);
        armMotors = new MotorGroup(rightArm, leftArm);

        pidController = new PIDController(kp, ki, kd);
        feedforward = new ArmFeedforward(ks, kcos, kv);

    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null, null);
    }

    public void setTarget(double target) {
        this.target = target;
        pidController.setSetPoint(target);
    }

    public void setTargetRadians(double targetRads) {
        setTarget(radiansToTicks(targetRads));
    }

    // note: still requires telemetry.update() in main opmode
    public void handleTelemetry() {
        telemetry.addData("Arm Target", target);
        telemetry.addData("output", output);
        telemetry.addData("Output * 1000", output * 1000);
        telemetry.addData("pidOutput * 1000", pidOutput * 1000);
        telemetry.addData("ffOutput * 1000", ffOutput * 1000);
        telemetry.addData("Arm Position", rightArm.getCurrentPosition());
        telemetry.addData("Arm Angle", angle);
        telemetry.addData("Arm Angle * 100", angle * 100);
        telemetry.addData("Arm Angle * 1000", angle * 1000);
    }

    public double ticksToRadians(double ticks) {
        return 2.0 * Math.PI * ticks / 1440.0;
    }

    public double radiansToTicks(double angle) {
        return 1440.0 * angle / (2.0 * Math.PI);
    }

    public void autoInit() {
        armMotors.resetEncoder();
        setTarget(radiansToTicks(AUTO_INIT_ANGLE));
    }

    public void autoPeriodic() {
        angle = 2.0 * Math.PI * rightArm.getCurrentPosition() / 1440.0; // radians
        boolean atSetPoint = pidController.atSetPoint();
        //if (!atSetPoint) { // only runs if arm is out of position
        pidOutput = pidController.calculate(rightArm.getCurrentPosition(), target);
        // "output" must be rads/sec
        ffOutput = ffWeight * feedforward.calculate(
                angle,
                0
        );
        output = pidOutput + ffOutput; // should this be scaled somehow? it can exceed 1 right now (?)
        armMotors.set(output);
        // }

        handleTelemetry();
    }

    public void teleopInit() {
        armMotors.resetEncoder();
        setTarget(rightArm.getCurrentPosition());
    }

    private void changePosition(double positionChange) {

        if(autoHighInput.getAsBoolean()){
            setTarget(highPosition);
        } else if(autoZeroInput.getAsBoolean()){
            setTarget(0);
        } else {
            setTarget(target + positionChange); // do we want something more intricate?
        }

        angle = 2.0 * Math.PI * rightArm.getCurrentPosition() / 1440.0; // radians
        boolean atSetPoint = pidController.atSetPoint();
        //if (!atSetPoint) { // only runs if arm is out of position
        pidOutput = pidController.calculate(rightArm.getCurrentPosition(), target);
        // "output" must be rads/sec
        ffOutput = ffWeight * feedforward.calculate(
                angle,
                rotationInput.getAsDouble() * 300.0 / 60.0 //"rads per second" but not really
        );
        output = pidOutput + ffOutput; // should this be scaled somehow? it can exceed 1 right now (?)
        armMotors.set(output);
        // }

        handleTelemetry();
    }

    public void teleopPeriodic() {
        // How often does this loop?
        double positionChange = 5 * Math.signum(rotationInput.getAsDouble()) * Math.pow(rotationInput.getAsDouble(), 2);
        changePosition(positionChange);
    }
}
