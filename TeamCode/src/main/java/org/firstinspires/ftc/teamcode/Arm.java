package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class Arm {
    private Motor rightArm;
    private Motor leftArm;
    private MotorGroup armMotors;

    private Telemetry telemetry;

    private DoubleSupplier rotationInput;

    private PIDFController pidfController;
    private ArmFeedforward feedforward;

    public static double kp = 0.05;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;

    public static double ks = 0; //static gain
    public static double kcos = 0; //gravity gain
    public static double kv = 0; //velocity gain
    public static double ka = 0; //acceleration gain

    private double target;
    private double output;
    private double angle;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry, DoubleSupplier rotationInput) {
        this.rotationInput = rotationInput;
        this.telemetry = telemetry;

        rightArm = new Motor(hardwareMap, "RightArm");
        leftArm = new Motor(hardwareMap, "LeftArm");
        rightArm.setInverted(true);
        armMotors = new MotorGroup(rightArm, leftArm);

        pidfController = new PIDFController(kp, ki, kd, kf);
        feedforward = new ArmFeedforward(ks, kcos, kv, ka);
    }

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    public void setTarget(double target) {
        this.target = target;
        pidfController.setSetPoint(target);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        setTarget(target + 10 * rotationInput.getAsDouble()); // do we want something more intricate?
        angle = 2.0 * Math.PI * rightArm.getCurrentPosition() / 1440.0; // radians

        if (!pidfController.atSetPoint()) { // only runs if arm is out of position
            double pidOutput = pidfController.calculate(rightArm.getCurrentPosition());
            double ff = kf * feedforward.calculate(angle, output);
            output = pidOutput + ff; // should this be scaled somehow? it can exceed 1 right now (?)
            armMotors.set(output);
        }

        telemetry.addData("Arm Target", target);
        telemetry.addData("Output", output);
        telemetry.addData("Arm Position", rightArm.getCurrentPosition());
        telemetry.addData("Arm Angle", angle);
    }
}
