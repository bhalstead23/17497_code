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

@SuppressWarnings("SpellCheckingInspection")
@Config
public class Arm {
    private Motor rightArm;
    private Motor leftArm;
    private MotorGroup armMotors;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Telemetry telemetry;

    private PIDFController pidfController;
    private ArmFeedforward feedforward;

    public static double kp = 0.0005;
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

    public Arm(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
        this.telemetry = telemetry;
    }

    public void setTarget(double target) {
        double sp = target;
        this.target = target;
        pidfController.setSetPoint(sp);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        rightArm = new Motor(hardwareMap, "RightArm");
        leftArm = new Motor(hardwareMap, "LeftArm");
        rightArm.setInverted(true);
        armMotors = new MotorGroup(rightArm, leftArm);
        pidfController = new PIDFController(kp, ki, kd, kf);
        feedforward = new ArmFeedforward(ks, kcos, kv, ka);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        this.telemetry = dashboard.getTelemetry();

        target = -100;
    }

    public void teleopPeriodic() {
        setTarget(target + 10 * gamepad.right_stick_x);
        angle = rightArm.getCurrentPosition() / 1440.0;
        if (!pidfController.atSetPoint()) {
            output = pidfController.calculate(rightArm.getCurrentPosition());
            double ff = kf * feedforward.calculate(angle, output);
            armMotors.set(output + ff);
        }
        telemetry.addData("Arm Target", target);
        telemetry.addData("Output", output);
        telemetry.addData("Arm Position", rightArm.getCurrentPosition());
        telemetry.addData("Arm Angle", angle);
    }
}
