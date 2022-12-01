package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.*;

@SuppressWarnings("SpellCheckingInspection")
public class Arm {
    private Motor rightArm;
    private Motor leftArm;
    private MotorGroup armMotors;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    private PIDFController pidfController;
    private ArmFeedforward feedforward;

    private double kp = 0.0005;
    private double ki = 0;
    private double kd = 0;
    private double kf = 0;

    private double ks = 1; //static gain
    private double kcos = 1; //gravity gain
    private double kv = 1; //velocity gain
    private double ka = 1; //acceleration gain

    private double target;

    public Arm(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
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
    }

    public String teleopPeriodic() {
//        armMotors.set(gamepad.right_stick_x);
        setTarget(target + 10 * gamepad.right_stick_x);
        if (!pidfController.atSetPoint()) {
            double output = pidfController.calculate(leftArm.getCurrentPosition());
            armMotors.set(output);
        }
        return target + ", " + leftArm.getCurrentPosition();
    }
}
