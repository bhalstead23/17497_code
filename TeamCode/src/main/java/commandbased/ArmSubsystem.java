package commandbased;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {
    private Telemetry telemetry;

    private MotorGroup armMotors;

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

    // creates a new Drive
    public ArmSubsystem(
            HardwareMap hardwareMap,
            String leftMotor,
            String rightMotor,
            Telemetry telemetry
    ) {
        Motor left = new Motor(hardwareMap, leftMotor);
        Motor right = new Motor(hardwareMap, rightMotor);
        right.setInverted(true);
        armMotors = new MotorGroup(left, right);

    }

    public void setTarget(double target) {
        this.target = target;
        pidfController.setSetPoint(target);
    }

    public Command joystickArmCommand(DoubleSupplier rotationInput) {
        return new RunCommand(() -> {
            setTarget(target + 10 * rotationInput.getAsDouble());
        });
    }

    public void periodic() {
        double leftPosition = armMotors.getPositions().get(0);

        telemetry.addData("Left Drive Power", leftPosition);

        angle = 2.0 * Math.PI * leftPosition / 1440.0; // radians

        if (!pidfController.atSetPoint()) { // only runs if arm is out of position
            double pidOutput = pidfController.calculate(leftPosition, target);
            double ff = kf * feedforward.calculate(angle, output);
            output = pidOutput + ff; // should this be scaled somehow? it can exceed 1 right now (?)
            armMotors.set(output);
        }

        telemetry.addData("Arm Target", target);
        telemetry.addData("Output", output);
        telemetry.addData("Arm Position", leftPosition);
        telemetry.addData("Arm Angle", angle);
    }

}
