package commandbased;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    private Telemetry telemetry;

    private MotorGroup elevatorMotors;

    private DoubleSupplier upInput;

    private double height;
    private double target;

    // creates a new Drive
    public ElevatorSubsystem(
            HardwareMap hardwareMap,
            String leftMotorName,
            String rightMotorName,
            Telemetry telemetry
    ) {
        Motor leftMotor = new Motor(hardwareMap, leftMotorName);
        Motor rightMotor = new Motor(hardwareMap, rightMotorName);
        rightMotor.setInverted(true);
        this.elevatorMotors = new MotorGroup(leftMotor, rightMotor);

        elevatorMotors.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public Command joystickElevatorCommand(DoubleSupplier upInput) {
        return new RunCommand(() -> {
            setTarget(target + 10 * upInput.getAsDouble());
        });
    }

    public void periodic() {
        elevatorMotors.setTargetPosition((int) Math.round(target));
        telemetry.addData(
                "Left Elevator Motor Position",
                elevatorMotors.getPositions().get(0)
        );
    }

}
