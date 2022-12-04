package commandbased;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.DoubleSupplier;

public class DrivetrainSubsystem extends SubsystemBase {
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;

    private Telemetry telemetry;

    private DifferentialDrive differentialDrive;

    private DoubleSupplier forwardInput;
    private DoubleSupplier rotationInput;

    // creates a new Drive
    public DrivetrainSubsystem(
            HardwareMap hardwareMap,
            String flName,
            String blName,
            String frName,
            String brName,
            Telemetry telemetry
    ) {
        this.leftMotors = new MotorGroup(
                new Motor(hardwareMap, flName),
                new Motor(hardwareMap, blName));
        this.rightMotors = new MotorGroup(
                new Motor(hardwareMap, frName),
                new Motor(hardwareMap, brName));
        this.differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
    }


    public Command joystickDriveCommand(DoubleSupplier forwardInput, DoubleSupplier rotationInput) {
        return new RunCommand(() -> {
            differentialDrive.arcadeDrive(forwardInput.getAsDouble(), rotationInput.getAsDouble());
        });
    }

    public void driveForward(double speed) {
        differentialDrive.arcadeDrive(speed, 0.0);
    }

    public void periodic() {
        telemetry.addData("Left Drive Power", leftMotors.get());
        telemetry.addData("Right Drive Power", rightMotors.get());
    }

}
