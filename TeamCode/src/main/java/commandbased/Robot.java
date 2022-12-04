package commandbased;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot extends com.arcrobotics.ftclib.command.Robot {
    protected DrivetrainSubsystem drivetrainSubsystem;
    protected ArmSubsystem armSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected IntakeSubsystem intakeSubsystem;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drivetrainSubsystem = new DrivetrainSubsystem(
                hardwareMap,
                "FrontLeftMotor",
                "BackLeftMotor",
                "FrontRightMotor",
                "BackRightMotor",
                telemetry
        );
        armSubsystem = new ArmSubsystem(
                hardwareMap,
                "LeftArm",
                "RightArm",
                telemetry
        );
        elevatorSubsystem = new ElevatorSubsystem(
                hardwareMap,
                "LeftElevator",
                "RightElevator",
                telemetry
        );
        intakeSubsystem = new IntakeSubsystem(
                hardwareMap,
                "FrontLeftServo",
                "BackLeftServo",
                "FrontRightServo",
                "BackRightServo",
                telemetry
        );
    }
}
