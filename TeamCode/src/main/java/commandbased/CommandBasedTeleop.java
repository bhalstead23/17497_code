package commandbased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Command-Based TeleOp", group = "Iterative Opmode")
public class CommandBasedTeleop extends OpMode {
    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private CommandScheduler scheduler;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    private Command joystickDriveCommand;
    private Command joystickArmCommand;
    private Command joystickElevatorCommand;

    public void init() {
        scheduler = CommandScheduler.getInstance();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        Robot robot = new Robot(hardwareMap, telemetry);

        joystickDriveCommand = robot.drivetrainSubsystem.joystickDriveCommand(
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX()
        );
        joystickArmCommand = robot.armSubsystem.joystickArmCommand(() -> driverGamepad.getRightX());
        joystickElevatorCommand = robot.elevatorSubsystem.joystickElevatorCommand(
                () -> driverGamepad.getRightY()
        );

        // adds subsystem periodic() methods to scheduler.run()
        scheduler.registerSubsystem(
                robot.drivetrainSubsystem,
                robot.elevatorSubsystem,
                robot.armSubsystem,
                robot.intakeSubsystem
        );

        scheduler.schedule(joystickDriveCommand, joystickElevatorCommand, joystickArmCommand);
    }

    public void loop() {
        scheduler.run();
    }
}
