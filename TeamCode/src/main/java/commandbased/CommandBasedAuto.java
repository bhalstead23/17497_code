package commandbased;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CommandBasedAuto extends CommandOpMode {
    private FtcDashboard dashboard;
    private ElapsedTime runtime = new ElapsedTime();
    private CommandScheduler scheduler;

    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Robot robot = new Robot(hardwareMap, telemetry);

        register(
                robot.drivetrainSubsystem,
                robot.elevatorSubsystem,
                robot.armSubsystem,
                robot.drivetrainSubsystem
        );
    }
}
