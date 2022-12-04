package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Full Auto")
public class Auto extends LinearOpMode {

    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Arm arm;

    private FtcDashboard dashboard;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
    }
}
