package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class DashboardSucks {
    public static double TURN_TICKS = 1500;

    private Telemetry telemetry;

    public DashboardSucks(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
