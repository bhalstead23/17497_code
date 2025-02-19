package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class AutoConfigHolder {
    public static double TURN_TICKS = 700;

    public static double AUTO_P_COEFF = 0.05;

    private Telemetry telemetry;

    public AutoConfigHolder(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
