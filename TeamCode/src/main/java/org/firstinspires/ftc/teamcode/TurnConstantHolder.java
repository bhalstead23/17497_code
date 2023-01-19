package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class TurnConstantHolder {
    public static double TURN_TICKS = 700;

    public static double AUTO_P_COEFF = 0.05;

    private Telemetry telemetry;

    public TurnConstantHolder(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
