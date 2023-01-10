package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@SuppressWarnings("SpellCheckingInspection")
@Config
public class TurnConstantHolder {
    public static double TURN_TICKS = 700;

    private Telemetry telemetry;

    public TurnConstantHolder(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}
