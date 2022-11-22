package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private DcMotor LeftElevator;
    private DcMotor RightElevator;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    public Elevator(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
    }

    public void setHeight(float height) {

    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {

    }
}
