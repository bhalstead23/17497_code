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

    public void setPowers(float power) {
        LeftElevator.setPower(power);
        RightElevator.setPower(-power);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        LeftElevator = hardwareMap.get(DcMotor.class, "LeftElevator");
        RightElevator = hardwareMap.get(DcMotor.class, "RightElevator");
    }

    public void teleopPeriodic() {
        setPowers(gamepad.right_stick_y);
    }
}
