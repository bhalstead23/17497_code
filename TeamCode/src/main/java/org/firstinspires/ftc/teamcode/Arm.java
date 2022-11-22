package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotor RightArm;
    private DcMotor LeftArm;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    public Arm(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
    }

    public void setAngle(float angle) {

    }

    public void teleopInit() {

    }
    public void teleopPeriodic() {

    }
}
