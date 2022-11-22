package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drive {
    private DcMotor FrontRightMotor;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
    }

    public void setPowers(float left, float right) {
        FrontLeftMotor.setPower(left);
        BackLeftMotor.setPower(left);
        FrontRightMotor.setPower(-right);
        BackRightMotor.setPower(-right);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

    }

    public void teleopPeriodic() {
        float leftPower = gamepad.left_stick_y + -gamepad.left_stick_x;
        float rightPower = gamepad.left_stick_y + gamepad.left_stick_x;
        setPowers(leftPower, rightPower);
    }
}
