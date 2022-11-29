package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;


public class Drive {
    private DcMotor FrontRightMotor;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
    }

    public void setPowers(double left, double right) {
        FrontLeftMotor.setPower(-left);
        BackLeftMotor.setPower(-left);
        FrontRightMotor.setPower(right);
        BackRightMotor.setPower(right);
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

    public Map teleopPeriodic() {
        double leftPower = gamepad.left_stick_y + gamepad.left_stick_x;
        double rightPower = gamepad.left_stick_y + -gamepad.left_stick_x;
        setPowers(leftPower, rightPower);
        Map telemetry = new HashMap<String, Double>();
        telemetry.put("leftPower", leftPower);
        telemetry.put("rightPower", rightPower);
        telemetry.put("left_stick_y", gamepad.left_stick_y);
        telemetry.put("left_stick_x", gamepad.left_stick_x);
        return telemetry;
    }
}
