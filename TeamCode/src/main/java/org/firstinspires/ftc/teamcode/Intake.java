package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private HardwareMap hardwareMap;
    private Gamepad gamepad;

    private CRServo LeftServos;
    private CRServo RightServos;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        LeftServos = hardwareMap.crservo.get("ServoPair1");
        RightServos = hardwareMap.crservo.get("ServoPair2");

        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, null, null);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        
    }

    public void teleopPeriodic() {
            if (gamepad.dpad_up) {
                LeftServos.setPower(1);
                RightServos.setPower(-1);
            } else if (gamepad.dpad_down) {
                LeftServos.setPower(-1);
                RightServos.setPower(1);
            } else {
                LeftServos.setPower(0);
                RightServos.setPower(0);
            }

    }
}
