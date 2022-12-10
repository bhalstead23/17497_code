package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Intake {

    private HardwareMap hardwareMap;
//    private Gamepad gamepad;

    private CRServo ServoPair1;
    private CRServo ServoPair2;

    private BooleanSupplier intakeUp;
    private BooleanSupplier intakeDown;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, BooleanSupplier intakeUp, BooleanSupplier intakeDown) {
        ServoPair1 = hardwareMap.crservo.get("ServoPair1");
        ServoPair2 = hardwareMap.crservo.get("ServoPair2");

        this.hardwareMap = hardwareMap;
//        this.gamepad = gamepad1;
        this.intakeUp = intakeUp;
        this.intakeDown = intakeDown;
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null, null);
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, null, null, null);
    }

    public void autoInit() {

    }

    public void autoPeriodic() {

    }

    public void teleopInit() {
        
    }

    public void teleopPeriodic() {
        if (intakeUp.getAsBoolean()) {
            ServoPair1.setPower(1);
            ServoPair2.setPower(-1);
        } else if (intakeDown.getAsBoolean()) {
            ServoPair1.setPower(-1);
            ServoPair2.setPower(1);
        } else {
            ServoPair1.setPower(0);
            ServoPair2.setPower(0);
        }

//        if (gamepad.dpad_up) {
//            ServoPair1.setPower(1);
//            ServoPair2.setPower(-1);
//        } else if (gamepad.dpad_down) {
//            ServoPair1.setPower(-1);
//            ServoPair2.setPower(1);
//        } else {
//            ServoPair1.setPower(0);
//            ServoPair2.setPower(0);
//        }
    }
}
