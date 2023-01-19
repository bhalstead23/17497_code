package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class Intake {

    private HardwareMap hardwareMap;
//    private Gamepad gamepad;

    private CRServo grabFwdServo; // this servo runs forward (power > 0) to grab a cone
    private CRServo grabRevServo; // this servo runs reverse (power < 0) to grab a cone

    private BooleanSupplier grabInput;
    private BooleanSupplier releaseInput;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, BooleanSupplier grabInput, BooleanSupplier releaseInput) {
        grabFwdServo = hardwareMap.crservo.get("ServoPair2");
        grabRevServo = hardwareMap.crservo.get("ServoPair1");

        this.hardwareMap = hardwareMap;
        this.grabInput = grabInput;
        this.releaseInput = releaseInput;
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
        if (grabInput.getAsBoolean()) {
            grabFwdServo.setPower(1);
            grabRevServo.setPower(-1);
        } else if (releaseInput.getAsBoolean()) {
            grabFwdServo.setPower(-1);
            grabRevServo.setPower(1);
        } else {
            grabFwdServo.setPower(0);
            grabRevServo.setPower(0);
        }
    }
}
