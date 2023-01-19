package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

@Config
public class Intake {

    private HardwareMap hardwareMap;
//    private Gamepad gamepad;

    private SimpleServo grabFwdServo; // this servo runs forward (power > 0) to grab a cone
    private SimpleServo grabRevServo; // this servo runs reverse (power < 0) to grab a cone

    private BooleanSupplier grabInput;
    private BooleanSupplier releaseInput;

    private static double SERVO_STEP = 1;
    private static double SERVO_CLOSE_POSITION = 0.7;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, BooleanSupplier grabInput, BooleanSupplier releaseInput) {
        grabFwdServo = new SimpleServo(hardwareMap, "ServoPair2", 0, 180);
        grabRevServo = new SimpleServo(hardwareMap, "ServoPair1", 0, 180);

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
//            grabFwdServo.rotateByAngle(SERVO_STEP);
//            grabRevServo.rotateByAngle(-SERVO_STEP);
            grabFwdServo.setPosition(SERVO_CLOSE_POSITION);
        } else if (releaseInput.getAsBoolean()) {
//            grabFwdServo.rotateByAngle(-SERVO_STEP);
//            grabRevServo.rotateByAngle(SERVO_STEP);
            grabFwdServo.setPosition(0);
        }

    }
}
