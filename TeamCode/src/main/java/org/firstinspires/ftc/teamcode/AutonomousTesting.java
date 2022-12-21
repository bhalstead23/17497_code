package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutononmousTesting", group = "Linear Opmode")
public class AutonomousTesting extends LinearOpMode {
    // Declare OpMode members.
    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Arm arm;

    private FtcDashboard dashboard;

    private ColorSensor colorSensor;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
//
//    private FtcDashboard dashboard;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor[] motors;

    private void initializeHardware() {
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        //drive = new Drive(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        colorSensor = hardwareMap.colorSensor.get("Color");

        backLeft = hardwareMap.dcMotor.get("BackLeftMotor");
        frontLeft = hardwareMap.dcMotor.get("FrontLeftMotor");
        backRight = hardwareMap.dcMotor.get("BackRightMotor");
        frontRight = hardwareMap.dcMotor.get("FrontRightMotor");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotor[] {backLeft, frontLeft, backRight, frontRight};

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    private String getCurrColor() {
        int[] colors = new int[] {colorSensor.red(), colorSensor.blue(), colorSensor.green()};
        String[] colorNames = new String[] {"red", "blue", "green"};

        int maxI = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[maxI]) {
                maxI = i;
            }
        }

        String currColor = colorNames[maxI];

        telemetry.addData("CURR COLOR", currColor);
        telemetry.update();

        return currColor;
    }

    private void driveForward(double speed, int ticks) {
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for (DcMotor motor : motors) {
            motor.setPower(speed);
        }

        for (DcMotor motor : motors) {
            motor.setTargetPosition(ticks);
        }

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        telemetry.addData("Status:", "Starting path");
        telemetry.update();

        while (backLeft.isBusy()) {
            telemetry.addData("Path:", "Moving " + ticks + " ticks forward");
            getCurrColor();
            telemetry.update();
        }

        for (DcMotor motor : motors) {
            motor.setPower(0);
        }

        telemetry.addData("Path:", "Done!");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        driveForward(0.5, 1000);
    }
}
