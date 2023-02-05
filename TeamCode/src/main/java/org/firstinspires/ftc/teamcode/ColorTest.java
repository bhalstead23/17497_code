package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Color Testing")
public class ColorTest extends OpMode {
    private Elevator elevator;
    private Intake intake;
    private Arm arm;

    private AutoConfigHolder ds;

    private FtcDashboard dashboard;

    private ColorSensor colorSensor;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;
    private DcMotor[] allMotors;

    private String state;

    private void initializeHardware() {
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        //drive = new Drive(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        ds = new AutoConfigHolder(telemetry);

        colorSensor = hardwareMap.colorSensor.get("Color");

        backLeft = hardwareMap.dcMotor.get("BackLeftMotor");
        frontLeft = hardwareMap.dcMotor.get("FrontLeftMotor");
        backRight = hardwareMap.dcMotor.get("BackRightMotor");
        frontRight = hardwareMap.dcMotor.get("FrontRightMotor");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotor[]{backLeft, frontLeft, backRight, frontRight};
        leftMotors = new DcMotor[]{backLeft, frontLeft};
        rightMotors = new DcMotor[]{backRight, frontRight};

        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    private int getCurrColor() {
        int[] colors = new int[]{colorSensor.red(), colorSensor.green(), colorSensor.blue()};
        String[] colorNames = new String[]{"red", "green", "blue"};

        int maxI = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[maxI]) {
                maxI = i;
            }
        }

        String currColor = colorNames[maxI];

        telemetry.addData("Reading Color", currColor);

        return maxI;
    }

    private void driveForward(double speed, double inches) {
        int ticks = (int) (inches / WHEEL_CIRCUMFERENCE * MOTOR_TICKS);
        drive(speed, speed, ticks, ticks);
    }

    private void drive(double leftSpeed, double rightSpeed, int leftTicks, int rightTicks) {
        if (leftSpeed * leftTicks < 0) {
            leftTicks *= -1;
        }

        if (rightSpeed * rightTicks < 0) {
            rightTicks *= -1;
        }

        for (DcMotor motor : allMotors) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        for (DcMotor motor : leftMotors) motor.setPower(leftSpeed);

        for (DcMotor motor : leftMotors) {
            motor.setTargetPosition(leftTicks);
        }

        for (DcMotor motor : rightMotors) motor.setPower(rightSpeed);

        for (DcMotor motor : rightMotors) motor.setTargetPosition(rightTicks);

        for (DcMotor motor : allMotors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Path:", "Moving " + leftTicks + " left ticks " + rightTicks + " right ticks");
//        telemetry.update();

//        while (backLeft.isBusy() || backRight.isBusy()) {
//            telemetry.addData("Path:", "Moving " + leftTicks + " left ticks " + rightTicks + " right ticks");
//            getCurrColor();
//            telemetry.update();
//        }
//
//        for (DcMotor motor : allMotors) motor.setPower(0);
//
//        telemetry.addData("Path:", "Done!");
//        telemetry.update();
    }

    private void turnLeft(double speed) {
        int TURN_TICKS = (int) AutoConfigHolder.TURN_TICKS;
        drive(-speed, speed, TURN_TICKS, TURN_TICKS);
    }

    private void turnRight(double speed) {
        int TURN_TICKS = (int) AutoConfigHolder.TURN_TICKS;
        drive(speed, -speed, TURN_TICKS, TURN_TICKS);
    }

    private void setState(String newState) {

        state = newState;

        telemetry.addData("State:", "newState");
//        telemetry.update();
    }

    @Override
    public void init() {
        initializeHardware();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        state = "DRIVE_TO_SIGNAL";
        arm.autoInit();
    }

    private boolean currentlyMoving() {
        return backLeft.isBusy() || backRight.isBusy();
    }

    @Override
    public void loop() {
        getCurrColor();
        telemetry.addData("State:", state);
        telemetry.update();
    }

    private static final int MOTOR_TICKS = 1440;
    private static final double ONE_SQUARE_INCHES = 23.5;
    private static final double WHEEL_CIRCUMFERENCE = 4.0 * 2 * Math.PI;
    private static final double AUTO_SPEED = 0.25;

}
