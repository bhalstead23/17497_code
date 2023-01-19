package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "AutononmousTesting", group = "Linear Opmode")
public class AutonomousMode extends LinearOpMode {
    // Declare OpMode members.
//    private Drive drive;
//    private Elevator elevator;
//    private Intake intake;
    private Arm arm;

    private TurnConstantHolder ds;

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

    private Motor[] armMotors;

    private void initializeHardware() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        //drive = new Drive(hardwareMap, telemetry);
//        elevator = new Elevator(hardwareMap, telemetry);
        arm = new Arm(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);

        ds = new TurnConstantHolder(telemetry);

        colorSensor = hardwareMap.colorSensor.get("Color");

        backLeft = hardwareMap.dcMotor.get("BackLeftMotor");
        frontLeft = hardwareMap.dcMotor.get("FrontLeftMotor");
        backRight = hardwareMap.dcMotor.get("BackRightMotor");
        frontRight = hardwareMap.dcMotor.get("FrontRightMotor");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        allMotors = new DcMotor[] {backLeft, frontLeft, backRight, frontRight};
        leftMotors = new DcMotor[] {backLeft, frontLeft};
        rightMotors = new DcMotor[] {backRight, frontRight};

        armMotors = new Motor[] { arm.leftArm, arm.rightArm};

        for (DcMotor motor : allMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    private int getCurrColor() {
        int[] colors = new int[] {colorSensor.red(), colorSensor.green(), colorSensor.blue()};
        String[] colorNames = new String[] {"red", "green", "blue"};

        int maxI = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[maxI]) {
                maxI = i;
            }
        }

        String currColor = colorNames[maxI];

        telemetry.addData("Reading Color", currColor);
        telemetry.update();

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

        for (DcMotor motor : leftMotors) motor.setTargetPosition(leftTicks);

        for (DcMotor motor : rightMotors) motor.setPower(rightSpeed);

        for (DcMotor motor : rightMotors) motor.setTargetPosition(rightTicks);

        for (DcMotor motor : allMotors) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status:", "Starting path");
        telemetry.update();

        while (backLeft.isBusy() || backRight.isBusy()) {
            telemetry.addData("Path:", "Moving " + leftTicks + " left ticks " + rightTicks + " right ticks");
            telemetry.update();
        }

        for (DcMotor motor : allMotors) motor.setPower(0);

        telemetry.addData("Path:", "Done!");
        telemetry.update();
    }

    private void moveArm(double speed, int ticks) {
        for (Motor motor : armMotors) {
            motor.setRunMode(Motor.RunMode.PositionControl);
            motor.setPositionCoefficient(TurnConstantHolder.AUTO_P_COEFF);
            motor.resetEncoder();
        }

        for (Motor motor : armMotors) {
            motor.set(0);
            motor.setTargetPosition(ticks);
        }

        while(!armMotors[0].atTargetPosition()) {
            for (Motor motor: armMotors) {
                motor.set(speed);
            }
        }

        for (Motor motor : armMotors) motor.stopMotor();
    }

    private void turnLeft(double speed) {
        int TURN_TICKS = (int) TurnConstantHolder.TURN_TICKS;
        drive(-speed, speed, TURN_TICKS, TURN_TICKS);
    }

    private void turnRight(double speed) {
        int TURN_TICKS = (int) TurnConstantHolder.TURN_TICKS;
        drive(speed, -speed, TURN_TICKS, TURN_TICKS);
    }

    private void readColorAndPark() {
        int colorIndex = getCurrColor();

        telemetry.addData("Status", "Performing auto #" + (colorIndex + 1) + ".");
        telemetry.update();

        if (colorIndex == 0) { // red
            turnLeft(AUTO_SPEED);
            driveForward(AUTO_SPEED, ONE_SQUARE_INCHES+2);
        } else if (colorIndex == 1) { // green
//            driveForward(0.5, 23.5);
        } else { // blue
            turnRight(AUTO_SPEED);
            driveForward(AUTO_SPEED, ONE_SQUARE_INCHES);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

//        driveForward(AUTO_SPEED, ONE_SQUARE_INCHES);

        moveArm(0.1, -300);

//        readColorAndPark();

    }

    private static final int MOTOR_TICKS = 1440;
    private static final double ONE_SQUARE_INCHES = 23.5;
    private static final double WHEEL_CIRCUMFERENCE = 4.0 * 2 * Math.PI;
    private static final double AUTO_SPEED = 0.25;
}
