package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Color Sensor Test", group = "Iterative Opmode")
public class ColorSensorTest extends OpMode {
    // Declare OpMode members.
    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Arm arm;

    private ColorSensor colorSensor;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;
//
//    private FtcDashboard dashboard;

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();

        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new Drive(
                hardwareMap,
                telemetry,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)
        );
        elevator = new Elevator(hardwareMap, telemetry, () -> driverGamepad.getRightY());
        arm = new Arm(hardwareMap, telemetry, () -> driverGamepad.getRightY());
        intake = new Intake(hardwareMap, telemetry, () -> driverGamepad.getButton(GamepadKeys.Button.DPAD_UP), () -> driverGamepad.getButton(GamepadKeys.Button.DPAD_DOWN));

        colorSensor = hardwareMap.colorSensor.get("Color");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("left_stick_y", driverGamepad.getLeftY());
        telemetry.addData("left_stick_x", driverGamepad.getLeftX());
        telemetry.addData("right_stick_y", driverGamepad.getRightY());
        telemetry.addData("right_stick_x", driverGamepad.getRightX());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        drive.teleopInit();
        //elevator.teleopInit();
        arm.teleopInit();
        intake.teleopInit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.teleopPeriodic();
        //elevator.teleopPeriodic();
        //arm.teleopPeriodic();
        //intake.teleopPeriodic();

//        telemetry.addData("left_stick_y", driverGamepad.getLeftY());
//        telemetry.addData("left_stick_x", driverGamepad.getLeftX());
//        telemetry.addData("right_stick_y", driverGamepad.getRightY());
//        telemetry.addData("right_stick_x", driverGamepad.getRightX());

        int[] colors = new int[] {colorSensor.red(), colorSensor.blue(), colorSensor.green()};
        String[] colorNames = new String[] {"red", "blue", "green"};

        int maxI = 0;
        for (int i = 0; i < 3; i++) {
            if (colors[i] > colors[maxI]) {
                maxI = i;
            }
        }

        telemetry.addData("CURR COLOR", colorNames[maxI]);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
