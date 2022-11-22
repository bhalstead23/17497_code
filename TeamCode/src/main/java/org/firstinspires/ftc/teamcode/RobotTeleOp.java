package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Full TeleOp", group = "Iterative Opmode")
public class RobotTeleOp extends OpMode {
    private Drive drive;
    private Elevator elevator;
    private Intake intake;
    private Arm arm;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new Drive(hardwareMap, gamepad1);
        elevator = new Elevator(hardwareMap, gamepad1);
        intake = new Intake(hardwareMap, gamepad1);
        arm = new Arm(hardwareMap, gamepad1);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        drive.teleopInit();
        elevator.teleopInit();
        arm.teleopInit();
        intake.teleopInit();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive.teleopPeriodic();
        elevator.teleopPeriodic();
        arm.teleopPeriodic();
        intake.teleopPeriodic();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
