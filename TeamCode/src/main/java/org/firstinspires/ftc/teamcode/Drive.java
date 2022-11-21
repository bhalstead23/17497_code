package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Drive {
    private DcMotor FrontRightMotor;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    public void teleopInit(Robot robot) {
        FrontRightMotor = robot.hardwareMap.get(DcMotor.class, "FrontRightMotor");
        FrontLeftMotor = robot.hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = robot.hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = robot.hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void teleopPeriodic() {
/*        // Put loop blocks here.
        // Front and Backward
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        FrontLeftMotor.setPower(gamepad1.left_stick_y + -gamepad1.left_stick_x);
        FrontRightMotor.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        BackLeftMotor.setPower(gamepad1.left_stick_y + -gamepad1.left_stick_x);
        BackRightMotor.setPower(-gamepad1.left_stick_y + -gamepad1.left_stick_x);
        telemetry.addData("Left Pow", FrontLeftMotor.getPower());
        telemetry.addData("Right Pow", FrontRightMotor.getPower());
        telemetry.addData("LeftPow2", BackLeftMotor.getPower());
        telemetry.addData("RightPow2", BackRightMotor.getPower());
        telemetry.update();
    */
    }
}
