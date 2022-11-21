package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Drive {
    private DcMotor FrontRightMotor;
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    private HardwareMap hardwareMap;
    private Gamepad gamepad;
    private Telemetry telemetry;

    public Drive(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad1;
        this.telemetry = telemetry;
    }
    public void teleopInit() {
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        telemetry.addData("right", "not switched");
        telemetry.addData("left", "not switched");
    }
    public void setSpeeds(float left, float right) {
        FrontLeftMotor.setPower(left);
        BackLeftMotor.setPower(left);
        FrontRightMotor.setPower(-right);
        BackRightMotor.setPower(-right);
    }
    public void teleopPeriodic() {
        // Put loop blocks here.
        // Front and Backward
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        float leftSpeed = gamepad.left_stick_y + -gamepad.left_stick_x;
        float rightSpeed = gamepad.left_stick_y + gamepad.left_stick_x;
        setSpeeds(leftSpeed, rightSpeed);
        /*FrontLeftMotor.setPower(gamepad.left_stick_y + -gamepad.left_stick_x);
        FrontRightMotor.setPower(gamepad.left_stick_y + gamepad.left_stick_x);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        BackLeftMotor.setPower(gamepad.left_stick_y + -gamepad.left_stick_x);
        BackRightMotor.setPower(-gamepad.left_stick_y + -gamepad.left_stick_x);*/
/*        telemetry.addData("Left Pow", FrontLeftMotor.getPower());
        telemetry.addData("Right Pow", FrontRightMotor.getPower());
        telemetry.addData("LeftPow2", BackLeftMotor.getPower());
        telemetry.addData("RightPow2", BackRightMotor.getPower());
        telemetry.update();*/

    }
}
