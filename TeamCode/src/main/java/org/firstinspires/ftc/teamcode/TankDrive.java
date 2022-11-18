package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDrive (Blocks to Java)")
public class TankDrive extends LinearOpMode {

  private DcMotor FrontRightMotor;
  private DcMotor FrontLeftMotor;
  private DcMotor BackLeftMotor;
  private DcMotor BackRightMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
    FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
    BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
    BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
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
      }
    }
  }
}
