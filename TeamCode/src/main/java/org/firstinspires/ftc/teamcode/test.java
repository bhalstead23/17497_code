package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "test (Blocks to Java)")
public class test extends LinearOpMode {

  private DcMotor Right;
  private DcMotor LeftMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    Right = hardwareMap.get(DcMotor.class, "Right");
    LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    Right.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        LeftMotor.setPower(-gamepad1.left_stick_y);
        Right.setPower(-gamepad1.right_stick_y);
        telemetry.addData("Left Pow", LeftMotor.getPower());
        telemetry.addData("Right Pow", Right.getPower());
        telemetry.update();
      }
    }
  }
}
