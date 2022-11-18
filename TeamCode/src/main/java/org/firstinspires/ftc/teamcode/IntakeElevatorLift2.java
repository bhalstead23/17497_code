package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IntakeElevatorLift2 (Blocks to Java)")
public class IntakeElevatorLift2 extends LinearOpMode {

  private DcMotor RightArm;
  private DcMotor RightElevator;
  private DcMotor LeftArm;
  private DcMotor LeftElevator;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    RightArm = hardwareMap.get(DcMotor.class, "Right Arm");
    RightElevator = hardwareMap.get(DcMotor.class, "RightElevator");
    LeftArm = hardwareMap.get(DcMotor.class, "LeftArm");
    LeftElevator = hardwareMap.get(DcMotor.class, "LeftElevator");

    // Put initialization blocks here.
    if (opModeIsActive()) {
      // Put run blocks here.
      RightArm.setDirection(DcMotorSimple.Direction.REVERSE);
      RightElevator.setDirection(DcMotorSimple.Direction.REVERSE);
      LeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      RightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      LeftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      RightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      LeftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      RightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      LeftArm.setPower(1);
      RightArm.setPower(1);
      LeftElevator.setPower(1);
      RightElevator.setPower(1);
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad1.a) {
          LeftArm.setTargetPosition(500);
          RightArm.setTargetPosition(500);
          LeftElevator.setTargetPosition(100);
          RightElevator.setTargetPosition(100);
          LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad1.b) {
          LeftArm.setTargetPosition(0);
          RightArm.setTargetPosition(0);
          LeftElevator.setTargetPosition(0);
          RightElevator.setTargetPosition(0);
          LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          LeftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          RightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.update();
      }
    }
    waitForStart();
  }
}
