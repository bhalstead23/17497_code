package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "ColorSensorAuto (Blocks to Java)", preselectTeleOp = "CompetitionOpMode (Blocks to Java)")
public class ColorSensorAuto extends LinearOpMode {

    private ColorSensor ColorSensor_REV_ColorRangeSensor;
    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int gain;
        NormalizedRGBA normalizedColors;
        int color;
        float hue;
        float saturation;
        float value;

        ColorSensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // This op mode demonstrates the color and distance features of the REV sensor.
        gain = 2;
        telemetry.addData("Color Distance Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Display distance info.
                telemetry.addData("Dist to tgt (cm)", ((DistanceSensor) ColorSensor_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
                // Display reflected light.
                telemetry.addData("Light detected", ((OpticalDistanceSensor) ColorSensor_REV_ColorRangeSensor).getLightDetected());
                // Adjust the gain.
                if (gamepad1.a) {
                    gain += 0.005;
                } else if (gamepad1.b && gain >= 1.005) {
                    gain += -0.005;
                }
                ((NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor).setGain(gain);
                telemetry.addData("Gain", ((NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor).getGain());
                // Read color from the sensor.
                normalizedColors = ((NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor).getNormalizedColors();
                telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
                telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3)));
                telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3)));
                // Convert RGB values to Hue, Saturation, and Value.
                // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
                color = normalizedColors.toColor();
                hue = JavaUtil.colorToHue(color);
                saturation = JavaUtil.colorToSaturation(color);
                value = JavaUtil.colorToValue(color);
                telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
                telemetry.addData("Saturation", Double.parseDouble(JavaUtil.formatNumber(saturation, 3)));
                telemetry.addData("Value", Double.parseDouble(JavaUtil.formatNumber(value, 3)));
                telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.alpha, 3)));
                // Show the color on the Robot Controller screen.
                JavaUtil.showColor(hardwareMap.appContext, color);
                // Use hue to determine if it's red, green, blue, etc..
                if (hue < 30) {
                    telemetry.addData("Color", "Red");
                    FrontLeftMotor.setTargetPosition(-1530);
                    FrontRightMotor.setTargetPosition(1530);
                    BackLeftMotor.setTargetPosition(-1530);
                    BackRightMotor.setTargetPosition(1530);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setTargetPosition(5000);
                    FrontRightMotor.setTargetPosition(5000);
                    BackLeftMotor.setTargetPosition(5000);
                    BackRightMotor.setTargetPosition(5000);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setTargetPosition(-1530);
                    FrontRightMotor.setTargetPosition(1530);
                    BackLeftMotor.setTargetPosition(-1530);
                    BackRightMotor.setTargetPosition(1530);
                } else if (hue < 60) {
                    telemetry.addData("Color", "Orange");
                } else if (hue < 90) {
                    telemetry.addData("Color", "Yellow");
                    FrontLeftMotor.setTargetPosition(5000);
                    FrontRightMotor.setTargetPosition(5000);
                    BackLeftMotor.setTargetPosition(5000);
                    BackRightMotor.setTargetPosition(5000);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (hue < 150) {
                    telemetry.addData("Color", "Green");
                } else if (hue < 225) {
                    telemetry.addData("Color", "Blue");
                    FrontLeftMotor.setTargetPosition(1530);
                    FrontRightMotor.setTargetPosition(-1530);
                    BackLeftMotor.setTargetPosition(1530);
                    BackRightMotor.setTargetPosition(-1530);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setTargetPosition(5000);
                    FrontRightMotor.setTargetPosition(5000);
                    BackLeftMotor.setTargetPosition(5000);
                    BackRightMotor.setTargetPosition(5000);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setTargetPosition(-1530);
                    FrontRightMotor.setTargetPosition(1530);
                    BackLeftMotor.setTargetPosition(-1530);
                    BackRightMotor.setTargetPosition(1530);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    FrontLeftMotor.setTargetPosition(5000);
                    FrontRightMotor.setTargetPosition(5000);
                    BackLeftMotor.setTargetPosition(5000);
                    BackRightMotor.setTargetPosition(5000);
                } else if (hue < 350) {
                    telemetry.addData("Color", "purple");
                } else {
                    telemetry.addData("Color", "Red");
                }
                // Check to see if it might be black or white.
                if (saturation < 0.2) {
                    telemetry.addData("Check Sat", "Is surface white?");
                }
                telemetry.update();
                if (value < 0.16) {
                    telemetry.addData("Check Val", "Is surface black?");
                }
            }
            // Show white on the Robot Controller screen.
            JavaUtil.showColor(hardwareMap.appContext, Color.parseColor("white"));
        }
    }
}