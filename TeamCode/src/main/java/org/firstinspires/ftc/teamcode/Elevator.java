package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private DcMotor LeftElevator;
    private DcMotor RightElevator;

    private HardwareMap hardwareMap;

    public Elevator(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

    }
}
