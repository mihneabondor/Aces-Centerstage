package org.firstinspires.ftc.teamcode.Archive.RobotAvion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {
    public DcMotorEx motordrfata;
    public DcMotorEx motordrspate;
    public DcMotorEx motorstfata;
    public DcMotorEx motorstspate;

    public Hardware(HardwareMap hardwareMap) {
        motordrfata = hardwareMap.get(DcMotorEx.class, "dreapta fata");
        motordrspate = hardwareMap.get(DcMotorEx.class, "dreapta spate");
        motorstfata = hardwareMap.get(DcMotorEx.class, "stanga fata");
        motorstspate = hardwareMap.get(DcMotorEx.class, "stanga spate");

        motordrfata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordrspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorstfata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorstspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorstspate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorstfata.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
