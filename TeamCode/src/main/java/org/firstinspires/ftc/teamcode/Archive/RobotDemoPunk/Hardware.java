package org.firstinspires.ftc.teamcode.Archive.RobotDemoPunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public DcMotorEx stfata;
    public DcMotorEx stspate;
    public DcMotorEx drfata;
    public DcMotorEx drspate;
    public DcMotorEx glisiera;
    public Servo pivot;
    public Servo avion;
    public Servo gripper;

    public Hardware(HardwareMap hardwareMap) {
        stfata = hardwareMap.get(DcMotorEx.class, "stanga_fata");
        stspate = hardwareMap.get(DcMotorEx.class, "stanga_spate");
        drfata = hardwareMap.get(DcMotorEx.class, "dreapta_fata");
        drspate = hardwareMap.get(DcMotorEx.class, "dreapta_spate");
        glisiera = hardwareMap.get(DcMotorEx.class, "glisiera");
        avion = hardwareMap.get(Servo.class, "avion");
        gripper = hardwareMap.get(Servo.class, "gripper");
        pivot = hardwareMap.get(Servo.class, "pivot");

        stfata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drfata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drfata.setDirection(DcMotorSimple.Direction.REVERSE);
        drspate.setDirection(DcMotorSimple.Direction.REVERSE);

        avion.setPosition(0);
        pivot.setPosition(0);
        gripper.setPosition(0.5);
    }
}
