package org.firstinspires.ftc.teamcode.Archive.RobotTimm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class Hardware {
    public DcMotorEx motordreaptaspate;
    public DcMotorEx motordreaptafata;
    public DcMotorEx motorstangaspate;
    public DcMotorEx motorstangafata;

    public Hardware(HardwareMap hardwareMap) {
        motordreaptafata = hardwareMap.get(DcMotorEx.class, "dreapta fata");
        motordreaptaspate = hardwareMap.get(DcMotorEx.class, "dreapta spate");
        motorstangafata = hardwareMap.get(DcMotorEx.class, "stanga fata");
        motorstangaspate = hardwareMap.get(DcMotorEx.class, "stanga spate");

        motordreaptafata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorstangafata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motordreaptaspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorstangaspate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorstangaspate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorstangafata.setDirection(DcMotorSimple.Direction.REVERSE);

    }

}
