package org.firstinspires.ftc.teamcode.UTCN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hardware {
    public Servo brat;

    public static double bratnormal = 0;
    public static double bratflexat = 0.3;


    public Hardware(HardwareMap hardwareMap) throws InterruptedException{
        brat = hardwareMap.get(Servo.class, "brat");

    }

}
