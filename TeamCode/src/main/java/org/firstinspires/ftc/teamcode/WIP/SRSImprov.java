package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class SRSImprov extends LinearOpMode {
    Servo servo1, servo2;
    FtcDashboard dashboard;

    public static double val1 = 0;
    public static double val2 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while(opModeIsActive()) {
            servo1.setPosition(val1);
            servo2.setPosition(val2);
        }
    }
}
