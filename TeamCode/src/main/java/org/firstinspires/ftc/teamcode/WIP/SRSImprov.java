package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
@Config
public class SRSImprov extends LinearOpMode {
    Servo servo_stanga, servo_dreapta, servo3;
    FtcDashboard dashboard;


    public static double sus1jos0 = 0;
    public static double val1 = 0.05;
    public static double val2 = 0.95; /// pozitii de jos la pivot!!!
    public static double val3 = 1;

    @Override
    public void runOpMode() throws InterruptedException {
//        servo_stanga = hardwareMap.get(Servo.class, "servo1"); // port 1
//        servo_dreapta = hardwareMap.get(Servo.class, "servo2"); // port 2
        servo3 = hardwareMap.get(Servo.class, "servo3");
        dashboard = FtcDashboard.getInstance();
        waitForStart();


        double pivotJos = 1, pivotSus = 0.15;

        while(opModeIsActive()) {

            if (sus1jos0 == 0) {
//                servo_stanga.setPosition(val1);
//                servo_dreapta.setPosition(val2);
                servo3.setPosition(pivotJos);
            }
            else {
//                servo_stanga.setPosition(val2);
//                servo_dreapta.setPosition(val1);
                servo3.setPosition(pivotSus);
            }
        }
    }
}
