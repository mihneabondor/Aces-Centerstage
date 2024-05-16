package org.firstinspires.ftc.teamcode.UTCN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp

public class TeleOpUTCN extends LinearOpMode {
    Hardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new org.firstinspires.ftc.teamcode.UTCN.Hardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                robot.brat.setPosition(robot.bratnormal);
            } else
            {
                robot.brat.setPosition(robot.bratflexat);
            }

        }

    }
}
