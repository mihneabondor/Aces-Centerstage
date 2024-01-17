package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Current.Hardware;
import org.firstinspires.ftc.teamcode.Current.Recognition.HuskyObjTracking;

@TeleOp
public class TestDetectieCazuri extends LinearOpMode {
    // detectie din Current Recognition

    FtcDashboard dash;
    Hardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        robot = new Hardware(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("caz", robot.recognition.getCaz(1));
            telemetry.update();
        }
    }
}
