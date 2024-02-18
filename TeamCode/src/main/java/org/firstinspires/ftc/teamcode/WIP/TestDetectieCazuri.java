package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Archive.MeetAlbaBeclean.Auto.Hardware;
import org.firstinspires.ftc.teamcode.Current.Recognition.Orientation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@TeleOp
public class TestDetectieCazuri extends LinearOpMode {
    // detectie din Current Recognition

    FtcDashboard dash;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());
        robot = new SampleMecanumDrive(hardwareMap, Orientation.LEFT);

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("caz", robot.recognition.getCaz(1));
            telemetry.update();
        }
    }
}
