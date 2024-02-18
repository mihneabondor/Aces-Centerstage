package org.firstinspires.ftc.teamcode.Current;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Current.Recognition.Caz;
import org.firstinspires.ftc.teamcode.Current.Recognition.Orientation;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class AutonomaRedStanga extends LinearOpMode {
    SampleMecanumDrive robot;
    public static Caz caz = Caz.RIGHT;
    public static double x_stanga = -24;
    public static double y_stanga = -29;
    public static double x_dreapta = -30;
    public static double y_dreapta = -22;
    public static double forward_center = 40;
    public static double slide_dr = 15;
    public static double back_dreapta = 15;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap, Orientation.LEFT);
        dashboard = FtcDashboard.getInstance();
        robot.closeGripper();
        while(opModeInInit()) caz = robot.recognition.getCaz(1);
        waitForStart();
        telemetry.addLine(caz.name());
        // caz stanga
        TrajectorySequence traj_stanga = robot.trajectorySequenceBuilder(new Pose2d(-35.3, -67, Math.toRadians(90))) // start!!
                .forward(15)
                .lineToLinearHeading(new Pose2d(x_stanga, y_stanga, Math.toRadians(5)))
                .addDisplacementMarker(() -> {
                    robot.openLeftGripper();
                })
                .waitSeconds(2)
                .back(15)
                .build();

        //caz dreapta
        TrajectorySequence traj_dreapta = robot.trajectorySequenceBuilder(new Pose2d(-35.3, -67, Math.toRadians(90))) // start!!
                .strafeRight(slide_dr)
                .lineToLinearHeading(new Pose2d(x_dreapta, y_dreapta, Math.toRadians(180)))
                .addDisplacementMarker(() -> {
                    robot.openLeftGripper();
                })
                .waitSeconds(2)
                .back(back_dreapta)

                .build();

        // caz centru
        TrajectorySequence traj_center = robot.trajectorySequenceBuilder(new Pose2d(-35.3, -67, Math.toRadians(90))) // start!!
                .forward(forward_center)
                .addDisplacementMarker(() -> {
                    robot.openLeftGripper();
                })
                .waitSeconds(2)
                .back(15)
                .build();

        if(caz == Caz.RIGHT) robot.followTrajectorySequence(traj_stanga);
        else if (caz == Caz.MID) robot.followTrajectorySequence(traj_center);
        else robot.followTrajectorySequence(traj_dreapta);
    }
}
