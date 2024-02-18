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
public class AutonomaBlueStanga extends LinearOpMode {
    SampleMecanumDrive robot;
    public static Caz caz = Caz.RIGHT;
    public static double x_stanga = 31.4;
    public static double y_stanga = 27;
    public static double x_dreapta = 2;
    public static double y_dreapta = 27;
    public static double forward_center = 36;
    public static double slide_st = 15;
    public static double slide_dr = -50;
    public static double back_dreapta = 40;
    public static double rad_dreapta = 170;
    public static double back_last_right = 10;
    public static double x_center = 55;
    public static double y_center = 33;
    public static double rad_center = 185;
    public static double slide_centru = 42;
    public static double back_last_center = 15;
    public static double rad_stanga = 185;
    public static double back_stanga = 27;
    public static double slide_stanga = 40;
    public static double back_last_left = 10;
    public static double slide_stanga_mic = 26;
    public static double slide_stanga_mic2 = 18;
    public static double timp_glisiere = 0.2;
    public static double back_mic_centru = 8;
    public static double timp_glisiere_centru = 0.1;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap, Orientation.LEFT);
        dashboard = FtcDashboard.getInstance();
        robot.setConfig(hardwareMap, Orientation.LEFT);
        robot.setDefaults();
        robot.lowerPendul();
        robot.closeGripper();
        while(opModeInInit()) caz = robot.recognition.getCaz(1);

        waitForStart();

        // caz stanga
        TrajectorySequence traj_stanga = robot.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(270))) // start!
                .lineToLinearHeading(new Pose2d(x_stanga, y_stanga, Math.toRadians(rad_stanga)))
                .addTemporalMarker(()-> {
                    robot.openLeftGripper();
                })
                .waitSeconds(1)
                .back(back_stanga)
                .strafeRight(slide_stanga_mic)
                .addTemporalMarker(()->{
                    robot.raisePendul();

                })
                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    robot.ascendGlisiere();
                })
                .waitSeconds(timp_glisiere)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.openRightGripper();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.descendGlisiere();
                })
                .waitSeconds(timp_glisiere)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lowerPendul();
                })
                .waitSeconds(1)
//                .strafeRight(slide_stanga)
                .build();

        //caz dreapta
        TrajectorySequence traj_dreapta = robot.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(270))) // start!!

                .forward(15)
                .lineToLinearHeading(new Pose2d(x_dreapta, y_dreapta, Math.toRadians(rad_dreapta)))
                .addTemporalMarker(()->{
                    robot.openLeftGripper();
                })
                .waitSeconds(0.5)
                .back(back_dreapta)
                .strafeLeft(slide_stanga_mic2)
                .addTemporalMarker(() -> {
                    robot.raisePendul();
                 })
                .waitSeconds(0.3)
                .addTemporalMarker(()-> {
                    robot.ascendGlisiere();
                })
                .waitSeconds(timp_glisiere)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.openRightGripper();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.descendGlisiere();
                })
                .waitSeconds(timp_glisiere)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lowerPendul();
                })
                .strafeRight(slide_dr)
                .back(back_last_right)
                .build();

        // caz centru
        TrajectorySequence traj_center = robot.trajectorySequenceBuilder(new Pose2d(10, 62, Math.toRadians(270))) // start!!
                .forward(forward_center)
                .addTemporalMarker(() -> {
                    robot.openLeftGripper();
                })
                .waitSeconds(0.5)
                .back(back_mic_centru)
                .lineToLinearHeading(new Pose2d(x_center, y_center, Math.toRadians(rad_center)))
                .addTemporalMarker(() -> {
                    robot.raisePendul();
                })
                .waitSeconds(1)
                .addTemporalMarker(()-> {
                    robot.ascendGlisiere();
                })
                .waitSeconds(timp_glisiere_centru)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    robot.openRightGripper();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.descendGlisiere();
                })
                .waitSeconds(timp_glisiere_centru)
                .addTemporalMarker(() -> {
                    robot.franaGlisiere();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    robot.lowerPendul();
                })
                .strafeRight(slide_centru)
                .back(back_last_center)
                .build();


        if(caz == Caz.LEFT) robot.followTrajectorySequence(traj_stanga);
        else if (caz == Caz.MID) robot.followTrajectorySequence(traj_center);
        else {
            robot.followTrajectorySequence(traj_dreapta);
//            robot.raisePendul();

        }


    }
}
