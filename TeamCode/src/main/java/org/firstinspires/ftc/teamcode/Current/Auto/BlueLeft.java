package org.firstinspires.ftc.teamcode.Current.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous
public class BlueLeft extends LinearOpMode {
    SampleMecanumDrive drive;
    FtcDashboard dash;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        dash = FtcDashboard.getInstance();
        waitForStart();

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(11, 61.4, Math.toRadians(270)))
                                .lineTo(new Vector2d(11, 35))
                                .lineToLinearHeading(new Pose2d(48, 35, Math.toRadians(180)))
                                .build();
        drive.followTrajectorySequence(traj);
    }
}
