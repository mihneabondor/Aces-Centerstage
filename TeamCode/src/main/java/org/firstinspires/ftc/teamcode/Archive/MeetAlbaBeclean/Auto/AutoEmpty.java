package org.firstinspires.ftc.teamcode.Archive.MeetAlbaBeclean.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class AutoEmpty extends LinearOpMode{
    Hardware robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Hardware(hardwareMap);
        waitForStart();

    }
}





