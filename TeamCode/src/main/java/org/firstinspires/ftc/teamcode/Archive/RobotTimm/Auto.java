package org.firstinspires.ftc.teamcode.Archive.RobotTimm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name = "auto")
public class Auto extends LinearOpMode {
    Hardware robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);

        waitForStart();
        robot.motorstangafata.setPower(-0.3);
        robot.motorstangaspate.setPower(-0.3);
        robot.motordreaptaspate.setPower(-0.3);
        robot.motordreaptafata.setPower(-0.3);

        sleep(2500);
        robot.motorstangafata.setPower(0);
        robot.motorstangaspate.setPower(0);
        robot.motordreaptaspate.setPower(0);
        robot.motordreaptafata.setPower(0);
    }
}
