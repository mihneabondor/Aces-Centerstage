package org.firstinspires.ftc.teamcode.Archive.RobotDemoPunk;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "auto deploy pixel")
public class AutoDeployPixel extends LinearOpMode{
    Hardware robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Hardware(hardwareMap);
        robot.pivot.setPosition(0.6);
        robot.glisiera.setPower(0);
        robot.gripper.setPosition(0.5);

        waitForStart();


        robot.stfata.setPower(-0.3);
        robot.stspate.setPower(-0.3);
        robot.drfata.setPower(-0.3);
        robot.drspate.setPower(-0.3);

//        robot.gripper.setPosition(0);
//        robot.avion.setPosition(0);



        sleep(2500);

        robot.stfata.setPower(0);
        robot.stspate.setPower(0);
        robot.drfata.setPower(0);
        robot.drspate.setPower(0);
        sleep(1000);

       // robot.gripper.setPosition(0);

        robot.stfata.setPower(0.3);
        robot.stspate.setPower(0.3);
        robot.drfata.setPower(0.3);
        robot.drspate.setPower(0.3);

        //robot.gripper.setPosition(0);

        sleep(1015);

        robot.stfata.setPower(0);
        robot.stspate.setPower(0);
        robot.drfata.setPower(0);
        robot.drspate.setPower(0);

        sleep(1000);
        robot.pivot.setPosition(0);
        sleep(1000);
        robot.gripper.setPosition(0);

        sleep(1000);
        robot.pivot.setPosition(0.6);
        sleep(1500);


    }
}





