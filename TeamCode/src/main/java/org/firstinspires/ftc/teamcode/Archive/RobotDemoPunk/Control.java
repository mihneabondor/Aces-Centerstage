package org.firstinspires.ftc.teamcode.Archive.RobotDemoPunk;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "-control")
public class Control extends LinearOpMode {

    Hardware robot;
    double sniperSpeed=0.6;
    double speed, direction, right, left;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Hardware(hardwareMap);
        robot.pivot.setPosition(0.5);
        robot.avion.setPosition(0);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad2.a) {
                robot.gripper.setPosition(0);
            } else {
                robot.gripper.setPosition(0.5);
                gamepad1.stopRumble();
            }

            if(gamepad2.right_bumper){
                robot.pivot.setPosition(0.2);
            }
            if(gamepad2.left_bumper) {
                robot.pivot.setPosition(0.5);
            }

            if(gamepad2.y){
                robot.avion.setPosition(0.3);
            }
            if(gamepad2.dpad_up) {
                robot.glisiera.setPower(0.4);
            } else if(gamepad2.dpad_down) {
                robot.glisiera.setPower(-0.4);
            } else {
                robot.glisiera.setPower(0);
            }
            speed = -gamepad1.right_trigger + gamepad1.left_trigger;
            direction = gamepad1.left_stick_x;

            right = speed + direction;
            left = speed - direction;

            if (right > 1)
                right = 1;
            if (right < -1)
                right = -1;
            if (left > 1)
                left = 1;
            if (left < -1)
                left = -1;

            if (gamepad1.right_stick_x != 0) {
                robot.stspate.setPower(gamepad1.right_stick_x * sniperSpeed);
                robot.stfata.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.drspate.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.drfata.setPower(gamepad1.right_stick_x * sniperSpeed);
            } else {
                robot.stspate.setPower(left * sniperSpeed);
                robot.stfata.setPower(left * sniperSpeed);
                robot.drspate.setPower(right * sniperSpeed);
                robot.drfata.setPower(right * sniperSpeed);
            }
        }
    }


}
