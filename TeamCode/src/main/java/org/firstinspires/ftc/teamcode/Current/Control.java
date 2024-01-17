package org.firstinspires.ftc.teamcode.Current;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class Control extends LinearOpMode {

    Hardware robot;
    double sniperSpeed=0.8;
    double speed, direction, right, left;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Hardware(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad2.a) {
                robot.gripperSt.setPosition(robot.gripperStDeschis);
                robot.gripperDr.setPosition(robot.gripperDrDeschis);
                gamepad1.rumble(100);
            } else {
                robot.gripperSt.setPosition(robot.gripperStInchis);
                robot.gripperDr.setPosition(robot.gripperDrInchis);
                gamepad1.stopRumble();
            }

            if(gamepad2.dpad_up) {
                robot.glisieraSt.setPower(0.4);
                robot.glisieraDr.setPower(0.4);
            } else if(gamepad2.dpad_down) {
                robot.glisieraDr.setPower(-0.4);
                robot.glisieraSt.setPower(-0.4);
            } else {
                robot.glisieraDr.setPower(0.05);
                robot.glisieraSt.setPower(0.05);
            }



            if(gamepad2.left_bumper) {
                robot.pivot.setPosition(robot.pivotScorat);
            }

            if(gamepad2.right_bumper) {
                robot.pivot.setPosition(robot.pivotJos);
            }

            if(gamepad1.y) {
                robot.avion.setPosition(robot.avionLansat);
            }

            speed = -gamepad1.right_trigger + gamepad1.left_trigger;
            direction = gamepad1.left_stick_x;

            right = speed - direction;
            left = speed + direction;

            if (right > 1)
                right = 1;
            if (right < -1)
                right = -1;
            if (left > 1)
                left = 1;
            if (left < -1)
                left = -1;

            if (gamepad1.right_stick_x != 0) {
                robot.leftMotorBack.setPower(gamepad1.right_stick_x * sniperSpeed);
                robot.leftMotorFront.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.rightMotorFront.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.rightMotorBack.setPower(gamepad1.right_stick_x * sniperSpeed);
            } else {
                robot.leftMotorFront.setPower(left * sniperSpeed);
                robot.leftMotorBack.setPower(left * sniperSpeed);
                robot.rightMotorFront.setPower(right * sniperSpeed);
                robot.rightMotorBack.setPower(right * sniperSpeed);
            }
        }
    }


}