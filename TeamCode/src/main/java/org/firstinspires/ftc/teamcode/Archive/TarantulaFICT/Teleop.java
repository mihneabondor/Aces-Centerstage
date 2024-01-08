package org.firstinspires.ftc.teamcode.Archive.TarantulaFICT;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Control Tarantula", group = "Control")

public class Teleop extends LinearOpMode {
    Hardware robot=new Hardware();

    double speed, direction, right, left;

    @Override
    public void runOpMode() throws InterruptedException {
        double sniperSpeed = 0.4;
        robot.innit(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad2.a) {
                robot.gripper.setPosition(0);
            } else {
                robot.gripper.setPosition(0.5);
            }

            if(gamepad2.right_bumper) {
                robot.pivot.setPosition(0);
            }
            if(gamepad2.left_bumper) {
                robot.pivot.setPosition(0.3);
            }

            if(gamepad2.dpad_up) {
                robot.glisiera.setPower(-0.4);
            } else if(gamepad2.dpad_down) {
                robot.glisiera.setPower(0.4);
            } else {
                robot.glisiera.setPower(0);
            }

            speed = gamepad1.right_trigger - gamepad1.left_trigger;
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
                robot.left_back.setPower(gamepad1.right_stick_x * sniperSpeed);
                robot.left_front.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.right_back.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.right_front.setPower(gamepad1.right_stick_x * sniperSpeed);
            } else {
                robot.left_back.setPower(left * sniperSpeed);
                robot.left_front.setPower(left * sniperSpeed);
                robot.right_back.setPower(right * sniperSpeed);
                robot.right_front.setPower(right * sniperSpeed);
            }
        }
    }
}
