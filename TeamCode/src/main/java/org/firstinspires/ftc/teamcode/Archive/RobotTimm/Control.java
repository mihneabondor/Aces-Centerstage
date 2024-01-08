package org.firstinspires.ftc.teamcode.Archive.RobotTimm;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
//@TeleOp(name = "Control")
public class Control extends LinearOpMode {
    Hardware robot;
    double sniperSpeed = 0.6;
    double speed, direction, right, left;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            speed = - gamepad1.right_trigger + gamepad1.left_trigger;
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
                robot.motorstangaspate.setPower(gamepad1.right_stick_x * sniperSpeed);
                robot.motorstangafata.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.motordreaptaspate.setPower(-gamepad1.right_stick_x * sniperSpeed);
                robot.motordreaptafata.setPower(gamepad1.right_stick_x * sniperSpeed);
            } else {
                robot.motorstangaspate.setPower(left * sniperSpeed);
                robot.motorstangafata.setPower(left * sniperSpeed);
                robot.motordreaptaspate.setPower(right * sniperSpeed);
                robot.motordreaptafata.setPower(right * sniperSpeed);
            }
        }

    }
}
