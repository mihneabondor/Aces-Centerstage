package org.firstinspires.ftc.teamcode.WIP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Current.Constants;
import org.firstinspires.ftc.teamcode.Current.Hardware;

@TeleOp
public class ControlOficial extends LinearOpMode {
    Hardware robot;
    boolean catarare = false;
    double pendulStartTime = 0.0;
    boolean pendulDown = true;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            gamepad1.setLedColor(255, 0, 0, 1000);

            if (gamepad2.a) {
                robot.openGripper();
            } else if(gamepad2.left_bumper) robot.openRightGripper();
            else if(gamepad2.right_bumper) robot.openLeftGripper();
            else {
                robot.closeGripper();
            }


            if(robot.gripperSt.getPosition() == Constants.GRIPPER_ST_DESCHIS || robot.gripperDr.getPosition() == Constants.GRIPPER_DR_DESCHIS) {
                gamepad1.rumble(100);
            } else {
                gamepad1.stopRumble();
            }


            if (gamepad2.dpad_up) {
                robot.ascendGlisiere();
            } else if (gamepad2.dpad_down) {
                robot.descendGlisiere();
            } else {
                robot.stopGlisiere(catarare);
            }

            if(gamepad2.left_stick_button) {
                catarare = true;
            }

            if (gamepad2.right_trigger > 0.8) robot.raisePendul();

            if (gamepad2.left_trigger > 0.8) {
                pendulStartTime = time;
                pendulDown = false;
                robot.intermediarPendul();
            }

            if (!pendulDown && time - pendulStartTime >= 0.5) {
                pendulDown = true;
                robot.lowerPendul();
            }

            if(gamepad1.right_trigger > 0.5) {
                robot.openPinballRight();
            } else{
                robot.closePinballRight();
            }

            if (gamepad1.left_trigger > 0.5) {
                robot.openPinballLeft();
            } else {
                robot.closePinballLeft();
            }

            if(robot.pinballDr.getPosition() == Constants.PINBALL_DR_DESCHIS || robot.pinballSt.getPosition() == Constants.PINBALL_ST_DESCHIS) {
                gamepad2.rumble(150);
            } else {
                gamepad2.stopRumble();
            }

            if (gamepad1.y) {
                robot.imu.resetYaw();
            }

            if(gamepad1.b) {
                robot.decolareeee();
            }

            Vector2D movement = new Vector2D(gamepad1.left_stick_x, gamepad1.left_stick_y);
            Vector2D rotated = rotate(movement, robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));


            double px = -rotated.getX();
            double py = rotated.getY();
            double pp = gamepad1.right_stick_x;

            robot.leftBack.setPower(py-px-pp);
            robot.leftFront.setPower(py+px-pp);
            robot.rightBack.setPower(-py-px-pp);
            robot.rightFront.setPower(-py+px-pp);
        }
    }

    private Vector2D rotate(Vector2D input, double angle) {
        return new Vector2D(
                input.getX() * Math.cos(angle) - input.getY() * Math.sin(angle),
                input.getX() * Math.sin(angle) + input.getY() * Math.cos(angle)
        );
    }
}
