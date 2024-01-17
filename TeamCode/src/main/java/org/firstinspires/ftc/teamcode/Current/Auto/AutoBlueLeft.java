package org.firstinspires.ftc.teamcode.Current.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Current.Hardware;
import org.firstinspires.ftc.teamcode.Current.Recognition.Caz;
import org.firstinspires.ftc.teamcode.Current.Recognition.HuskyObjTracking;

@Autonomous
public class AutoBlueLeft extends LinearOpMode {
    Hardware robot;
    ElapsedTime runtime = new ElapsedTime();
    public static final double      DRIVE_VALUE = 1.8;
    public static final double      TurnValue = 2;
    public static final double      StrafeValue = 2;
    public static final double      COUNTS_PER_MOTOR_REV = 1120;
    public static final double      DRIVE_GEAR_REDUCTION = 2.5;
    public static final double      WHEEL_DIAMETER_MM = 96;
    public static final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double      TURN_SPEED = 0.5;
    Caz caz;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        robot.resetEncoders();
        robot.runWithEncoders();

        while(opModeInInit()) {
            caz = robot.recognition.getCaz(1);
        }

        waitForStart();
        switch(caz) {
            case LEFT: cazSt();
            break;
            case MID: cazMid();
            break;
            case RIGHT: cazDr();
            break;
        }
    }

    private void cazSt() {
        driveForward(30, 0.2);
        stopAllMotion();
        sleep(100);
        strafeLeft(25, 0.2);
        stopAllMotion();
        sleep(500);
        robot.pivot.setPosition(robot.pivotJos);
        sleep(1000);
        robot.gripperDr.setPosition(robot.gripperDrDeschis);
        robot.gripperSt.setPosition(robot.gripperStDeschis);
        sleep(1000);
        driveBackward(20, 0.2);
        stopAllMotion();
        sleep(1000);
        robot.pivot.setPosition(robot.pivotScorat);
        sleep(1000);
    }

    private void cazDr() {
        driveForward(30, 0.2);
        stopAllMotion();
        sleep(100);
        strafeRight(23, 0.2);
        stopAllMotion();
        sleep(500);
        robot.pivot.setPosition(robot.pivotJos);
        sleep(1000);
        robot.gripperDr.setPosition(robot.gripperDrDeschis);
        robot.gripperSt.setPosition(robot.gripperStDeschis);
        sleep(1000);
        driveBackward(20, 0.2);
        stopAllMotion();
        sleep(1000);
        robot.pivot.setPosition(robot.pivotScorat);
        sleep(1000);
    }

    private void cazMid() {
        driveForward(51, 0.2);
        stopAllMotion();
        sleep(100);
        stopAllMotion();
        sleep(500);
        stopAllMotion();
        sleep(500);
        robot.pivot.setPosition(robot.pivotJos);
        sleep(1000);
        robot.gripperDr.setPosition(robot.gripperDrDeschis);
        robot.gripperSt.setPosition(robot.gripperStDeschis);
        sleep(1000);
        robot.pivot.setPosition(robot.pivotScorat);
        sleep(1000);
        driveBackward(20, 0.2);
        stopAllMotion();
        sleep(1000);
    }

    public void stopAllMotion() {
        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
    }

    public void EncoderDrive (double speed_RF,double speed_RB,double speed_LF,double speed_LB, double distance, double timeoutS)
    {
        if (opModeIsActive()) {
            int newBackLeftTarget;
            int newBackRightTarget;
            int newFrontLeftTarget;
            int newFrontRightTarget;
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DRIVE_VALUE);
                newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DRIVE_VALUE);
                newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DRIVE_VALUE);
                newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DRIVE_VALUE);

                robot.rightMotorBack.setTargetPositionTolerance(50);
                robot.rightMotorFront.setTargetPositionTolerance(50);
                robot.leftMotorBack.setTargetPositionTolerance(50);
                robot.leftMotorFront.setTargetPositionTolerance(50);


                robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
                robot.rightMotorBack.setTargetPosition(newBackRightTarget);
                robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
                robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

                // Turn On RUN_TO_POSITION
                robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftMotorBack.setPower(speed_LB);
                robot.rightMotorBack.setPower(speed_RB);
                robot.leftMotorFront.setPower(speed_LF);
                robot.rightMotorFront.setPower(speed_RF);

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (
                                robot.rightMotorBack.isBusy() ||
                                        robot.leftMotorFront.isBusy() || robot.rightMotorFront.isBusy()
                        )) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d"
                            , newFrontLeftTarget, newFrontRightTarget
                            , newBackLeftTarget, newBackRightTarget
                    );
                    telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                            robot.leftMotorFront.getCurrentPosition(),
                            robot.rightMotorFront.getCurrentPosition()
                            ,
                            robot.leftMotorBack.getCurrentPosition(),
                            robot.rightMotorBack.getCurrentPosition()
                    );
                    telemetry.addData("RF", String.valueOf(robot.rightMotorFront.getPower()));
                    telemetry.addData("RB", String.valueOf(robot.rightMotorBack.getPower()));
                    telemetry.addData("LF", String.valueOf(robot.leftMotorFront.getPower()));
                    telemetry.addData("LB", String.valueOf(robot.leftMotorBack.getPower()));
                    telemetry.update();
                }

                // Stop all motion;


                //robot.leftMotorBack.setPower(0);
                //robot.rightMotorBack.setPower(0);
                //robot.leftMotorFront.setPower(0);
                //robot.rightMotorFront.setPower(0);


                /* COMMENT THESE FOR SPEED */

                // Turn off RUN_TO_POSITION
                robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //sleep(250);   // optional pause after each move
            }
        }
    }
    public void EncoderStrafe(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);
            newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);

            robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
            robot.rightMotorBack.setTargetPosition(newBackRightTarget);
            robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
            robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotorBack.setPower(-speed);
            robot.rightMotorBack.setPower(-speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                    robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
                    ))
            {

                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition()
                        ,
                        robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());

                telemetry.update();
            }

            robot.leftMotorBack.setPower(0);
            robot.rightMotorBack.setPower(0);
            robot.leftMotorFront.setPower(0);
            robot.rightMotorFront.setPower(0);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void driveBackward (double distance, double speed)
    {
        EncoderDrive(speed, speed, speed, speed, distance,15);
        //sleep(100);
    }

    public void driveForward (double distance, double speed)
    {
        EncoderDrive(speed, speed, speed, speed, -distance,15);
        sleep(100);
    }

    public void strafeRight(double distance, double speed)
    {
        EncoderStrafe(-speed, -distance, 15);
        sleep(100);
    }

    public void strafeLeft (double distance, double speed)
    {
        EncoderStrafe(speed, distance, 15);
        sleep(100);
    }

    public void EncoderTurn(double speed, double distance, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        if (opModeIsActive()) {

            newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);
            newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * TurnValue);
            newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * TurnValue);

            robot.leftMotorBack.setTargetPosition(newBackLeftTarget);
            robot.rightMotorBack.setTargetPosition(newBackRightTarget);
            robot.leftMotorFront.setTargetPosition(newFrontLeftTarget);
            robot.rightMotorFront.setTargetPosition(newFrontRightTarget);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorBack.setPower(speed);
            robot.leftMotorFront.setPower(speed);
            robot.rightMotorFront.setPower(speed);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (
                            robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                    robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
                    ))
            {
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d"
                        , newFrontLeftTarget, newFrontRightTarget
                        , newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftMotorFront.getCurrentPosition(),
                        robot.rightMotorFront.getCurrentPosition()
                        ,
                        robot.leftMotorBack.getCurrentPosition(),
                        robot.rightMotorBack.getCurrentPosition());
                telemetry.update();
            }

            //robot.leftMotorBack.setPower(0);
            //robot.rightMotorBack.setPower(0);
            //robot.leftMotorFront.setPower(0);
            //robot.rightMotorFront.setPower(0);

            robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void rotateRight(double angle)
    {
        EncoderTurn(TURN_SPEED, angle, 15);
    }

    public void rotateLeft(double angle)
    {
        EncoderTurn(-TURN_SPEED, -angle, 15);
    }
}
