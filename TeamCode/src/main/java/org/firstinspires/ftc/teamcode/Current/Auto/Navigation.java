package org.firstinspires.ftc.teamcode.Current.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.Current.Hardware;

import java.util.Objects;


public class Navigation extends LinearOpMode {
    public static final double      DriveValue = 2.43;
    public static final double      TurnValue = 2;
    public static final double      StrafeValue = 2;
    public static final double      COUNTS_PER_MOTOR_REV = 1120;
    public static final double      DRIVE_GEAR_REDUCTION = 2.0;
    public static final double      WHEEL_DIAMETER_MM = 96;
    public static final double     COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    public static final double      TURN_SPEED = 0.5;

    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot;
    Telemetry telemetry;

    public HardwareMap hMap = null;

    double reductie = 13.7; //pt motor 40:1 , 13.7:1 pt gobilda
    double coutPerRev = 384.5; //count ul encoderului :)
    double wheelDiam = 96;
    double k = (reductie * coutPerRev) / (wheelDiam * 3.14);
    // 5265.65 / 301.44
    double GlobalAngle = 0.0;
    Orientation lastAngle = new Orientation();


    public Navigation(Hardware robot) {
        this.robot = robot;
    }

    public void setHardwareMap(HardwareMap hmap) {
        this.hMap = hmap;
    }

    public void setTelemetry(OpMode opMode) {
        telemetry = new TelemetryImpl(opMode);
    }

    public void resetEncoders() {
        robot.leftMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotorBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftMotorBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void gyroTurn(double angle, double speed) {
        resetEncoders();
        if (angle < 0) {
            robot.leftMotorFront.setPower(-speed);
            robot.leftMotorBack.setPower(-speed);
            robot.rightMotorFront.setPower(speed);
            robot.rightMotorBack.setPower(speed);
        }
        if (angle > 0) {
            robot.leftMotorFront.setPower(speed);
            robot.leftMotorBack.setPower(speed);
            robot.rightMotorFront.setPower(-speed);
            robot.rightMotorBack.setPower(-speed);
        }

        robot.leftMotorBack.setPower(0);
        robot.leftMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);

        waitUntil(0.5);

    }


    public void frana() {
        robot.leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(int Target, double Speed) {

        Target = (int) (Target * k);
        if (Target < 0) Speed = Speed * (-1);

        robot.navigation.resetEncoders();
        robot.leftMotorBack.setTargetPosition((robot.leftMotorBack.getCurrentPosition() + Target) * (-1));
        robot.rightMotorBack.setTargetPosition((robot.rightMotorBack.getCurrentPosition() + Target) * (-1));
        robot.rightMotorFront.setTargetPosition((robot.rightMotorBack.getCurrentPosition() + Target)*(-1));
        robot.leftMotorFront.setTargetPosition((robot.leftMotorBack.getCurrentPosition() + Target)*(-1));

        robot.leftMotorFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION); /// run to pos
        robot.rightMotorFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftMotorBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.leftMotorFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        robot.rightMotorFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        robot.rightMotorBack.setPower(Speed);
        robot.leftMotorBack.setPower(Speed);
        robot.leftMotorFront.setPower(Speed);
        robot.rightMotorFront.setPower(Speed);


        while (robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                robot.rightMotorFront.isBusy() && robot.leftMotorFront.isBusy()) {
        }
    }


    public void waitUntil ( double waitTime)
    {
        ElapsedTime time = new ElapsedTime();

        while (time.seconds() <= waitTime) ;
    }

    public void SlideOnTime ( double time, double speed){
        resetEncoders();
        robot.leftMotorBack.setPower(speed);
        robot.leftMotorFront.setPower(-speed);
        robot.rightMotorBack.setPower(-speed);
        robot.rightMotorFront.setPower(speed);
        waitUntil(time);
        robot.leftMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.rightMotorBack.setPower(0);

    }

    public void DriveOnTime ( double time, double speed)
    {
        robot.leftMotorBack.setPower(-speed);
        robot.leftMotorFront.setPower(-speed);
        robot.rightMotorBack.setPower(-speed);
        robot.rightMotorFront.setPower(-speed);

        waitUntil(time);

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
                newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);
                newFrontRightTarget = robot.rightMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_MM * DriveValue);

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
                                robot.leftMotorBack.isBusy() && robot.rightMotorBack.isBusy() &&
                                        robot.leftMotorFront.isBusy() && robot.rightMotorFront.isBusy()
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

            newBackLeftTarget = robot.leftMotorBack.getCurrentPosition() + (int)(distance * COUNTS_PER_MM * StrafeValue);
            newBackRightTarget = robot.rightMotorBack.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
            newFrontLeftTarget = robot.leftMotorFront.getCurrentPosition() + (int)(-distance * COUNTS_PER_MM * StrafeValue);
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
        EncoderDrive(speed, speed, speed, speed, -distance,15);
        //sleep(100);
    }

    public void driveForward (double distance, double speed)
    {
        EncoderDrive(speed, speed, speed, speed, distance,15);
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

    public void StopAllMotion() {
        robot.leftMotorFront.setPower(0);
        robot.rightMotorFront.setPower(0);
        robot.leftMotorBack.setPower(0);
        robot.rightMotorBack.setPower(0);

    }

    public void runUsingEncoders(double p, int ticks, double timeout) {

        runtime.reset();

        robot.leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftMotorFront.setTargetPosition(-ticks);
        robot.leftMotorBack.setTargetPosition(-ticks);
        robot.rightMotorFront.setTargetPosition(-ticks);
        robot.rightMotorBack.setTargetPosition(-ticks);

        robot.leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.rightMotorFront.isBusy() && robot.rightMotorBack.isBusy() && robot.leftMotorFront.isBusy() && robot.leftMotorBack.isBusy()
                && runtime.seconds() < timeout){
            robot.leftMotorFront.setPower(p);
            robot.leftMotorBack.setPower(p);
            robot.rightMotorFront.setPower(p);
            robot.rightMotorBack.setPower(p);
        }

        StopAllMotion();
    }


    public int convertire(double cm){
        int ticks = 563;
        int cmperrotatie=30;
        int x = (int)(cm*ticks)/cmperrotatie;
        return x;
    }
    
    @Override
    public void runOpMode () throws InterruptedException {

    }
}