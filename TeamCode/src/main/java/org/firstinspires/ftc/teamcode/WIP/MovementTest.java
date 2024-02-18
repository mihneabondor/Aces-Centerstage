package org.firstinspires.ftc.teamcode.WIP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class MovementTest extends LinearOpMode {


    public static double sniperSpeed=0.8;
    public static double speed, direction, right, left;
    public DcMotorEx leftMotorBack, rightMotorBack, rightMotorFront, leftMotorFront, glisieraDr, glisieraSt;
    boolean catarare = false;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException{
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "stanga_fata");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "dreapta_spate");
        rightMotorFront = hardwareMap.get(DcMotorEx.class, "dreapta_fata");
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "stanga_spate");

        // franaaaaaa
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // reverse pentru turn
        leftMotorFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorEx.Direction.REVERSE);


        dashboard = FtcDashboard.getInstance();


        waitForStart();
        while(opModeIsActive()) {




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
                leftMotorBack.setPower(gamepad1.right_stick_x * sniperSpeed);
                leftMotorFront.setPower(-gamepad1.right_stick_x * sniperSpeed);
                rightMotorFront.setPower(-gamepad1.right_stick_x * sniperSpeed);
                rightMotorBack.setPower(gamepad1.right_stick_x * sniperSpeed);
            } else {
                leftMotorFront.setPower(left * sniperSpeed);
                leftMotorBack.setPower(left * sniperSpeed);
                rightMotorFront.setPower(right * sniperSpeed);
                rightMotorBack.setPower(right * sniperSpeed);
            }
        }
    }


}