package org.firstinspires.ftc.teamcode.Archive.Robot2EventCentral;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import teamcode.TarantulaFICT.Hardware;

@Disabled
@TeleOp(name = "Control robot 2")
public class ControlRobot2 extends LinearOpMode {
    DcMotorEx motorStanga = null, motorDreapta = null;

    @Override
    public void runOpMode() throws InterruptedException {
        motorDreapta = hardwareMap.get(DcMotorEx.class, "motor dreapta");
        motorStanga = hardwareMap.get(DcMotorEx.class, "motor stanga");

        waitForStart();
        // daca nu sunt bune directiile vezi -/+ gamepad1.left_stick_x in loc de +/-
        motorStanga.setPower(gamepad1.left_trigger - gamepad1.right_trigger + gamepad1.left_stick_x);
        motorDreapta.setPower(gamepad1.left_trigger - gamepad1.right_trigger - gamepad1.left_stick_x);

    }
}
