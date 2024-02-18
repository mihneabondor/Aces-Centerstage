package org.firstinspires.ftc.teamcode.WIP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sun.source.tree.WhileLoopTree;

import org.firstinspires.ftc.teamcode.Current.Hardware;

@TeleOp
public class TestCatare extends LinearOpMode {
    DcMotorEx glisieraDr, glisieraSt;
    double power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        glisieraDr = hardwareMap.get(DcMotorEx.class, "glisiera_dreapta");
        glisieraSt = hardwareMap.get(DcMotorEx.class, "glisiera_stanga");
        glisieraSt.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()) {
            glisieraSt.setPower(power);
            glisieraDr.setPower(power);
            if(gamepad1.dpad_up) {
                power = 0.6;
            } else if(gamepad1.dpad_down) {
                power = -0.6;
            } else {
                power = 0;
            }
        }
    }
}
