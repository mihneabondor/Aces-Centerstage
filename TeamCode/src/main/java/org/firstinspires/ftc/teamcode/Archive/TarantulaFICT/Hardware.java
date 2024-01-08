package org.firstinspires.ftc.teamcode.Archive.TarantulaFICT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    DcMotor left_front=null;
    DcMotor left_back=null;
    DcMotor right_front=null;
    DcMotor right_back=null;
    DcMotor glisiera=null;
    Servo gripper=null;
    Servo pivot=null;

    void innit(HardwareMap hmap) {
        left_back = hmap.get(DcMotor.class, "left_back");
        left_front = hmap.get(DcMotor.class, "left_front");
        right_back = hmap.get(DcMotor.class, "right_back");
        right_front = hmap.get(DcMotor.class, "right_front");
        gripper = hmap.get(Servo.class, "gripper");
        pivot = hmap.get(Servo.class, "pivot");
        glisiera = hmap.get(DcMotor.class, "glisiera");

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot.setPosition(0);
        gripper.setPosition(0.5);
    }
}
