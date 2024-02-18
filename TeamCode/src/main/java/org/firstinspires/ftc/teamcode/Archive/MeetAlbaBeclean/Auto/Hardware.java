package org.firstinspires.ftc.teamcode.Archive.MeetAlbaBeclean.Auto;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Current.Recognition.HuskyObjTracking;

@Config
public class Hardware {
    public DcMotorEx leftMotorBack, rightMotorBack, rightMotorFront, leftMotorFront, glisieraDr, glisieraSt;
    public Servo avion, pivot, gripperSt, gripperDr;
    public Navigation navigation;
    public HuskyObjTracking recognition;
    public HuskyLens cam;

    public static double gripperStDeschis = 0.3;
    public static double gripperDrDeschis = 0.5;
    public static double gripperStInchis = 0.7;
    public static double gripperDrInchis = 0.2;
    public static double avionDefault = 0.41;
    public static double avionLansat = 0.5;
    public static double pivotJos = 0;
    public static double pivotScorat = 0.05;

    public Hardware(HardwareMap hardwareMap) throws InterruptedException {
        leftMotorBack = hardwareMap.get(DcMotorEx.class, "stanga_fata");
        rightMotorBack = hardwareMap.get(DcMotorEx.class, "dreapta_spate");
        rightMotorFront = hardwareMap.get(DcMotorEx.class, "dreapta_fata");
        leftMotorFront = hardwareMap.get(DcMotorEx.class, "stanga_spate");

        avion = hardwareMap.get(Servo.class, "avion");
        pivot = hardwareMap.get(Servo.class, "pivot");
        gripperSt = hardwareMap.get(Servo.class, "gripper_st");
        gripperDr = hardwareMap.get(Servo.class, "gripper_dr");

        glisieraDr = hardwareMap.get(DcMotorEx.class, "glisiera_dr");
        glisieraSt = hardwareMap.get(DcMotorEx.class, "glisiera_st");

        cam = hardwareMap.get(HuskyLens.class, "camera");
//        if (!cam.knock()) {
//            telemetry.addData(">>", "Problem communicating");
//        } else {
//            telemetry.addData(">>", "Press start to continue");
//        }
        cam.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        navigation = new Navigation(this);
//        recognition = new HuskyObjTracking(this);

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
        glisieraSt.setDirection(DcMotorSimple.Direction.REVERSE);

        avion.setPosition(avionDefault);
        gripperDr.setPosition(gripperDrInchis);
        gripperSt.setPosition(gripperStInchis);
    }

    public void runWithEncoders() {
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
