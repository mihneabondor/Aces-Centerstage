package org.firstinspires.ftc.teamcode.Current;

import static org.firstinspires.ftc.teamcode.Current.Constants.AVION_DESCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.AVION_INCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.GRIPPER_DR_DESCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.GRIPPER_DR_INCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.GRIPPER_ST_DESCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.GRIPPER_ST_INCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PENDUL_DR_JOS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PENDUL_DR_SUS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PENDUL_ST_JOS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PENDUL_ST_SUS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PINBALL_DR_DESCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PINBALL_DR_INCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PINBALL_ST_DESCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PINBALL_ST_INCHIS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PIVOT_JOS;
import static org.firstinspires.ftc.teamcode.Current.Constants.PIVOT_SUS;
import static org.firstinspires.ftc.teamcode.Current.Constants.POWER_GLISIERE_CATARARE;
import static org.firstinspires.ftc.teamcode.Current.Constants.POWER_GLISIERE_JOS;
import static org.firstinspires.ftc.teamcode.Current.Constants.POWER_GLISIERE_SUS;
import static org.firstinspires.ftc.teamcode.Current.Constants.STOP_GLISIERE;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class Hardware {
    public DcMotorEx glisieraSt, glisieraDr;
    public Servo pendulSt, pendulDr, gripperSt, gripperDr, pivot, pinballSt, pinballDr, avion;

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public IMU imu;
    public List<DcMotorEx> motors;


    public Hardware(HardwareMap hardwareMap) throws InterruptedException {
        setConfig(hardwareMap);
        setDirections();
        setDefaults();
        setFrana();
    }

    public void setFrana() {
        motors = Arrays.asList(leftBack, leftFront, rightBack, rightFront, glisieraDr, glisieraSt);
        for (DcMotorEx motor: motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setConfig(HardwareMap hardwareMap) {
        glisieraDr = hardwareMap.get(DcMotorEx.class, "glisiera_dr");
        glisieraSt = hardwareMap.get(DcMotorEx.class, "glisiera_st");
        pivot = hardwareMap.get(Servo.class, "pivot");
        pendulDr = hardwareMap.get(Servo.class, "pendul_dreapta");
        pendulSt = hardwareMap.get(Servo.class, "pendul_stanga");
        gripperSt = hardwareMap.get(Servo.class, "gripper_stanga");
        gripperDr = hardwareMap.get(Servo.class, "gripper_dreapta");

        pinballDr = hardwareMap.get(Servo.class, "pinball_dreapta");
        pinballSt = hardwareMap.get(Servo.class, "pinball_stanga");

        leftFront = hardwareMap.get(DcMotorEx.class, "stanga_fata");
        leftBack = hardwareMap.get(DcMotorEx.class, "stanga_spate");
        rightBack = hardwareMap.get(DcMotorEx.class, "dreapta_spate");
        rightFront = hardwareMap.get(DcMotorEx.class, "dreapta_fata");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        avion = hardwareMap.get(Servo.class, "avion");
    }

    public void setDefaults() throws InterruptedException {
        lowerPendul();
        closeGripper();
        closePinball();
        avion_poz_initiala();
    }

    private void closePinball() {
        closePinballLeft();
        closePinballRight();
    }

    public void setDirections() {
        glisieraDr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void closeGripper() {
        gripperSt.setPosition(GRIPPER_ST_INCHIS);
        gripperDr.setPosition(GRIPPER_DR_INCHIS);
    }

    public void avion_poz_initiala(){
        avion.setPosition(AVION_INCHIS);
    }

    public void openGripper() {
        gripperDr.setPosition(GRIPPER_DR_DESCHIS);
        gripperSt.setPosition(GRIPPER_ST_DESCHIS);
    }

    public void openLeftGripper(){
        gripperSt.setPosition(GRIPPER_ST_DESCHIS);
    }

    public void openRightGripper(){
        gripperDr.setPosition(GRIPPER_DR_DESCHIS);
    }

    public void closeRightGripper(){
        gripperDr.setPosition(GRIPPER_DR_INCHIS);
    }

    public void closeLeftGripper(){
        gripperSt.setPosition(GRIPPER_ST_INCHIS);
    }

    public void raisePivot() {
        pivot.setPosition(PIVOT_SUS);
    }

    public void lowerPivot() {
        pivot.setPosition(PIVOT_JOS);
    }

    public void ascendGlisiere() {
        glisieraSt.setPower(POWER_GLISIERE_SUS);
        glisieraDr.setPower(POWER_GLISIERE_SUS);
    }

    public void descendGlisiere() {
        glisieraSt.setPower(POWER_GLISIERE_JOS);
        glisieraDr.setPower(POWER_GLISIERE_JOS);
    }

    public void stopGlisiere(boolean catarare) {
        double power = catarare ? POWER_GLISIERE_CATARARE : STOP_GLISIERE;
        glisieraDr.setPower(power);
        glisieraSt.setPower(power);
    }

    public void raisePendul() {
        pendulSt.setPosition(PENDUL_ST_SUS);
        pendulDr.setPosition(PENDUL_DR_SUS);
        raisePivot();
    }

    public void lowerPendul() {
        pendulDr.setPosition(PENDUL_DR_JOS);
        pendulSt.setPosition(PENDUL_ST_JOS);
        lowerPivot();
    }

    public void intermediarPendul() {
        pendulDr.setPosition(PENDUL_DR_JOS * 2/3);
        pendulSt.setPosition(PENDUL_ST_JOS * 2/3);
        lowerPivot();
    }

    public void openPinballRight() {
        pinballDr.setPosition(PINBALL_DR_DESCHIS);
    }

    public void openPinballLeft(){
        pinballSt.setPosition(PINBALL_ST_DESCHIS);
    }

    public void closePinballRight() {
        pinballDr.setPosition(PINBALL_DR_INCHIS);
    }

    public void closePinballLeft(){
        pinballSt.setPosition(PINBALL_ST_INCHIS);
    }

    public void decolareeee(){
        avion.setPosition(AVION_DESCHIS);
    }

}
