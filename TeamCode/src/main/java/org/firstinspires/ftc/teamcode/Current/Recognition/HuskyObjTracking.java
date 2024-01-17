package org.firstinspires.ftc.teamcode.Current.Recognition;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Current.Hardware;

public class HuskyObjTracking {
    Hardware robot;
    //TODO: aici
    private final short[] casex = {70, 150, 230};

    public HuskyObjTracking(Hardware robot) {
        this.robot = robot;
    }

    public Caz getCaz(int id) {
        HuskyLens.Block[] results = robot.cam.blocks();
        for (HuskyLens.Block elem: results) {
            if(elem.id == id) {
                double xMinim = Math.abs(elem.x - casex[0]);
                int index = 0;
                for(int i = 0; i < 3; i++) {
                    double dist = Math.abs(elem.x - casex[i]);
                    if(xMinim > dist) {
                        xMinim = dist;
                        index = i;
                    }
                }
                return Caz.fromIndex(index);
            }
        }
        return Caz.MID;
    }
}
