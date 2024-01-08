package org.firstinspires.ftc.teamcode.Current.Recognition;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

enum Caz {
    LEFT, MID, RIGHT;

    public static Caz fromIndex(int x) {
        switch(x) {
            case 0: return LEFT;
            case 2: return RIGHT;
            default: return MID;
        }
    }
}

public class HuskyObjTracking {
    HuskyLens cam = null;

    //TODO: aici
    private final byte[] casex = {0, 0, 0};

    public HuskyObjTracking(HardwareMap hmap) {
        cam = hmap.get(HuskyLens.class, "camera");
    }

    public Caz getCaz(int id) {
        HuskyLens.Block[] results = cam.blocks();
        for (HuskyLens.Block elem: results) {
            if(elem.id == id) {
                double xMinim = Math.abs(elem.x - casex[0]);
                int index = 0;
                for(int i = 0; i < 3; i++) {
                    double dist = Math.abs(elem.x - casex[i]);
                    if(xMinim < dist) {
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
