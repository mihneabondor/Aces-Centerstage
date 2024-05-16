package org.firstinspires.ftc.teamcode.Current.Recognition;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.Archive.MeetAlbaBeclean.Auto.Hardware;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

public class HuskyObjTracking {
    SampleMecanumDrive robot;
    Orientation orientation;
    //TODO: aici
    private final short[] casex = {70, 150, 230};

    public HuskyObjTracking(SampleMecanumDrive robot, Orientation orientation) {
        this.robot = robot;
        this.orientation = orientation;
        if(orientation == Orientation.LEFT) casex[0] = 5000;
        else casex[2] = 5000;
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
        if(orientation == Orientation.LEFT)
            return Caz.LEFT;
        return Caz.RIGHT;
    }
}
