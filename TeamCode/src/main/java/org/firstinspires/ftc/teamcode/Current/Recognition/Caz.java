package org.firstinspires.ftc.teamcode.Current.Recognition;

public enum Caz {
    LEFT, MID, RIGHT;

    public static Caz fromIndex(int x) {
        switch (x) {
            case 0:
                return LEFT;
            case 1:
                return MID;
            default:
                return RIGHT;
        }
    }
}
