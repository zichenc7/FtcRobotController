package org.firstinspires.ftc.teamcode.vision;

public enum PropPosition {
    LEFT,
    CENTER,
    RIGHT;

    public PropPosition swap(PropPosition position) {
        if (position.equals(LEFT)) {
            return RIGHT;
        } else if (position.equals(RIGHT)) {
            return LEFT;
        }
        return CENTER;
    }
}
