package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.BACKSTAGE_OFFSET;

public enum StartPosition {
    FRONT(0, 1),
    BACK(BACKSTAGE_OFFSET, -1);
    public final double offset;
    public final double direction;

    private StartPosition(double offset, double direction) {
        this.offset = offset;
        this.direction = direction;
    }
}
