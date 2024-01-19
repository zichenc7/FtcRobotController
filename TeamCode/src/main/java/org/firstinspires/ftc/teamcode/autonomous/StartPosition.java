package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.DriveConstants.BACKSTAGE_OFFSET;

public enum StartPosition {
    FRONT(0),
    BACK(BACKSTAGE_OFFSET);
    public final double offset;

    private StartPosition(double offset) {
        this.offset = offset;
    }
}
