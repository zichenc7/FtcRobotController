package org.firstinspires.ftc.teamcode.vision;


public enum TeamColour {
    RED(-1),
    BLUE(1);

    public final double direction;

    private TeamColour(double direction) {
        this.direction = direction;
    }
}
