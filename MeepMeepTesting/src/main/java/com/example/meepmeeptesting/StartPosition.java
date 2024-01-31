package com.example.meepmeeptesting;


public enum StartPosition {
    FRONT(0, 1),
    BACK(-24, -1);
    public final double offset;
    public final double direction;

    private StartPosition(double offset, double direction) {
        this.offset = offset;
        this.direction = direction;
    }
}
