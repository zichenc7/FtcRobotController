package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class TelemetryPacketV2 extends TelemetryPacket {
    private Canvas fieldOverlay;
    private static final Canvas DEFAULT_FIELD = new Canvas();
    static {
        DEFAULT_FIELD.setAlpha(0.25);
        DEFAULT_FIELD.drawImage("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/centerstage.webp", 0, 0, 144, 144);
        DEFAULT_FIELD.setAlpha(1.0);
        DEFAULT_FIELD.drawGrid(0, 0, 144, 144, 7, 7);
    }
    public TelemetryPacketV2 (){
        super(false);
        fieldOverlay = new Canvas();
        fieldOverlay.getOperations().addAll(DEFAULT_FIELD.getOperations());
    }
}
