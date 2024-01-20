package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Example VisionPortal OpMode")
@Disabled
public class visionTestOp extends LinearOpMode {
    public String path = "blue-center";

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (isStopRequested()) {
            return;
        }

        PropProcessor prop = new PropProcessor(TeamColour.BLUE);
        VisionPortal visionPortal;
        VisionPortal.Builder builder = new VisionPortal.Builder();
        visionPortal = builder.setCamera(hardwareMap.get(WebcamName.class, "C:/Users/jeffx/Downloads/prop/" + path + ".jpg"))
                .setCameraResolution(new Size(320, 180))
                .addProcessor(prop)
                .build();

        while (opModeIsActive()) {
            telemetry.addData("Location: ", "Pos: " + prop.getPropPosition().toString());
            telemetry.update();
        }
        visionPortal.close();
    }
}
