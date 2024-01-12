package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class driveTestop extends OpModeBase{
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveBase(hardwareMap);
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        while(opModeIsActive()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );
        }

    }
}
