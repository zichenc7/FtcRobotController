package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    public static double DRIVE_MULTI = 0.5;
    public static double ARM_MULTI = 0.5;
    //speed of macro movements
    public static double ARM_MACRO_POWER = 1;
    public static double ARM_ADJUST_POWER = 0.5;
    // as a percentage
    public static double ARM_READJUSTMENT_TOLERANCE = 5;
    public static int ARM_MIN = 0;
    public static int ARM_MAX = 4000;
    public static double DRONE_LAUNCH_POS = 0;
    public static double DRONE_REST_POS = 0.5;
    public static double CLAW_CLOSE = 0.95;
    public static double CLAW_OPEN = 0.62;
    public static double WRIST_UP = 0;
    public static double WRIST_DOWN = 0.6;
    public static double WRIST_MID= 0.23;
    public static double WRIST_INCREMENT = 0.01;
    public static int ARM_POS_OUTPUT = 3100;
    public static int ARM_POS_INTAKE = 250;
    public static double WRIST_OUTPUT = 0;
    public static double WRIST_INTAKE = 0.58;
    public static double DEAD_BAND = 0.05;
    public static double deadband(double x) {
        return abs(x) <= DEAD_BAND ? 0 : x;
    }
    public static double percentDifference(double target, double current) {
        return abs(((target - current) / target) * 100);
    }
    public static boolean USE_WEBCAM = true;

    public static int EXPOSURE_MS = 6;
    public static int GAIN = 125;
    // to be determined
    public static int DESIRED_DISTANCE = 10;
    public static double BACKSTAGE_OFFSET = -24;


    public static String IMAGE_PATH = "/images/centerstage.webp";
    /*
    public static String TFOD_MODEL_ASSET_BLUE = "CenterStage.tflite";
    public static String TFOD_MODEL_ASSET_RED = "CenterStage.tflite";

    public static String TFOD_MODEL_FILE = "/CenterStage.tflite";

     */

    // RoadRunner constants below beware!!!!!!!!!
    // DO NOT CHANGE UNLESS YOU KNOW WHAT YOU'RE DOING!!!!!!!

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, 0);
    // p=25, i=0, d=10, f=13.5

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.889764; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.37; // in
    // need to be testeds

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0185;
    public static double kA = 0.002;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(124.00389847982507);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);

    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
