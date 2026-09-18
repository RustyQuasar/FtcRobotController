package Utilities;

import com.pedropathing.math.Pose;

public final class Constants {
    public static boolean onRed;
    public static final int StudickaMotorMax = 24; //360 degrees
    public static final int GoBildaMotorMax = 28;

    public static final class DriveTrainConstants {
        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor = "frontLeft"; //Control hub Motor port 1e, connected to perpendicular deadwheel
        public static final String frontRightMotor = "frontRight"; //Expansion hub Motor port 3e, connected to parallel (1) deadwheel
        public static final String backLeftMotor = "backLeft"; //Control hub Motor port 0e connected to parallel (2) deadwheel
        public static final String backRightMotor = "backRight"; //Expansion hub Motor port 1e
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
        //PIDF Heading constants
        public static double autoAlignmentP = 0.2;
        public static double autoAlignmentI = 0.2;
        public static double autoAlignmentD = 0.05;
        public static double autoAlignmentF = 0.25;
        public static double autoAlignmentTolerance = 1.0/20;
    }

    public static final class FlywheelConstants {
        public static final int hoodMax = 255;
        public static final String flywheel1 = "flywheel1"; //Expansion hub Motor port 0e, rr parallel deadwheel 1
        public static final String flywheel2 = "flywheel2"; //Control hub Motor port 2e, rr parallel deadwheel 2
        public static final String transfer = "transfer"; //Expansion hub Motor port 1
        public static final String turretHeadServo = "turretHood"; //Expansion hub Servo pbort 0
        public enum FlywheelState {
            SCORE,
            PASS,
            FLOWER,
            FIREATWILL
        }


    }

    public static final class OdometryConstants{
        public static Pose fieldPos = new Pose(0, 0, 0);
        public static Double[] fieldVels = {0.0, 0.0};
        public static boolean[] directions = new boolean[2];
    }

    public static final class TurretConstants {
        public static final String turretNeckServo = "turretNeck"; //Expansion hub Motor port 1, rr perpendicular deadwheel
        public static final double turretNeckGearRatio = 19.2 * 197 / 36; //Gear multiplier
        public static final double turretHeadGearRatio = (double) 16 /165; //Gear multiplier
        public enum TurretState {
            AUTO,
            LOCKED,
            MANUAL,
        }

    }

    public static final class IntakeConstants {
        public static final String intake = "intake"; //Control hub Motor port 3
    }

    public static final class VisionConstants {
        //Webcam
        public static double shooterCamDist =0;//inch
        public static double shooterCenterDist =0;//inch
        public static final String camera = "Webcam"; //USB port
        public static final int resX = 320;
        public static final int resY = 240;
        public static final int FOV = 54;
        public static final double inOffset = 4.798 + 0.5;
        public static int cameraAngle = 11;
        public static int pipeline = 0;
    }

    public static final class Sizes {
        public static final double robotWidth = 15.586;
        public static final double robotHeight = 18;
        public static final double robotLength = 17.496;
        public static final double robotOffset = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)) / 2;
        public static final double field = 72;
        public static final double artifactRadius = 2.50;
    }

    public static double heading(double angle) {
        if (onRed) angle *= -1;
        return angle;
    }
}
