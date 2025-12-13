package Utilities;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Constants {
    public static final String TEAM =
            "RED"
            //"BLUE"
            ; //Has to be "RED" or "BLUE"
    public static final int StudickaMotorMax = 24; //360 degrees
    public static final int GoBildaMotorMax = 28;


    public static final class ElevatorConstants {
        public static final String elevator = "elevator"; //Control hub Servo port 1
    }//TEST
    public static final class DriveTrainConstants {

        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor = "frontLeft"; //Control hub Motor port 0, connected to perpendicular deadwheel
        public static final String frontRightMotor = "frontRight"; //Control hub Motor port 1, connected to parallel (1) deadwheel
        public static final String backLeftMotor = "backLeft"; //Control hub Motor port 2 connected to parallel (2) deadwheel
        public static final String backRightMotor = "backRight"; //Control hub Motor port 3
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
    }

    public static final class IntakeConstants {
        public static final String intake = "intake"; //Expansion hub Motor port 0
        public static final String transferServo = "transferServo"; //Expansion hub Servo port 2
        public static final String transferServo2 = "transferServo2"; //Expansion hub Servo port 3
    }

    public static final class ShooterConstants {
        public static final int hoodMax = 255;
        public static final String leftShooter = "rightShooter"; //Expansion hub Motor port 3, rr parallel deadwheel 1
        public static final String rightShooter = "leftShooter"; //Expansion hub Motor port 2, rr parallel deadwheel 2
        public static final String turretNeckMotor = "turretNeck"; //Expansion hub Motor port 1, rr perpendicular deadwheel
        public static final String flipServo = "flipServo"; //Expansion hub Servo port 1
        public static final String turretHeadServo = "turretHead"; //Expansion hub Servo port 0
        public static final double turretNeckGearRatio = 19.2 * 197 / 35; //Gear multiplier
        public static final double turretHeadGearRatio = (double) 16 /165; //Gear multiplier
        public static final double maxHeadAngle = 50;
        public static final double maxNeckAngle = 90;
    }
    public static final class OdometryConstants{
        public static Pose2d fieldPos = new Pose2d(0, 0, 0);
        public static PoseVelocity2d fieldVels = new PoseVelocity2d(new Vector2d(0, 0), 0);
        public static final Vector2d targetPosBlue = new Vector2d(-(Sizes.field - 4), -(Sizes.field - 4));
        public static final Vector2d targetPosRed = new Vector2d(-(Sizes.field - 4), (Sizes.field - 4));
        public static final Vector2d targetPosMotif = new Vector2d(Sizes.field, 0);
        public static final Vector2d resetPosRed = new Vector2d((Sizes.field - Sizes.robotOffset), (Sizes.field - Sizes.robotOffset));
        public static final Vector2d resetPosBlue = new Vector2d((Sizes.field - Sizes.robotOffset), -(Sizes.field - Sizes.robotOffset));
        public static double startHeading = heading(Math.PI/2);
        public static Pose2d startPos = new Pose2d(Constants.OdometryConstants.resetPosBlue.x, y(Sizes.robotOffset * 2), startHeading);

        public static Pose2d endPos = new Pose2d(Constants.OdometryConstants.resetPosBlue.x, y(Sizes.robotOffset + 12), startHeading);
        public static boolean[] directions = new boolean[2];
    }
    public static final class Sizes {
        public static final double robotWidth = 15.586;
        public static final double robotHeight = 18;
        public static final double robotLength = 17.496;
        public static final double robotOffset = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)) / 2;
        public static final double field = 72;
        public static final double artifactRadius = 2.50;
    }

    public static final class VisionConstants {
        //Webcam
        public static double shooterCamDist =0;//inch
        public static double shooterCenterDist =0;//inch
        public static final String camera = "Webcam"; //USB port
        public static final int resX = 640;
        public static final int resY = 480;
        public static final int FOV = 54;
        public static final double inOffset = 4.798 + 0.5;
        public static String[] colours = {"N", "N", "N"};
        public static int cameraAngle = 11;
        public static int pipeline = 0;
    }
    public static double y(double offset){
        if (TEAM.equals("BLUE")) offset *= -1;
        return offset;
    }
    private static double heading(double angle) {
        if (TEAM.equals("BLUE")) angle *= -1;
        return angle;
    }
}
