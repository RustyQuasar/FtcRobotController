package Utilities;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Constants {
    public static String TEAM;
    public static final int StudickaMotorMax = 24; //360 degrees
    public static final int GoBildaMotorMax = 28;

    public static final class DriveTrainConstants {

        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor = "frontLeft"; //Control hub Motor port 0, connected to perpendicular deadwheel
        public static final String frontRightMotor = "frontRight"; //Control hub Motor port 1, connected to parallel (1) deadwheel
        public static final String backLeftMotor = "backLeft"; //Control hub Motor port 2 connected to parallel (2) deadwheel
        public static final String backRightMotor = "backRight"; //Control hub Motor port 3
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
    }
    public static final class Sizes {
        public static final double robotWidth = 15.586;
        public static final double robotHeight = 18;
        public static final double robotLength = 17.496;
        public static final double robotOffset = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)) / 2;
        public static final double field = 66;
        public static final double artifactRadius = 2.50;
    }
    public static double heading(double angle) {
        if (TEAM.equals("BLUE")) angle *= -1;
        return angle;
    }
}
