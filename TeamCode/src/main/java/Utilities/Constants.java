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
    public static final class CollectorConstants {
        public static final String claw = "claw";
        public static final String elevator = "elevator";
        public static final String collectorHeight = "collectorHeight";
        public static final String colourSensor = "colourSensor";
        public static final String arm = "arm";
        public static final double armRatio = 2;
        public static final double armServoDeg = 270;
    }
    public static final class Sizes {
        public static final double armLength = 10;
    }
    public static final class ClimberConstants {
        public static final String elevator = "climber";
        public static final double elevatorServoDeg = 270;
    }
    public static double heading(double angle) {
        if (TEAM.equals("BLUE")) angle *= -1;
        return angle;
    }
}
