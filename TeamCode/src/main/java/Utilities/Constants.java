package Utilities;

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
        //PIDF Heading constants
        public static double autoAlignmentP = 0.2;
        public static double autoAlignmentI = 0.2;
        public static double autoAlignmentD = 0.05;
        public static double autoAlignmentF = 0.25;

    }
    public static final class CollectorConstants {
        public static final String claw = "claw";
        public static final String elevator = "elevator";
        public static final String colourSensor = "colourSensor"; //I2C Bus 2
        public static final String arm1 = "arm1";
        public static final String arm2 = "arm2";
        public static final String wrist = "wrist";
        public static final double armServoDeg = 270;
    }
    public static final class ClimberConstants {
        public static final String climber = "climber";
    }
    public static double heading(double angle) {
        if (TEAM.equals("BLUE")) angle *= -1;
        return angle;
    }
}
