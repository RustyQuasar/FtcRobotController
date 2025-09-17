package Utilities;

public class Constants {
    public static final int DCMotorMax = 1464; //360 degrees
    public static final int ServoMax = 1; //180 degrees
    public static final double inToM = 0.0254;
    public static final int defaultDCRPS = (100/60); //https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html
    public static final class DriveTrainConstants {

        //Wheel constants
        public static final String frontLeftMotor0 = "frontLeft";
        public static final String frontRightMotor1 = "frontRight";
        public static final String backLeftMotor2 = "backLeft";
        public static final String backRightMotor3 = "backRight";
        //Gyro
        public static final String imu = "imu";
        //idk
        public static final double controlHubOffset = 0;
        public static final double gearRatio = (61 * 1);
        public static final double wheelDiameter = 0.10; //in meters
    }

    public static final class IntakeConstants {
        public static final String intake = "intake";
        public static final String colourSensor = "colourSensor"; //#TODO: Figure out what it's actually called and configure accordingly
    }

    public static final class ShooterConstants {
        public static final String leftShooter0 = "leftShooter";
        public static final String rightShooter1 = "rightShooter";
        public static final double shooterGearRatio = (61 * 1); //Gear multiplier
        //TODO: Get actual hardware for neck, head and transfer in configuration
        public static final String turretNeck = "turretNeck";
        public static final String turretHead = "turretHead";
        public static final String transferServo = "transferServo";
        public static final double turretNeckGearRatio = 2; //Gear multiplier
        public static final double turretHeadGearRatio = 2; //Gear multiplier
        public static final double turretTransferGearRatio = 2; //Gear multiplier
        public static final double flyWheelDiameter = 4 * inToM;
        public static final int centerOffset = 12;
        public static final double TurretRadius = 0; //#TODO: figure out what this is
    }

    public static final class VisionConstants {
        public static final String camera = "Webcam";
        public static final int resX = 640;
        public static final int resY = 480;
        public static final int FOV = 55;
        public static String[] colours = {"N", "N", "N"};
    }

}

