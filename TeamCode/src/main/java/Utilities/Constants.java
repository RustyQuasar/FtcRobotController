package Utilities;

public class Constants {
    public static final int DCMotorMax = 1464; //360 degrees
    public static final int ServoMax = 1; //180 degrees
    public static final double inToM = 0.0254;
    public static final int defaultDCRPS = (100/60); //https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html
    public static final class DriveTrainConstants {

        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor0 = "frontLeft"; //Control hub Motor port 0
        public static final String frontRightMotor1 = "frontRight"; //Control hub Motor port 1
        public static final String backLeftMotor2 = "backLeft"; //Control hub Motor port 2
        public static final String backRightMotor3 = "backRight"; //Control hub Motor port 3
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
        //idk
        public static final double controlHubOffset = 0;
        public static final double gearRatio = (61 * 1);
        public static final double wheelDiameter = 0.10; //in meters
    }

    public static final class IntakeConstants {
        //Servo0 (Continuous Rotation - Control Hub)
        public static final String intake = "intake";
        //Digital 0:1 (Colour Sensor - Control Hub)
        public static final String colourSensor = "colourSensor"; //#TODO: Figure out what it's actually called and configure accordingly
    }

    public static final class ShooterConstants {
        //Studica Mavericks
        public static final String leftShooter0 = "leftShooter"; //Expansion hub Motor port 0
        public static final String rightShooter1 = "rightShooter"; //Expansion hub Motor port 0
        public static final double shooterGearRatio = (61 * 1); //Gear multiplier
        //TODO: Get actual hardware for neck, head and transfer in configuration
        //Servos
        public static final String turretNeck = "turretNeck"; //Expansion hub Servo port 0
        public static final String turretHead = "turretHead"; //Expansion hub Servo port 1
        //CR Servo
        public static final String transferServo = "transferServo"; //Expansion hub Servo port 2
        public static final double turretNeckGearRatio = (double) 22 /110; //Gear multiplier
        public static final double turretHeadGearRatio = 2; //Gear multiplier
        public static final double turretTransferGearRatio = 2; //Gear multiplier
        public static final double flyWheelDiameter = 4 * inToM;
        public static final int centerOffset = 12;
        public static final double TurretRadius = 0; //#TODO: figure out what this is
    }

    public static final class VisionConstants {
        //Webcam
        public static final String camera = "Webcam"; //USB port
        public static final int resX = 640;
        public static final int resY = 480;
        public static final int FOV = 55;
        public static String[] colours = {"N", "N", "N"};
    }

}

