package Utilities;

import com.acmerobotics.dashboard.config.Config;

public class Constants {
    public static final String TEAM = "RED"; //Has to be "RED" or "BLUE"
    public static final int StudickaMotorMax = 1464; //360 degrees
    public static final int GoBildaMotorMax = 28; //says 2786 tho //360
    public static final int ServoMax = 1; //180 degrees
    public static final double inToM = 0.0254;
    public static final int defaultDCRPS = (100 / 60); //https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html

    public static final class DriveTrainConstants {

        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor0 = "frontLeft"; //Control hub Motor port 0, connected to front deadwheel
        public static final String frontRightMotor1 = "frontRight"; //Control hub Motor port 1, connected to sideways deadwheel
        public static final String backLeftMotor2 = "backLeft"; //Control hub Motor port 2
        public static final String backRightMotor3 = "backRight"; //Control hub Motor port 3
        //Deadwheel info
        public static final double deadwheelDiameter = 4 * inToM;
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
        //idk
        public static final double controlHubOffset = 0;
        public static final double gearRatio = (61 * 1);
        public static final double wheelDiameter = 5 * inToM; //in meters
    }

    public static final class IntakeConstants {
        //Servo0 (Continuous Rotation - Control Hub)
        public static final String intake = "intake";
        //Control hub - I2C port/bus 1
        public static final String colourSensor = "colourSensor";

    }

    public static final class ShooterConstants {
        //GoBilda
        public static final String leftShooter0 = "leftShooter"; //Expansion hub Motor port 1
        public static final String rightShooter1 = "rightShooter"; //Expansion hub Motor port 3
        public static final double shooterGearRatio = 1; //Gear multiplier
        public static final double shooterAngle = 45; //degrees
        //Servos
        public static final String turretHead = "turretHead"; //Expansion hub Servo port 1
        //CR Servo
        public static final String turretNeckServo = "turretNeckServo"; //Expansion hub Servo port 0
        public static final String transferServo = "transferServo"; //Control hub Servo port 1
        //Encoders
        public static final String turretNeckEncoder = "turretNeckEncoder";
        public static final double turretNeckGearRatio = (double) 22 / 110; //Gear multiplier
        public static final double flyWheelDiameter = 4 * inToM;
        public static final double centerOffset = 12 * inToM;
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

