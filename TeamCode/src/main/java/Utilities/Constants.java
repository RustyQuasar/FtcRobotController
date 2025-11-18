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
    public static final int GoBildaMotorMax = 28; //360
    public static final int defaultDCRPS = (100 / 60); //https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html

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
        //idk
        public static final double controlHubOffset = 0;
        public static final double gearRatio = 20;
        public static final double wheelDiameter = 4; //in meters
        public static final double wheelWidth = 1; //in meters
    }

    public static final class IntakeConstants {
        public static final String intake = "intake"; //Expansion hub Motor port 0
        public static final String colourSensor = "colourSensor"; //Control hub - I2C port/bus 1 - Rev Color Sensor v3
        public static final String transferServo = "transferServo"; //Control hub Servo port 2
        public static final String transferServo2 = "transferServo2"; //Control hub Servo port 3
    }

    public static final class ShooterConstants {
        //GoBilda
        public static final double smallTriangleDist = 0.23; /*meters*/
        public static final double largeTriangleDist = 0.71; /*meters*/
        public static final String leftShooter = "leftShooter"; //Expansion hub Motor port 3, rr parallel deadwheel 1
        public static final String rightShooter = "rightShooter"; //Expansion hub Motor port 2, rr parallel deadwheel 2
        public static final String turretNeckMotor = "turretNeck"; //Expansion hub Motor port 0, rr perpendicular deadwheel
        public static final String flipServo = "flipServo"; //Control hub Servo port 1
        //Encoders
        public static final String turretHeadEncoder = "turretHeadEncoder"; //Analog Input Devices 0
        public static final String turretBlockServo = "turretBlock";
        public static final String turretHeadServo = "turretHead"; //Expansion hub Servo port 0
        public static final double shooterGearRatio = 21/15; //Gear multiplier
        public static final double turretNeckGearRatio = (double) 18 / 210; //Gear multiplier
        public static final double turretHeadGearRatio = (double) 16 /165; //Gear multiplier
        public static final double maxHeadAngle = 40;
        public static final double maxNeckAngle = 90;
        public static final double flyWheelDiameter = 4;
        public static final double centerOffset = 12;
    }
    public static final class OdometryConstants{
        public static final double externalMax = 8192; //https://www.revrobotics.com/rev-11-1271, under specifications
        public static final double deadwheelDiameter = 1;
        public static Pose2d fieldPos = new Pose2d(0, 0, 0);
        public static PoseVelocity2d fieldVels = new PoseVelocity2d(new Vector2d(0, 0), 0);
        public static final Vector2d targetPosBlue = new Vector2d(-1 * (Sizes.field/2 - 12), Sizes.field - 12);
        public static final Vector2d targetPosRed = new Vector2d(Sizes.field/2 - 12, Sizes.field - 12);
        public static final Vector2d targetPosMotif = new Vector2d(Sizes.field/2, 0);
        public static Pose2d startPos = new Pose2d(0, 0, 0);
    }
    public static final class Sizes {
        public static final double robotWidth = 18; //TODO
        public static final double robotHeight = 18; //TODO
        public static final double robotLength = 18; //TODO
        public static final double robotOffset = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)) / 2;
        public static final double field = 144 - 2 * robotOffset;
        public static final double artifactRadius = 2.50;
    }

    public static final class VisionConstants {
        //Webcam
        public static final String camera = "Webcam"; //USB port
        public static final int resX = 640;
        public static final int resY = 480;
        public static final int FOV = 55;
        public static final double inOffset = 4.798 + 0.5;
        public static String[] colours = {"N", "N", "N"};
    }

}
