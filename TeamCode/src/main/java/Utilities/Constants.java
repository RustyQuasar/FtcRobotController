package Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public final class Constants {
    public static final String TEAM =
            "RED"
            //"BLUE"
            ; //Has to be "RED" or "BLUE"
    public static final int StudickaMotorMax = 1464; //360 degrees
    public static final int GoBildaMotorMax = 28; //says 2786 tho //360
    public static final int ServoMax = 1; //180 degrees
    public static final int defaultDCRPS = (100 / 60); //https://docs.wsr.studica.com/en/latest/docs/Motors/maverick-dc-motor.html

    public static final class ElevatorConstants {
        public static final String elevator = "elevator";
        public static final double motorRatio = 1/1; // 1 motor rotation to meters
        public static final double elevatorLength = 12;
    }
    public static final class DriveTrainConstants {

        //Wheel constants (Studica Mavericks)
        public static final String frontLeftMotor0 = "frontLeft"; //Control hub Motor port 0, connected to perpendicular deadwheel
        public static final String frontRightMotor1 = "frontRight"; //Control hub Motor port 1, connected to parallel deadwheel
        public static final String backLeftMotor2 = "backLeft"; //Control hub Motor port 2,
        public static final String backRightMotor3 = "backRight"; //Control hub Motor port 3
        //Gyro (12ICU 0) Something like that idk
        public static final String imu = "imu";
        //idk
        public static final double controlHubOffset = 0;
        public static final double gearRatio = (61 * 1);
        public static final double wheelDiameter = 5; //in meters
    }

    public static final class IntakeConstants {
        //Servo0 (Continuous Rotation - Control Hub)
        public static final String intake = "intake";
        //Control hub - I2C port/bus 1
        public static final String colourSensor = "colourSensor";
        public static final String pressureSensor = "pressureSensor";

    }

    public static final class ShooterConstants {
        //GoBilda
        public static final double smallTriangleDist = 0.23; /*meters*/
        public static final double largeTriangleDist = 0.71; /*meters*/
        public static final String leftShooter0 = "leftShooter"; //Expansion hub Motor port 1
        public static final String rightShooter1 = "rightShooter"; //Expansion hub Motor port 3
        public static final double shooterGearRatio = 1; //Gear multiplier
        //Servos
        public static final String turretHead = "turretHead"; //Expansion hub Servo port 1
        //CR Servo
        public static final String turretNeckServo = "turretNeckServo"; //Expansion hub Servo port 0
        public static final String transferServo = "transferServo"; //Control hub Servo port 1
        //Encoders
        public static final String turretNeckEncoder = "turretNeckEncoder";
        public static final double turretNeckGearRatio = (double) 22 / 110; //Gear multiplier
        public static final double turretHeadGearRatio = 16/120; //Gear multiplier
        public static final double flyWheelDiameter = 4;
        public static final double centerOffset = 12;
    }
    public static final class OdometryConstants{
        public static final double externalMax = 8192; //https://www.revrobotics.com/rev-11-1271, under specifications
        public static final double deadwheelDiameter = 4;
        public static Pose2d fieldPos = new Pose2d(0, 0, 0);
        public static PoseVelocity2d fieldVels = new PoseVelocity2d(new Vector2d(0, 0), 0);


        //0, 0 is blue square corner
        public static final Vector2d targetPosBlue = new Vector2d(0.3, Sizes.field - 0.3);
        public static final Vector2d targetPosRed = new Vector2d(Sizes.field - 0.3, Sizes.field - 0.3);
    }
    public static final class Sizes {
        public static final double robotWidth = 18; //TODO
        public static final double robotHeight = 18; //TODO
        public static final double robotOffset = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotHeight, 2)) / 2; //TODO: Simplify it to the largest side's offset
        public static final double field = 144 - 2 * robotOffset;
        public static final double artifactRadius = 2.50;
    }

    public static final class VisionConstants {
        //Webcam
        public static final String camera = "Webcam"; //USB port
        public static final int resX = 640;
        public static final int resY = 480;
        public static final int FOV = 55;
        public static String[] colours = {"U", "U", "U"};
    }

}

