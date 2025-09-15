import com.qualcomm.robotcore.hardware.Servo;

public class Constants {

    public static final class DriveTrainConstants {

        //Wheel constants
        public static final String frontLeftMotor = "frontLeft0";
        public static final String frontRightMotor = "frontRight1";
        public static final String backLeftMotor = "backLeft2";
        public static final String backRightMotor = "backRight3";
        //Gyro
        public static final String imu = "imu";
        //idk
        public static final double controlHubOffset = 0;
    }

    public static final class IntakeConstants {

        public static final String leftIntake0 = "leftIntake0";
        public static final String rightIntake1 = "rightIntake1";
    }
    public static final class ShooterConstants {
        public static final String leftShooter = "leftShooter";
        public static final String rightShooter = "rightShooter";
        public static final String turretNeck = "turretNeck";
        public static final String turretHead = "turretHead";
        public static final int centerOffset = 12;
        public static final double TurretRadius = 0; //#TODO: figure out what this is
        public static final int resX = 640;
        public static  final int resY = 480;
        public static final int FOV = 55;
    }

    public static final class ElevatorConstants {
        //Imma see what design is cooking before deleting this
        public static final String elevator = "elevator";
        public static final int highBasket = 2000;
        public static final int lowBasket = 0;
        public static final int exchange = 0;
    }

}

