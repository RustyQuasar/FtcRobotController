package Modes;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Utilities.Constants;

public final class AutoConstants {
    public static final double closeShootTime = 3000;
    public static final double farShootTime = 8000;
    public static final double inPerTick =
            //58.25 / 100000;
            5.959660158235999E-4 * (144.0 / 122) * (75.5 / 78);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.38)
            .forwardZeroPowerAcceleration(-46.1818245724384)
            .lateralZeroPowerAcceleration(-70.40489701170591)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.007, 0, 0, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(-1, 0, 0, 0))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(inPerTick)
            .strafeTicksToInches(inPerTick)
            .turnTicksToInches(0.7762012505734939)
            .leftPodY(-3/inPerTick)
            .rightPodY(3/inPerTick)
            .strafePodX(4.337/inPerTick)
            .leftEncoder_HardwareMapName("frontRight")
            .rightEncoder_HardwareMapName("backLeft")
            .strafeEncoder_HardwareMapName("frontLeft")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(76.66649715055927)
            .yVelocity(61.6460503814876)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }

    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}
