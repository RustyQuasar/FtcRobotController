package Modes;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
    public static double inPerTick =
            //58.25 / 100000;
            0.0008414221034381905 * 53/68;
    //1;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.38)
            .forwardZeroPowerAcceleration(-37.032228508506705)
            .lateralZeroPowerAcceleration(-56.07771593470723)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.007, 0, 0, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(-1, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00001, 0.6, 0.01))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(inPerTick)
            .strafeTicksToInches(inPerTick)
            .turnTicksToInches(-1)
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
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
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
            .xVelocity(61.90324583022347)
            .yVelocity(47.311355927444446)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
