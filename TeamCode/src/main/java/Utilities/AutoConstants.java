package Utilities;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public final class AutoConstants {
    public static final double closeShootTime = 1800;
    public static final double farShootTime = 4000;
    public static final double gateHoldTime = 1000;
    public static double inPerTick = (5.856422408217959E-4 + 5.910703012522641E-4 + 5.803648488060199E-4 + 5.819446164942314E-4 + 5.831955874526663E-4 + 5.758567437351553E-4 + 5.838536067503198E-4) / 7;
    public static double par0YIn = 1.88; // y position of the first parallel encoder (in tick units)
    public static double par1Yin = -1.88; // y position of the second parallel encoder (in tick units)
    public static double perpXIn = 4.7905; // x position of the perpendicular encoder (in tick units)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.29)
            .forwardZeroPowerAcceleration( -27.369131628964293)
            .lateralZeroPowerAcceleration(  -50.83431009168295)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0.0001, 0, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(-1, 0.001, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00001, 0.6, 0.01))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(inPerTick)
            .strafeTicksToInches(inPerTick)
            .turnTicksToInches(-0.001809168299744426)
            .leftPodY(par1Yin)
            .rightPodY(par0YIn)
            .strafePodX(perpXIn)
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
            .xVelocity(62.339783684715904) //Forward vel
            .yVelocity(46.41357453228209)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }

}
