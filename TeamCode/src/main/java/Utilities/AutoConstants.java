package Utilities;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Utilities.Constants;

public final class AutoConstants {
    public static double inPerTick = (6.039002650352243E-4 + 5.876961416916444E-4 + 5.881017982674494E-4) / 3;
    public static double parYIn = 1.922; // y position of the first parallel encoder (in tick units)
    public static double perpXIn = 4.222; // x position of the perpendicular encoder (in tick units)
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.74175)
            .forwardZeroPowerAcceleration(-27.369131628964293)
            .lateralZeroPowerAcceleration(-50.83431009168295)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0.0001, 0, 0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(-0.8, 0, 0.001, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.00001, 0.6, 0.01))
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 10, 0.736, 1);
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(inPerTick)
            .strafeTicksToInches(6.122E-4)
            .strafePodX(perpXIn)
            .forwardPodY(parYIn)
            .forwardEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("frontLeft")
            .forwardEncoderDirection(Encoder.REVERSE)
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
                .twoWheelLocalizer(localizerConstants)
                .build();
    }

}
