package Utilities;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class AutoConstants {
    //This was all tuned using PedroPathing's Quickstart repo

    //Inches per encoder tick
    public static double posTolerance = 0.05, velocityTolerance = 0.1;
    //Y-axis difference for the parallel deadwheel (explained on pedropathing tuning)
    public static double parYIn =
            //0;
            -3.6;
    //X-axis difference for the perpendicular deadwheel (explained on pedropathing tuning)
    public static double perpXIn =
            //0;
            -2.5;
    //Information used to control the drivetrain, values change with any bot change
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7.8) //Mass (kg)
            .forwardZeroPowerAcceleration(-24.029400059336396) //Forward acceleration without power, in/s
            .lateralZeroPowerAcceleration(-65.19757687965621) //Sideways acceleration without power, in/s
            //Numbers automatically tuned, used to accurately predict how well the bot brakes
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients( 0.1, 0.08749315601799938, 0.0037664233965230168))
            //PID used when the bot goes off track
            .translationalPIDFCoefficients(new PIDFCoefficients(.5, .2, 0, 0.05))
            //PID used to turn the bot
            .headingPIDFCoefficients(new PIDFCoefficients(0.2, 0.04, 0, 0.05))
            //PID used to drive the bot
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(.02, 0.004, 0, 0.0, 0.05))
            .stuckTimeout(.5)
            ;
    //Movement constraints on the path, these don't matter as much to tune but can still be good at high levels
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 0, 0.736, 1);
    /*
    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            //Inches per tick are the same for our deadwheels, so I initialized them with the same value
            .forwardTicksToInches(inPerTick)
            .strafeTicksToInches(inPerTick)
            //Variables made prior for this
            .strafePodX(perpXIn)
            .forwardPodY(parYIn)
            //Hardware map names, these are the encoder ports the pods are plugged into
            .forwardEncoder_HardwareMapName("backLeft")
            .strafeEncoder_HardwareMapName("backRight")
            //Encoder directions, forward and left should increase your x/y axis in tuning
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            //IMU initialization, "imu" is the hardware map name and the orientation determines the gyro direction tracked
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN))
            ;

     */


    //Mecanum drive constants
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1) //I used this to cap the speed, any higher and there's unmanageable drift
            .useVoltageCompensation(true) //Makes sure we drive the same, whether we have a dead battery or a new, fully charged one
            //Hardware map names of wheels
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            //Wheel directions, usually the right rear motor would also be reversed but we did that in wiring
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //Max velocity of the drivetrain in in/s
            .xVelocity(79.07768059937501)
            .yVelocity(64.78823469902159)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(parYIn)
            .strafePodX(perpXIn)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    //The follower puts everything together
    public static Follower createFollower(HardwareMap hardwareMap) {
        PedroDrivetrain drivetrain = new PedroDrivetrain(hardwareMap, driveConstants);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .setDrivetrain(drivetrain)
                .build();
    }

}
