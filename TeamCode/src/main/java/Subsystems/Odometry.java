package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Utilities.Constants;

@Config
public final class Odometry {
    public static class Params {
        // Offsets of the encoders relative to robot center (in inches, or your chosen unit)
        // +x = forward, +y = left
        public double parallelX = Constants.Sizes.robotHeight / 2.0;   // distance from center to parallel wheel along X
        public double parallelY = 0.0;                                 // typically centered

        public double perpX = 0.0;                                     // perpendicular wheel sits at the robot center front/back
        public double perpY = Constants.Sizes.robotWidth / 2.0;        // distance from center to perpendicular wheel along Y
    }

    public static Params PARAMS = new Params();

    private final Encoder parallel, perpendicular;
    private final LazyImu imu;
    private final double inPerTick;

    private int lastParallelPos = 0;
    private int lastPerpPos = 0;
    private double lastHeading = 0.0;
    private double yawOffset = 0.0;

    public Odometry(HardwareMap hardwareMap, Pose2d initialPose) {
        // Encoders (actual hardware names)
        parallel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1)));
        perpendicular = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0)));

        this.inPerTick = Constants.OdometryConstants.deadwheelDiameter / Constants.OdometryConstants.externalMax;

        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        yawOffset = getRawHeading() - Math.toDegrees(initialPose.heading.toDouble());
        lastHeading = Math.toRadians(getRawHeading() - yawOffset);

        Constants.OdometryConstants.fieldPos = initialPose;

        lastParallelPos = parallel.getPositionAndVelocity().position;
        lastPerpPos = perpendicular.getPositionAndVelocity().position;
    }

    private double getRawHeading() {
        Orientation angles = imu.get().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetYaw() {
        yawOffset = getRawHeading();
    }

    public void update() {
        PositionVelocityPair parData = parallel.getPositionAndVelocity();
        PositionVelocityPair perpData = perpendicular.getPositionAndVelocity();

        int parDeltaTicks = parData.position - lastParallelPos;
        int perpDeltaTicks = perpData.position - lastPerpPos;

        lastParallelPos = parData.position;
        lastPerpPos = perpData.position;

        double dsPar = parDeltaTicks * inPerTick;
        double dsPerp = perpDeltaTicks * inPerTick;

        double heading = Math.toRadians(getRawHeading() - yawOffset);
        double dTheta = heading - lastHeading;

        // normalize heading delta
        while (dTheta > Math.PI) dTheta -= 2 * Math.PI;
        while (dTheta < -Math.PI) dTheta += 2 * Math.PI;

        lastHeading = heading;

        // Extract encoder offsets
        double x_par = PARAMS.parallelX;
        double y_par = PARAMS.parallelY;
        double x_perp = PARAMS.perpX;
        double y_perp = PARAMS.perpY;

        // Compute robot-centric displacement
        double dxRobot = dsPar + y_par * dTheta;
        double dyRobot = dsPerp - x_perp * dTheta;

        // Rotate into field coordinates
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double dxField = dxRobot * cosH - dyRobot * sinH;
        double dyField = dxRobot * sinH + dyRobot * cosH;

        Pose2d prev = Constants.OdometryConstants.fieldPos;
        Constants.OdometryConstants.fieldPos = new Pose2d(
                prev.position.x + dxField,
                prev.position.y + dyField,
                heading
        );
    }

    public boolean isInTriangle() {
        double[] pose = {
                Constants.OdometryConstants.fieldPos.position.x,
                Constants.OdometryConstants.fieldPos.position.y
        };
        boolean isInBigTriangle = pose[1] >= pose[0] && pose[1] >= -pose[0] + 12;
        boolean isInSmallTriangle = pose[1] >= pose[0] - (2 * 0.3048) && pose[1] >= -pose[0] + (4 * 0.3048);
        return isInBigTriangle || isInSmallTriangle;
    }
}
