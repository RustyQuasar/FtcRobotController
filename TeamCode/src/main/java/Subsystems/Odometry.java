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


public class Odometry {

    public static class Params {
        // Offsets (in inches or your chosen units) from robot center
        // +x = forward, +y = left
        public double parallelX = 0.0;
        public double parallelY = Constants.Sizes.robotWidth / 2.0;
        public double perpX = -Constants.Sizes.robotLength / 2.0;
        public double perpY = 0.0;
    }

    public static Params PARAMS = new Params();

    private final Encoder parallel, perpendicular;
    private final BNO055IMU imu;
    private final double inPerTick;

    private int lastParallelPos = 0;
    private int lastPerpPos = 0;
    private double lastHeading = 0.0;
    private double yawOffset = 0.0;

    public Odometry(HardwareMap hardwareMap, Pose2d initialPose) {
        // Assign encoders to deadwheel ports
        parallel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1)));
        perpendicular = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0)));

        this.inPerTick = Constants.OdometryConstants.deadwheelDiameter / Constants.OdometryConstants.externalMax;

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        yawOffset = getRawHeading() - Math.toDegrees(initialPose.heading.toDouble());
        lastHeading = Math.toRadians(getRawHeading() - yawOffset);

        Constants.OdometryConstants.fieldPos = initialPose;

        lastParallelPos = parallel.getPositionAndVelocity().position;
        lastPerpPos = perpendicular.getPositionAndVelocity().position;
    }

    private double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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

        // Encoder positions relative to robot center
        double yPar = PARAMS.parallelY;
        double xPerp = PARAMS.perpX;

        // Compute robot-centric delta
        double dxRobot = dsPar + yPar * dTheta;
        double dyRobot = dsPerp - xPerp * dTheta;

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
        boolean isInSmallTriangle = pose[1] >= pose[0] - (2 * 0.3048) && pose[1] >= -pose[0] + (4 * 12);
        return isInBigTriangle || isInSmallTriangle;
    }
}
