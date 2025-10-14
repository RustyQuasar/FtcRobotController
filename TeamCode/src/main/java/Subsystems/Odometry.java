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
        // Default physical offsets (robot-centric coordinates):
        // +x = forward, +y = left
        // We assume encoders are mounted at the front (x = +Height/2) and left/right sides (y = ±Width/2).
        // Adjust if your encoders are mounted somewhere else.
        public double parallelX = Constants.DriveTrainConstants.height / 2.0;   // frontRight: forward offset
        public double parallelY = -Constants.DriveTrainConstants.width / 2.0;  // frontRight: right side is negative y

        public double perpX = Constants.DriveTrainConstants.height / 2.0;      // frontLeft: front offset
        public double perpY = +Constants.DriveTrainConstants.width / 2.0;      // frontLeft: left side is positive y
    }

    public static Params PARAMS = new Params();

    // Encoders: parallel = frontRight, perpendicular = frontLeft
    private final Encoder parallel;
    private final Encoder perp;
    private final double inPerTick;
    private final LazyImu imu;

    private int lastParallelPos = 0;
    private int lastPerpPos = 0;
    private double lastHeading = 0.0;   // radians
    private double yawOffset = 0.0;

    public Odometry(HardwareMap hardwareMap, Pose2d initialPose) {
        // Use the same motor ports you told me earlier
        parallel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor1)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor0)));

        // inPerTick: keep your project's constant formula (match units of fieldPos)
        this.inPerTick = Constants.OdometryConstants.deadwheelDiameter / Constants.OdometryConstants.externalMax;

        // IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        // initialize offsets and last heading
        yawOffset = getRawHeading() - Math.toDegrees(initialPose.heading.toDouble()); // keep degrees arithmetic consistent
        lastHeading = Math.toRadians(getRawHeading() - yawOffset);

        // set initial pose
        Constants.OdometryConstants.fieldPos = initialPose;

        // read initial encoder positions
        lastParallelPos = parallel.getPositionAndVelocity().position;
        lastPerpPos = perp.getPositionAndVelocity().position;
    }

    private double getRawHeading() {
        Orientation angles = imu.get().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetYaw() {
        yawOffset = getRawHeading();
    }

    public void update() {
        PositionVelocityPair parPosVel = parallel.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        int parDeltaTicks = parPosVel.position - lastParallelPos;
        int perpDeltaTicks = perpPosVel.position - lastPerpPos;

        lastParallelPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;

        // convert to distance units (same units as fieldPos)
        double dsPar = parDeltaTicks * inPerTick;   // forward/back along robot x
        double dsPerp = perpDeltaTicks * inPerTick; // sideways along robot y

        // heading (absolute) in radians
        double heading = Math.toRadians(getRawHeading() - yawOffset);

        // change in heading since last update
        double dTheta = heading - lastHeading;
        // normalize dTheta to [-pi, pi]
        while (dTheta > Math.PI) dTheta -= 2.0 * Math.PI;
        while (dTheta < -Math.PI) dTheta += 2.0 * Math.PI;

        lastHeading = heading;

        // Encoder positions relative to robot center
        double x_par = PARAMS.parallelX;
        double y_par = PARAMS.parallelY;
        double x_perp = PARAMS.perpX;
        double y_perp = PARAMS.perpY;

        // Robot-centric delta (correcting out rotational contribution to each encoder)
        // Derived:
        //   dsPar = Δx + (- y_par * dθ)  =>  Δx = dsPar + y_par * dθ
        //   dsPerp = Δy + (  x_perp * dθ) => Δy = dsPerp - x_perp * dθ
        double dxRobot = dsPar + y_par * dTheta;
        double dyRobot = dsPerp - x_perp * dTheta;

        // rotate robot-centric delta into field coordinates
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        double dxField = dxRobot * cosH - dyRobot * sinH;
        double dyField = dxRobot * sinH + dyRobot * cosH;

        // update global pose (heading replaced by IMU heading)
        Pose2d prev = Constants.OdometryConstants.fieldPos;
        Constants.OdometryConstants.fieldPos = new Pose2d(
                prev.position.x + dxField,
                prev.position.y + dyField,
                Math.toRadians(heading)
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
