package Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Utilities.Constants;
import messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer {
    public static class Params {
        double inPerTick = 57.0 / 100000;
        public double par0YTicks = 3/inPerTick; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = -3/inPerTick; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 4.337/inPerTick; // x position of the perpendicular encoder (in tick units)
    }
    public static Params PARAMS = new Params();
    public final Encoder par0, par1, perp;
    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    BNO055IMU imu;
    private double yawOffset;
    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {

        imu = hardwareMap.get(BNO055IMU.class, Constants.DriveTrainConstants.imu);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html

        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontRightMotor)));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.backLeftMotor)));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.frontLeftMotor)));

        par0.setDirection(DcMotorSimple.Direction.REVERSE);

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        Constants.OdometryConstants.fieldPos = initialPose;
        resetYaw();
        initialized = false;
    }

    private double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double rad =  angles.firstAngle;
        if (rad < 0){
            rad += Math.PI * 2;
        }
        return rad;
    }

    public void resetYaw() {
        yawOffset = getRawHeading();
        if (yawOffset > 2 * Math.PI) {
            yawOffset += Math.PI * 2;
        } else if (yawOffset < 0) {
            yawOffset += Math.PI * 2;
        }
    }

    public void update() {
        Pose2d pose = new Pose2d(0, 0, 0);
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            Constants.OdometryConstants.fieldVels = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            initialized = true;
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(PARAMS.inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(PARAMS.inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );
// save last encoder positions (you already do this)
        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

// local twist (robot frame)
        Pose2d localDelta = pose.plus(twist.value());            // .position.x/.position.y are in robot frame
        double dtheta = localDelta.heading.toDouble(); // change in heading (radians)

// previous/global heading before applying this delta
        double prevHeading = Constants.OdometryConstants.fieldPos.heading.toDouble();

// use midpoint heading for rotation (better accuracy for finite rotation)
        double midHeading = prevHeading + dtheta * 0.5;

// rotate local dx,dy into global frame
        double ly = localDelta.position.x;
        double lx = localDelta.position.y;
        double gx = lx * Math.cos(midHeading) - ly * Math.sin(midHeading);
        double gy = lx * Math.sin(midHeading) + ly * Math.cos(midHeading);

// apply to global pose
        double newX = Constants.OdometryConstants.fieldPos.position.x + gx;
        double newY = Constants.OdometryConstants.fieldPos.position.y + gy;

// compute new heading — prefer IMU if you want absolute heading, otherwise integrate:
        double imuHeading = getRawHeading() - yawOffset;
// wrap imuHeading to [0, 2pi)
        while (imuHeading < 0) imuHeading += 2.0 * Math.PI;
        while (imuHeading >= 2.0 * Math.PI) imuHeading -= 2.0 * Math.PI;

// choose which heading to use. If you want to trust IMU:
        double newHeading = imuHeading;
// OR, if you prefer integrated heading from odometry uncomment:
// double newHeading = prevHeading + dtheta;

// set final pose
        Constants.OdometryConstants.fieldPos = new Pose2d(newX, newY, newHeading);

// velocities
        Constants.OdometryConstants.fieldVels = twist.velocity().value();

    }

    public void telemetry(Telemetry telemetry){
        update();
        telemetry.addData("Field pos: ", Constants.OdometryConstants.fieldPos.position);
        telemetry.addData("Field heading: ", Constants.OdometryConstants.fieldPos.heading.toDouble());
        telemetry.addData("Field vel: ", Constants.OdometryConstants.fieldVels);
        telemetry.addData("Par 0 position: ", par0.getPositionAndVelocity().position);
        telemetry.addData("Par 1 position: ", par1.getPositionAndVelocity().position);
        telemetry.addData("Perp position: ", perp.getPositionAndVelocity().position);
    }

}
