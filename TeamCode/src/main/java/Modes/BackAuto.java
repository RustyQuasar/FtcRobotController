package Modes;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

@Autonomous(name = "3 Piece + Line Autonomous", group = "Auto")
public class BackAuto extends OpMode {
    public static Follower follower;
    CoordinateSystem system;
    SmartIntake intake;
    SmartShooter3 shooter;
    ThreeDeadWheelLocalizer odometry;
    Vision vision;
    PathChain path;
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (follower.atParametricEnd()) {
                follower.followPath(path, true);
            }
            shooter.aim(true);
            odometry.update();
        }
    }

    @Override
    public void init() {
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8));
        path = follower.pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.000, 8.000), new Pose(110.000, 8.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        vision = new Vision(hardwareMap);
        intake = new SmartIntake(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, new Pose2d(72-8, 72 - x(88), Constants.OdometryConstants.startHeading));
        follower.followPath(path, true);
    }

    @Override
    public void init_loop() {
        follower.update();
        shooter.aim(true);
    }
    public static double x(double offset){
        if (Constants.TEAM.equals("BLUE")) offset += (72-offset) * 2;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle += (90-angle) * 2;
        return angle;
    }
}