package Modes.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.AutoConstants;
import Utilities.Constants;

public class Back3 {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain path;
    int currentPath = 1;
    boolean running = true;
    boolean lastTimeSet = false;
    double lastTime = 0;

    public void loop() {
        follower.update();
        Pose2D followerPose = PoseConverter.poseToPose2D(follower.getPose(), FTCCoordinates.INSTANCE);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) - Math.PI);
        if (running) {
            if (!follower.isBusy()) {
                switch (currentPath) {
                    case 1:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime < AutoConstants.farShootTime + 1000) {
                            shooter.transfer(true);
                            intake.intake(true, true);
                        } else {
                            shooter.transfer(false);
                            currentPath = 2;
                        }
                        break;
                    case 2:
                        follower.followPath(path);
                        currentPath = 3;
                        break;
                    default:
                        running = false;
                        break;
                }
            }
            shooter.aim(true, false);
            vision.updateAprilTags();
        } else {
            shooter.chill();
            intake.intake(false, false);
        }
    }

    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        Constants.OdometryConstants.fieldPos = new Pose2d(72, 0, Constants.heading(Math.PI/2));
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8, heading(0)));
        path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(88.000), 8.000),
                                new Pose(x(120), 8.000)
                        )
                )
                .setConstantHeadingInterpolation(heading(0))
                .build();
        vision = new Vision(hardwareMap);
        intake = new SmartIntake(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
    }

    public void init_loop() {
        follower.update();
        vision.updateAprilTags();
    }

    public static double x(double offset) {
        if (Constants.TEAM.equals("BLUE"))
            offset += (72 - offset) * 2;
        return offset;
    }

    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle += (90 - angle) * 2;
        return Math.toRadians(angle);
    }
}
