package Modes.Autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.AutoConstants;
import Utilities.Constants;

public class Back6 {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;
    int currentPath = 1;
    double lastTime = 10;
    double pathStartTime = 0;
    boolean lastTimeSet = false;
    boolean running = true;
    double pathCooldown = 1000;
    final int loops = 2;
    int loopsComplete = 1;
    public void loop() {
        follower.update();
        Pose2D followerPose = PoseConverter.poseToPose2D(follower.getPose(), FTCCoordinates.INSTANCE);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) + Constants.heading(Math.PI/2) * 2);
        vision.updateAprilTags();
        if (running) {
            shooter.aim(true, false);
            //also mentions of follower.atParametricEnd() but idk how much to trust that
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown) {
                switch(currentPath){
                    case 1:
                        intake.intake(true, false);
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            lastTimeSet = false;
                            follower.followPath(Path1);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                        }
                        break;
                    case 2:
                        follower.followPath(Path2);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                    case 3:
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 4;
                        break;
                    case 4:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path4);
                        currentPath = 5;
                        break;
                    case 5:
                        if (!lastTimeSet){
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                            shooter.transfer(false);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 6;
                            lastTimeSet = false;
                            follower.followPath(Path5);
                        } else {
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;
                    case 6:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path6);
                        currentPath = 7;
                        break;
                    case 7:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path7);
                        currentPath = 8;
                        break;
                        case 8:
                            if (!lastTimeSet){
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime > AutoConstants.farShootTime) {
                                shooter.transfer(false);
                                pathStartTime = System.currentTimeMillis();
                                if (loopsComplete < loops) {
                                    currentPath = 6;
                                    loopsComplete++;
                                } else {
                                    currentPath = 9;
                                }
                                lastTimeSet = false;
                            } else {
                                shooter.transfer(true);
                                intake.intake(true, false);
                            }
                            break;
                    default: running = false;
                }
            }
        } else {
            shooter.chill();
            intake.intake(false, false);
        }
    }

    public void init_loop(){
        pathStartTime = System.currentTimeMillis();
        vision.updateAprilTags();
    }

    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(64), 9, heading(180)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(64.000), 9.000),

                                new Pose(x(50.000), 20.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(50.000), 20.000),

                                new Pose(x(25.000), 22.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(30))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(25.000), 22.000),

                                new Pose(x(10.5), 9.000)
                        )
                ).setLinearHeadingInterpolation(heading(30), heading(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(10.5), 9.000),

                                new Pose(x(58.000), 15.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(180))

                .build();
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(58.000), 15.000),

                                new Pose(x(56.000), 14.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(56.000), 14.000),

                                new Pose(x(14), 10.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(14), 10.000),

                                new Pose(x(58.000), 15.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(180))

                .build();
    }
    private static double x(double offset){
        if (Constants.TEAM.equals("RED")) offset = 144 - offset;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("RED")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}