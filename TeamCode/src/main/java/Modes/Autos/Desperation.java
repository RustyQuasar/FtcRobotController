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

public class Desperation {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11, Path12, Path13, Path14, Path15, Path16, Path17, Path18, Path19, Path20, Path21;
    int currentPath = 1;
    double lastTime = 10;
    double pathStartTime = 0;
    boolean lastTimeSet = false;
    boolean running = true;
    double pathCooldown = 1000;
    boolean shooting = false;
    public void loop() {
        follower.update();
        Pose2D followerPose = PoseConverter.poseToPose2D(follower.getPose(), FTCCoordinates.INSTANCE);
        Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.fieldPos.position, followerPose.getHeading(AngleUnit.RADIANS) - Math.PI);
        vision.updateAprilTags();
        if (shooting) {
            shooter.aim(true, false);
        } else {
            if (currentPath >= 10 || currentPath <= 1){
                shooter.aim(false, false);
            } else {
                shooter.lockMotors();
            }
        }
        if (running) {
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown || (System.currentTimeMillis() - pathStartTime> 3000)) {
                switch(currentPath){
                    case 1:
                        follower.followPath(Path2);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 2;
                        break;
                    case 2:
                        shooting = true;
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                    case 3:
                        follower.followPath(Path4);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 4;
                        break;
                    case 4:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 5;
                            follower.followPath(Path5);
                            shooter.transfer(false);
                            shooting = false;
                        } else {
                            shooter.transfer(true);
                            shooting = true;
                        }
                        break;
                    case 5:
                        follower.followPath(Path6);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 6;
                        break;
                    case 6:
                        follower.followPath(Path7);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 7;
                        break;
                    case 7:
                        follower.followPath(Path8);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 8;
                        break;
                    case 8:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 9;
                            follower.followPath(Path9);
                            shooter.transfer(false);
                            shooting = false;
                        } else {
                            shooter.transfer(true);
                            shooting = true;
                        }
                        break;
                    case 9:
                        lastTimeSet = false;
                        follower.followPath(Path10);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 10;
                        break;
                    case 10:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path11);
                        currentPath = 11;
                        break;
                    case 11:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path12);
                        currentPath = 12;
                        break;
                    case 12:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path13);
                        currentPath = 13;
                        break;
                    case 13:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path14);
                        currentPath = 14;
                    case 14:
                        //If we want for there to be only a 9 piece, remove anything afterward and change Path14 to end 48, 115
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 15;
                            shooter.transfer(false);
                            shooting = false;
                            follower.followPath(Path15);
                        } else {
                            shooter.transfer(true);
                            shooting = true;
                        }
                        break;
                    case 15:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path16);
                        currentPath = 16;
                        break;
                    case 16:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path17);
                        currentPath = 17;
                        break;
                    case 17:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path18);
                        currentPath = 18;
                        break;
                    case 18:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path19);
                        shooting = true;
                        currentPath = 19;
                        break;
                    case 19:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path20);
                        currentPath = 20;
                        break;
                    case 20:
                        pathStartTime = System.currentTimeMillis();
                        follower.followPath(Path21);
                        currentPath = 21;
                        break;
                    case 21:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                            lastTimeSet = false;
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 22;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                            shooting = true;
                        }
                        break;
                    default: running = false;
                }
            }
        } else {
            intake.intake(true, false);
            shooter.transfer(true);
        }
    }
    public void start(){
        follower.setPose(new Pose(x(29), 135, heading(180)));
        pathStartTime = System.currentTimeMillis();
        follower.followPath(Path1);
    }

    public void init(HardwareMap hardwareMap, String team) {
        Constants.TEAM = team;
        Constants.OdometryConstants.fieldPos = new Pose2d(-72, 0, Constants.heading(Math.PI/2));
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(29), 135, heading(180)));
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(33.000), 136.000),

                                new Pose(x(48.000), 136.000)
                        )
                ).setConstantHeadingInterpolation(heading(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 136.000),

                                new Pose(x(48.000), 130.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 130.000),

                                new Pose(x(48.000), 130.000)
                        )
                ).setLinearHeadingInterpolation(heading(180), heading(90))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 130.000),

                                new Pose(x(48.000), 84.000)
                        )
                ).setConstantHeadingInterpolation(heading(90))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 84.000),

                                new Pose(x(48.000), 84.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 84.000),

                                new Pose(x(18.000), 84.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(18.000), 84.000),

                                new Pose(x(48.000), 84.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 84.000),

                                new Pose(x(48.000), 84.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(90))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 84.000),

                                new Pose(x(48.000), 60.000)
                        )
                ).setConstantHeadingInterpolation(heading(90))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 60.000),

                                new Pose(x(48.000), 60.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(0))

                .build();

        Path11 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 60.000),

                                new Pose(x(15.000), 60.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path12 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(15.000), 60.000),

                                new Pose(x(48.000), 60.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path13 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 60.000),

                                new Pose(x(48.000), 60.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(90))

                .build();

        Path14 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 60.000),

                                new Pose(x(48.000), 86.000)
                        )
                ).setConstantHeadingInterpolation(heading(90))

                .build();

        Path15 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 86.000),

                                new Pose(x(48.000), 36.000)
                        )
                ).setConstantHeadingInterpolation(heading(90))

                .build();

        Path16 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 36.000),

                                new Pose(x(48.000), 36.000)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(0))

                .build();

        Path17 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 36.000),

                                new Pose(x(15.000), 36.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path18 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(15.000), 36.000),

                                new Pose(x(48.000), 36.000)
                        )
                ).setConstantHeadingInterpolation(heading(0))

                .build();

        Path19 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 36.000),

                                new Pose(x(48.000), 36.000)
                        )
                ).setLinearHeadingInterpolation(heading(0), heading(90))

                .build();

        Path20 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 36.000),

                                new Pose(x(48.000), 115)
                        )
                ).setConstantHeadingInterpolation(heading(90))

                .build();

        Path21 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(x(48.000), 115),

                                new Pose(x(48.000), 115)
                        )
                ).setLinearHeadingInterpolation(heading(90), heading(0))
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