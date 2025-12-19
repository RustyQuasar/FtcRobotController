package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.Constants;

@Autonomous(name = "Red No Gate Autonomous", group = "Red Auto")
public class RedFrontAuto extends OpMode {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;
    int currentPath = 1;
    double lastTime = 10;
    double pathStartTime = 0;
    boolean lastTimeSet = false;
    boolean running = true;
    double pathCooldown = 1000;
    @Override
    public void loop() {
        follower.update();
        if (running) {
            //also mentions of follower.atParametricEnd() but idk how much to trust that
            if ((!follower.isBusy()) && System.currentTimeMillis() - pathStartTime > pathCooldown) {
                switch(currentPath){
                    case 1:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime + 700) {
                            lastTimeSet = false;
                            follower.followPath(Path2);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            vision.updateAprilTags();
                            shooter.transfer(true);
                            intake.intake(true, false);
                        }
                        break;

                        case 2:
                            intake.intake(true, false);
                        follower.followPath(Path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                        case 3:
                            intake.intake(false, false);
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
                                intake.intake(false, false);
                                lastTimeSet = false;
                                follower.followPath(Path5);
                                pathStartTime = System.currentTimeMillis();
                                currentPath = 5;
                                shooter.transfer(false);
                            } else {
                                shooter.transfer(true);
                                vision.updateAprilTags();
                                intake.intake(true, true);
                            }
                        break;
                        case 5:
                            intake.intake(true, false);
                        follower.followPath(Path6);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 6;
                        break;
                        case 6:
                            intake.intake(false, false);
                        follower.followPath(Path7);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 7;
                        break;
                        case 7:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                                intake.intake(false, false);
                                lastTimeSet = false;
                                follower.followPath(Path8);
                                pathStartTime = System.currentTimeMillis();
                                currentPath = 8;
                                shooter.transfer(false);
                            } else {
                                shooter.transfer(true);
                                vision.updateAprilTags();
                                intake.intake(true, true);
                            }
                        break;
                        case 8:
                            intake.intake(true, false);
                        follower.followPath(Path9);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 9;
                        break;
                        case 9:
                            intake.intake(false, false);
                        follower.followPath(Path10);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 10;
                        break;
                        case 10:
                            pathStartTime = System.currentTimeMillis();
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }if (System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime) {
                                lastTimeSet = false;
                                currentPath = 11;
                            } else {
                            shooter.transfer(true);
                            vision.updateAprilTags();
                            intake.intake(true, true);
                            }
                        break;
                    default: running = false;
                }
            }
            shooter.aim(false, false);
        } else {
            shooter.chill();
            intake.intake(false, false);
            shooter.transfer(false);
        }
    }
    @Override
    public void start(){
        pathStartTime = System.currentTimeMillis();
        follower.followPath(Path1);
        shooter.aim(false, false);
    }
    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(110), 136, heading(0)));

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(111.000), 135.000), new Pose(x(111.000), 110.000))
                )
                .setLinearHeadingInterpolation(heading(0), heading(50))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(111.000), 110.000), new Pose(x(100.000), 83.000))
                )
                .setLinearHeadingInterpolation(heading(50), heading(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(100.000), 83.000), new Pose(x(126.000), 83.000))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(126.000), 83.000),
                                new Pose(x(105.000), 60.000),
                                new Pose(x(141.000), 75.000),
                                new Pose(x(111.000), 110.000)
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(50))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(111.000), 110.000), new Pose(x(100.000), 58.000))
                )
                .setLinearHeadingInterpolation(heading(50), heading(180))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(100.000), 58.000), new Pose(x(136.000), 58.000))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(136.000), 58.000),
                                new Pose(x(95.000), 62.000),
                                new Pose(x(111.000), 110.000)
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(50))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(111.000), 110.000), new Pose(x(100.000), 30.000))
                )
                .setLinearHeadingInterpolation(heading(50), heading(180))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(100.000), 30.000), new Pose(x(135.000), 30.000))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(135.000), 30.000),
                                new Pose(x(90.000), 102.000),
                                new Pose(x(90.000), 135.000)
                        )
                )
                .setLinearHeadingInterpolation(heading(180), heading(0))
                .build();
    }

    @Override
    public void init_loop() {
        follower.update();
    }
    private static double x(double offset){
        if (Constants.TEAM.equals("BLUE")) offset += (72-offset) * 2;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle += (90-angle) * 2;
        return Math.toRadians(angle);
    }
}