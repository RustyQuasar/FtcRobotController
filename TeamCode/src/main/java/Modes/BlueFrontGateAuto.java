package Modes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.Constants;

@Autonomous(name = "Blue Gate Autonomous", group = "Blue Auto")
public class BlueFrontGateAuto extends OpMode {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    Vision vision;
    PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;
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
                            follower.followPath(path2);
                            pathStartTime = System.currentTimeMillis();
                            currentPath = 2;
                            shooter.transfer(false);
                        } else {
                            shooter.transfer(true);
                            vision.updateAprilTags();
                            intake.intake(true, false);
                        }
                        break;

                        case 2:
                            intake.intake(true, false);
                        follower.followPath(path3);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 3;
                        break;
                        case 3:
                            intake.intake(false, false);
                        follower.followPath(path4);
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
                                follower.followPath(path5);
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
                        follower.followPath(path6);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 6;
                        break;
                        case 6:
                            intake.intake(false, false);
                        follower.followPath(path7);
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
                                follower.followPath(path8);
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
                        follower.followPath(path9);
                            pathStartTime = System.currentTimeMillis();
                        currentPath = 9;
                        break;
                        case 9:
                            intake.intake(false, false);
                        follower.followPath(path10);
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
                            /*
                        case 11:
                            shooter.transfer(false);
                            intake.intake(false, false);
                        follower.followPath(path11);
                        pathStartTime = System.currentTimeMillis();
                        currentPath = 12;
                        break;
                             */
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
        follower.followPath(path1);
        shooter.aim(false, false);
    }
    @Override
    public void init() {
        vision = new Vision(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        intake = new SmartIntake(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(x(110), 136, heading(0)));

        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(110), 136), new Pose(x(100.000), 105.000))
                )
                .setLinearHeadingInterpolation(heading(0), heading(40))
                .build();

        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(100.000), 105.000), new Pose(x(90), 83.000))
                )
                .setLinearHeadingInterpolation(heading(40), heading(180))
                .build();

        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(90), 83), new Pose(x(108), 83))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(108), 83.000), new Pose(x(85), 95))
                )
                .setLinearHeadingInterpolation(heading(180), heading(40))
                .build();

        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(85), 95), new Pose(x(85), 55))
                )
                .setLinearHeadingInterpolation(heading(40), heading(180))
                .build();

        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(85), 55), new Pose(x(92), 55))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(92), 55),
                                new Pose(x(113), 69),
                                new Pose(x(85), 100))
                )
                .setLinearHeadingInterpolation(heading(180), heading(35))
                .build();

        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(85), 100), new Pose(x(80), 30))
                )
                .setLinearHeadingInterpolation(heading(35), heading(180))
                .build();

        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(80), 30), new Pose(x(112), 30))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(114), 30), new Pose(x(73), 135))
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