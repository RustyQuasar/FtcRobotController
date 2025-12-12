package Modes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.Vision;
import Utilities.Constants;

@Autonomous(name = "Blue Front Autonomous", group = "Auto")
public class BlueFrontAuto extends OpMode {
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
        telemetry.addData("Current path: ", currentPath);
        telemetry.addData("Current time: ", System.currentTimeMillis());
        telemetry.addData("Last time: ", lastTime);
        telemetry.addData("Difference: ", System.currentTimeMillis() - lastTime);
        telemetry.addData("Done shooting: ", System.currentTimeMillis() - lastTime > AutoConstants.closeShootTime);
        telemetry.addData("Done path: ", !follower.atParametricEnd() || !follower.isBusy());
        telemetry.addData("Running: ", running);
        telemetry.addData("Current pos: ", follower.getPose());
        //shooter.telemetry(telemetry);
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
        telemetry.update();
    }
    @Override
    public void start(){
        pathStartTime = System.currentTimeMillis();
        follower.followPath(path1);
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
                        new BezierLine(new Pose(x(90), 83.000), new Pose(x(118), 83.000))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(118), 83.000), new Pose(x(85), 95))
                )
                .setLinearHeadingInterpolation(heading(180), heading(40))
                .build();

        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(85), 95), new Pose(x(87), 54))
                )
                .setLinearHeadingInterpolation(heading(40), heading(180))
                .build();

        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(87), 54), new Pose(x(113), 54))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(120), 54), new Pose(x(85), 100))
                )
                .setLinearHeadingInterpolation(heading(180), heading(40))
                .build();

        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(85), 100), new Pose(x(80), 28))
                )
                .setLinearHeadingInterpolation(heading(48), heading(180))
                .build();

        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(80), 28), new Pose(x(114), 28))
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();

        path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(114), 28), new Pose(x(76), 130))
                )
                .setLinearHeadingInterpolation(heading(180), heading(0))
                .build();

        path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(x(74), 110), new Pose(x(120), 69))
                )
                .setLinearHeadingInterpolation(heading(30), heading(0))
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