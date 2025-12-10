package Modes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Utilities.Constants;

@Autonomous(name = "12 Piece Autonomous", group = "Auto")
public class FrontAuto extends OpMode {
    public static Follower follower;
    CoordinateSystem system;
    SmartIntake intake;
    SmartShooter3 shooter;
    PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10;;
    int currentPath = 1;
    double shootTime = 3000;
    double lastTime = 0;
    boolean lastTimeSet = false;
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy() && currentPath < 11) {
            if (follower.atParametricEnd()) {
                switch(currentPath){
                    case 1:
                        if (!lastTimeSet) {
                            lastTime = System.currentTimeMillis();
                            lastTimeSet = true;
                        }
                        if (System.currentTimeMillis() - lastTime < shootTime) {
                        shooter.transfer(true);
                        intake.intake(true, true);
                        return;
                        }
                        lastTimeSet = false;
                        follower.followPath(path2);
                        currentPath = 2;
                        break;
                        case 2:
                            intake.intake(true, false);
                        follower.followPath(path3);
                        currentPath = 3;
                        break;
                        case 3:
                            intake.intake(false, false);
                        follower.followPath(path4);
                        currentPath = 4;
                        break;
                        case 4:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime < shootTime) {
                                shooter.transfer(true);
                                intake.intake(true, true);
                                return;
                            }
                            intake.intake(false, false);
                            lastTimeSet = false;
                        follower.followPath(path5);
                        currentPath = 5;
                        break;
                        case 5:
                            intake.intake(true, false);
                        follower.followPath(path6);
                        currentPath = 6;
                        break;
                        case 6:
                            intake.intake(false, false);
                        follower.followPath(path7);
                        currentPath = 7;
                        break;
                        case 7:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime < shootTime) {
                                shooter.transfer(true);
                                intake.intake(true, true);
                                return;
                            }
                            intake.intake(false, false);
                            lastTimeSet = false;
                        follower.followPath(path8);
                        currentPath = 8;
                        break;
                        case 8:
                            intake.intake(true, false);
                        follower.followPath(path9);
                        currentPath = 9;
                        break;
                        case 9:
                            intake.intake(false, false);
                        follower.followPath(path10);
                        currentPath = 10;
                        break;
                        case 10:
                            if (!lastTimeSet) {
                                lastTime = System.currentTimeMillis();
                                lastTimeSet = true;
                            }
                            if (System.currentTimeMillis() - lastTime < shootTime) {
                                shooter.transfer(true);
                                intake.intake(true, true);
                                return;
                            }
                            lastTimeSet = false;
                        currentPath = 11;
                        break;

                }
            }
        } else {
            shooter.chill();
            intake.intake(false, false);
        }
    }

    @Override
    public void init() {
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(123, 123));
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(123.000, 123.000), new Pose(100.000, 105.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 105.000), new Pose(102.000, 83.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();

        path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.000, 83.000), new Pose(130.000, 83.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 83.000), new Pose(100.000, 105.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .build();

        path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 105.000), new Pose(102.000, 59.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();

        path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.000, 59.000), new Pose(130.000, 59.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 59.000), new Pose(100.000, 105.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .build();

        path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 105.000), new Pose(102.000, 35.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(180))
                .build();

        path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.000, 35.000), new Pose(130.000, 35.000))
                )
                .setTangentHeadingInterpolation()
                .build();

        path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(130.000, 35.000), new Pose(100.000, 105.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(37))
                .build();
    }

    @Override
    public void init_loop() {
        follower.update();
        shooter.aim(true);
    }
    public static double y(double offset){
        if (Constants.TEAM.equals("BLUE")) offset *= -1;
        return offset;
    }
    private static double heading(double angle) {
        if (Constants.TEAM.equals("BLUE")) angle *= -1;
        return angle;
    }
}