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
import Utilities.AutoConstants;
import Utilities.Constants;

@Autonomous(name = "Back 3", group = "Auto")
public class Back3 extends OpMode {
    public static Follower follower;
    SmartIntake intake;
    SmartShooter3 shooter;
    //ThreeDeadWheelLocalizer odometry;
    Vision vision;

    PathChain path;

    int currentPath = 1;
    boolean running = true;
    boolean lastTimeSet = false;
    double lastTime = 0;

    @Override
    public void loop() {
        //odometry.update();
        shooter.telemetry(telemetry);
        telemetry.update();
        follower.update();
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
                        follower.followPath(path, true);
                        currentPath = 3;
                        break;
                    default:
                        running = false;
                        break;
                }
            }
            shooter.aim(true);
            vision.updateAprilTags();
        } else {
            shooter.chill();
            intake.intake(false, false);
        }
    }

    @Override
    public void start() {
        //follower.followPath(path);
    }

    @Override
    public void init() {
        Constants.OdometryConstants.fieldPos = Constants.OdometryConstants.startPos;
        follower = AutoConstants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88, 8));
        path = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(x(88.000), 8.000),
                                new Pose(x(110.000), 8.000)
                        )
                )
                .setConstantHeadingInterpolation(follower.getHeading())
                .build();
        vision = new Vision(hardwareMap);
        intake = new SmartIntake(hardwareMap);
        shooter = new SmartShooter3(hardwareMap, vision);
        //odometry = new ThreeDeadWheelLocalizer(hardwareMap, new Pose2d(72 - 8, 72 - x(88), Constants.OdometryConstants.startHeading));
    }

    @Override
    public void init_loop() {
        follower.update();
        vision.updateAprilTags();
        //shooter.aim(true);
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
