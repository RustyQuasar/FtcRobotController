package Modes.Autos;

import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.match;

import static Utilities.AutoConstants.followerConstants;
import static Utilities.AutoConstants.posTolerance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.*;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.EnumMap;

import Commands.Collector;
import Utilities.AutoConstants;
import Utilities.Constants;

public class Auto_Baseplate {
    Telemetry telemetry; //Dashboard (pretty much)
    public static Follower follower; //Follower
    Collector collector; //Need this for the commands to exist
    CollectorCommands command = CollectorCommands.CONE; //The active command (servo positions in this case), starting with cone
    int currentPath = 1; //Current path being followed
    long pathCooldown = 500, pathStartTime;
    boolean running = true; //Is the auto actively running? Used for subsystems that need constant attention
    boolean newCommand = false; //Become true if a new command is needed
    Command priorCommand;
    EnumMap<CollectorCommands, Command> cases = new EnumMap<>(CollectorCommands.class); //Command map

    public void loop() {
        follower.update(); //Updates followeer
        if (running) {
            if (atTarget() || isStuck()) {
                switch (currentPath) {
                //insert case breaks here
                }
                pathStartTime = System.currentTimeMillis();
            }
            if (newCommand) {
                priorCommand = match(() -> command, cases);
                Scheduler.schedule(priorCommand);
                newCommand = false;
            }
            Scheduler.execute();
            collector.updateHardware();
            telemetry.addData("Action: ", command.name());
            telemetry.addData("Phase: ", currentPath);
            telemetry.addData("Done previous action", !Scheduler.isRunning(priorCommand));
            telemetry.update();
        }
    }

    public void init_loop() {
        pathStartTime = System.currentTimeMillis();
    }

    public void init(HardwareMap hardwareMap, String team) {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Constants.TEAM = team;
        collector = new Collector(hardwareMap);
        follower = AutoConstants.createFollower(hardwareMap);

        collector.openClaw();
        follower.setStartingPose(new Pose(x(38), 28, heading(180)));

        cases.put(CollectorCommands.CONE, lazy(() -> collector.pickupCone()));
        cases.put(CollectorCommands.DIAMOND, lazy(() -> collector.pickupDiamond()));
        cases.put(CollectorCommands.GEM, lazy(() -> collector.pickupGem()));
        cases.put(CollectorCommands.BIN_DIAMOND, lazy(() -> collector.bin().then(collector.pickupDiamond())));
        cases.put(CollectorCommands.BIN_GEM, lazy(() -> collector.bin().then(collector.pickupGem())));
        cases.put(CollectorCommands.BIN, lazy(() -> collector.bin()));

        //Insert paths here

        /* Example:
        line1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(x(38.000), 28.000),
                                new Pose(x(65.000), 28.000),
                                new Pose(x(65.000), 28.000),
                                new Pose(x(65.000), 53.000),
                                new Pose(x(52.000), 52.000)
                        )
                )
                .setConstantHeadingInterpolation(heading(180))
                .build();
         */

    }

    private static double x(double offset) { //Swaps the x depending on alliance
        if (Constants.TEAM.equals("RED")) offset = 144 - offset;
        return offset;
    }

    private static double heading(double angle) { //Swaps the heading depending on alliance
        if (Constants.TEAM.equals("RED")) angle += (90 - angle) * 2;
        return Math.toRadians(angle);
    }

    private boolean atTarget() {
        return follower.atParametricEnd() && follower.getVelocity().getMagnitude() < follower.getCurrentPath().getPathEndVelocityConstraint() && follower.getHeading() < follower.getCurrentPath().getPathEndHeadingConstraint();
    }

    private boolean isStuck(){
        return follower.getVelocity().getMagnitude() < AutoConstants.velocityTolerance && System.currentTimeMillis() - pathStartTime > pathCooldown;
    }

    private void updatePosition(Pose2d pose){
        Pose pedroPose = new Pose(pose.getX(), pose.getY(), pose.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        if (pedroPose.initialized() || pedroPose.roughlyEquals(follower.getPose(), posTolerance)) return;
        follower.pausePathFollowing();
        follower.setPose(pedroPose);
        follower.resumePathFollowing();
    }

    public enum CollectorCommands { //Variable defining
        CONE,
        GEM, 
        DIAMOND,
        BIN_DIAMOND,
        BIN_GEM,
        BIN
    }
}