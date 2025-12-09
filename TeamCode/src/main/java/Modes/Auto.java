package Modes;
//package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.*;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ErrorCalculator;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Command;
import Commands.SmartIntake;
import java.util.ArrayList;
import java.util.List;
import com.pedropathing.Drivetrain;
//import java.
@Autonomous(name = "NextFTC Autonomous Program 2 Java")
public class Auto extends OpMode {
//    public AutonomousProgram() {
//        //super(Claw.INSTANCE, Lift.INSTANCE);
//    }
    public static Follower follower;
HardwareMap hardwareMap;
    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();


    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose interPose = new Pose(24 + 72, -24 + 72, Math.toRadians(90));
    private final Pose endPose = new Pose(24 + 72, 24 + 72, Math.toRadians(45));
    private final SmartIntake intake = new SmartIntake(hardwareMap);
    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    public void onSelect() {
        if (follower == null) {
            follower = AutoConstants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = AutoConstants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }
    @Override
    public void loop() {

        follower.update();


        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {

        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("This will run in a roughly triangular shape, starting on the bottom-middle point.");
        telemetryM.debug("So, make sure you have enough space to the left, front, and right to run the OpMode.");
        telemetryM.update(telemetry);
        follower.update();

    }

    /** Creates the PathChain for the "triangle".*/

    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }

//    public Command secondRoutine() {
//        return new SequentialGroup(
//                new ParallelGroup(
//                        new FollowPath(triangle),
//                        intake.intake(true,false)
//                )
//        );
//    }
}
