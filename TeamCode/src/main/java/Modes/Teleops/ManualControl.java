package Modes.Teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

public class ManualControl {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry();
    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizer odometry;
    boolean upLastState = false;
    int shooterVel = 0;
    boolean autoNeck = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, new Pose2d(new Vector2d(0, 0), Constants.heading(Math.PI/2)));
        odometry.update();
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
    }
    public void run(Gamepad gamepad1) {
        Vision.updateAprilTags();
        activeGamepad1.copy(gamepad1);
        if (gamepad1.dpad_up != upLastState && gamepad1.dpad_up) {
            autoNeck = !autoNeck;
        }
        upLastState = gamepad1.dpad_up;
        Intake.intake(true, false);

        if (false) {
            telemetry.addData("State: ", "Servos overriden");
            Shooter.turretHeadTester(activeGamepad1.b);
            Shooter.turretFingerTester(activeGamepad1.y);
            Shooter.shoot(0);
        } else {
            telemetry.addData("State: ", "Shot tuning");
            Shooter.overrideTransfer(true);
            Shooter.aim(false, true);
            if (gamepad1.right_bumper) {
                shooterVel += 4;
            } else if (gamepad1.left_bumper) {
                shooterVel -= 4;
            }
            Shooter.shoot(shooterVel);
        }
        if (activeGamepad1.dpad_down) {
            odometry.resetYaw();
        }
        telemetry.addData("Tuning vel: ", shooterVel);
        Shooter.telemetry(telemetry);
        odometry.telemetry(telemetry);
        //Vision.telemetry(telemetry);
    }
}
