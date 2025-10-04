package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MechanumDrive;
import Commands.Odometry;
import Commands.SmartIntake;
import Commands.SmartShooter;
import Commands.Vision;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter Shooter;
    Vision Vision;
    Odometry Odometry;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {

        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, dashboard);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter(hardwareMap, Constants.TEAM, Vision);
        Odometry = new Odometry(Mechanum);
        waitForStart();

        while (opModeIsActive()) {
            Vision.updateAprilTags();
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.right_bumper);

            if (activeGamepad1.right_trigger > 0.5) {
                Shooter.transfer();
                Intake.colorWipe();
            }
            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            if (activeGamepad1.dpad_down) {
                Mechanum.resetYaw();
            }
            Shooter.aim(Mechanum.getDrivetrainVelocities());
            Odometry.updatePose(Mechanum.getDrivetrainVelocities());
            //Mechanum.periodic(telemetry, telemetryPacket);
            Shooter.periodic(telemetry);
            // Intake.periodic(telemetry, telemetryPacket);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
