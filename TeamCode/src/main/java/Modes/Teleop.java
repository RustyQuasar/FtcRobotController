package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.MechanumDrive;
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

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, dashboard);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter(hardwareMap, Constants.TEAM, Vision);
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
            //Mechanum.periodic(telemetry, telemetryPacket);
            Shooter.periodic(telemetry);
            // Intake.periodic(telemetry, telemetryPacket);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
