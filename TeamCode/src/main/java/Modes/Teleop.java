package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Elevator;
import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter;
import Commands.SmartShooter2;
import Commands.Vision;
import Utilities.Constants;
import Subsystems.Odometry;
@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter Shooter;
    Vision Vision;
    Odometry Odometry;
    Elevator Elevator;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {
        Odometry = new Odometry(hardwareMap, Constants.OdometryConstants.fieldPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, dashboard);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter(hardwareMap, Vision);
        Elevator = new Elevator(hardwareMap);
        boolean lastYInput = false;
        waitForStart();

        while (opModeIsActive()) {
            Odometry.update();
            Vision.updateAprilTags();

            activeGamepad1.copy(gamepad1);
            Intake.intake(true);

            if (activeGamepad1.right_trigger > 0.5) {
                Shooter.transfer(true);
                Intake.colorWipe();
            } else {
                Shooter.transfer(false);
            }
            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
            if (activeGamepad1.dpad_down) {
                Odometry.resetYaw();
            }
            /*
            if (activeGamepad1.left_trigger > 0.1) {
                Shooter.shoot(activeGamepad1.left_trigger * 2040);
            }
             */

            if (activeGamepad1.y && !lastYInput) {
                Elevator.switchState();
            }
            lastYInput = activeGamepad1.y;

            Shooter.aim();
            //Mechanum.periodic(telemetry, telemetryPacket);
            //Shooter.periodic(telemetry);
            Intake.periodic(telemetry);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
