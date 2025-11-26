package Modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Commands.Limelight;
import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    boolean test = true; // iddk if I have to say this kill
    Limelight lime;
    ThreeDeadWheelLocalizer odometry;
    //Elevator Elevator;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetry = dashboard.getTelemetry(); //Comment this out before comps
    @Override
    public void runOpMode() {
        lime = new Limelight(hardwareMap, telemetry);
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, Constants.OdometryConstants.fieldPos);
        activeGamepad1 = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap, telemetry);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
        //Elevator = new Elevator(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            lime.runVision(test);
            odometry.update();
            Vision.updateAprilTags();
            Vision.hasTarget();
            Shooter.aim();
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.right_trigger > 0.5, activeGamepad1.a);
            Shooter.transfer(activeGamepad1.left_trigger > 0.3);
            //Shooter.turretHeadTester(activeGamepad1.b);
            //Shooter.shoot(activeGamepad1.left_trigger * 2040);
            if (activeGamepad1.left_trigger > 0.3) {
                Shooter.transfer(true);
            } else {
                Shooter.transfer(false);
            }
            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (activeGamepad1.dpad_down) {
                odometry.resetYaw();
            }
            //Mechanum.telemetry(telemetry);
            Shooter.telemetry(telemetry);
            //Intake.telemetry(telemetry);
            //Vision.telemetry(telemetry);
            odometry.telemetry(telemetry);
            telemetry.update();
            dashboard.updateConfig();
        }
    }
}
