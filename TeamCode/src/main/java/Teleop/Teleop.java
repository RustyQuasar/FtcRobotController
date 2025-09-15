package Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter;

@TeleOp
public class Teleop extends LinearOpMode {

    Gamepad activeGamepad1;
    static boolean manualMode;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter Shooter;
    String TEAM;
    @Override
    public void runOpMode() {
        telemetry.update();
        activeGamepad1 = new Gamepad();
        manualMode = true;
        Mechanum = new MechanumDrive(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter(hardwareMap, TEAM);

        waitForStart();

        while (opModeIsActive()) {
            activeGamepad1.copy(gamepad1);
            Intake.intake(activeGamepad1.a);

            if (activeGamepad1.back) {
                manualMode = !manualMode;
            }

            Mechanum.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (activeGamepad1.options) {
                Mechanum.resetYaw();
            }

            if (manualMode) {

            } else {
            //Why is there an else? Idk
            }


            Mechanum.periodic(telemetry);
            telemetry.addData("Manual Mode: ", manualMode);
            telemetry.update();
        }
    }
}
