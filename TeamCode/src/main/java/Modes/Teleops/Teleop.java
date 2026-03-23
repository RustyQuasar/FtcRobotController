package Modes.Teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import Commands.MechanumDrive;
import Commands.SmartIntake;
import Commands.SmartShooter3;
import Subsystems.ThreeDeadWheelLocalizer;
import Subsystems.Vision;
import Utilities.Constants;

public class Teleop{
    Runnable shooterCalculations;
    Gamepad driver;
    MechanumDrive Mechanum;
    SmartIntake Intake;
    SmartShooter3 Shooter;
    Vision Vision;
    ThreeDeadWheelLocalizer odometry;
    boolean upLastState = false;
    boolean autoNeck = true;
    public void init(HardwareMap hardwareMap, String team){
        Constants.TEAM = team;
        odometry = new ThreeDeadWheelLocalizer(hardwareMap, new Pose2d(new Vector2d(0, 0), Constants.heading(Math.PI/2)));
        odometry.update();
        driver = new Gamepad();
        Mechanum = new MechanumDrive(hardwareMap);
        Vision = new Vision(hardwareMap);
        Intake = new SmartIntake(hardwareMap);
        Shooter = new SmartShooter3(hardwareMap, Vision);
        shooterCalculations = new Runnable() {
            @Override
            public void run() {
                Shooter.calculateAim(autoNeck, false);
                Vision.updateAprilTags();
            }
        };
    }
    public void run(Gamepad newDriver){
        Shooter.updateVariables();
        shooterCalculations.run();
        Shooter.updateHardware();
        driver.copy(newDriver);
        odometry.update();
        if (driver.dpad_up != upLastState && driver.dpad_up) {
            autoNeck = !autoNeck;
        }
        upLastState = driver.dpad_up;
        driver.copy(driver);
        Intake.intake(driver.right_trigger > 0.5, driver.a);
        Shooter.transfer(driver.left_trigger > 0.3);
        Shooter.manualOffset(driver.left_bumper, driver.right_bumper);
        Mechanum.drive(
                -driver.left_stick_y,
                driver.left_stick_x,
                -driver.right_stick_x
        );
        if (driver.dpad_down) {
            odometry.resetYaw();
        }
        if (driver.dpad_left) {
            Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.resetPosBlue, Constants.OdometryConstants.fieldPos.heading);
        }
        if (driver.dpad_right){
            Constants.OdometryConstants.fieldPos = new Pose2d(Constants.OdometryConstants.resetPosRed, Constants.OdometryConstants.fieldPos.heading);
        }
    }
 
}
