package Commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;

import Utilities.Constants;

public class SmartShooter {
    private final DcMotor leftShooter, rightShooter;
    private final Servo turretNeck, turretHead, transferServo;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0); //Idk what this is but I'm too afraid to delete it
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int aimedTagID;
    private final int resX = 640;
    private final int resY = 480;
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

    public SmartShooter(HardwareMap hardwareMap, String TEAM) {
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");
        turretNeck = hardwareMap.get(Servo.class, "turretNeck");
        turretHead = hardwareMap.get(Servo.class, "turretHead");
        transferServo = hardwareMap.get(Servo.class, "transferServo");
        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (TEAM.equals("Red")) {
            aimedTagID = 24;
        } else {
            aimedTagID = 20;
        }
        initAprilTag();
    }

    private static double rightShooterVelocity(DcMotor rightShooter, int DCMotorMax, double gearRatio) {
        try {
            int initReading = rightShooter.getCurrentPosition();
            Thread.sleep(10);                      // not recommended — use a longer window
            int finalReading = rightShooter.getCurrentPosition();

            double deltaTicks = (double) (finalReading - initReading);
            double deltaTimeSec = 0.01;
            double motorRevs = deltaTicks / (double) DCMotorMax;
            double wheelRevs = motorRevs * gearRatio;          // if your gearRatio is motorRev/wheelRev
            double wheelCircum = Math.PI * 4 * 0.0254;      // 4 * pi * 0.0254 (4in diameter)
            double meters = wheelRevs * wheelCircum;
            double velocity = meters / deltaTimeSec;
            return velocity;

        } catch (Exception e) {
            return 0;
        }
    }

    public void shoot(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void aim(double[] v, String[] colours) {
        double detectedX;
        double distance;
        double fv = v[0];
        double sv = v[1];
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            int angle1, angle2;
            double frontV, sideV;
            if (detection.id == aimedTagID) {
                double neckHeading = (turretNeck.getPosition() * Constants.ShooterConstants.turretNeckGearRatio);
                neckHeading -= (Math.round((neckHeading / 360) - 0.5)) * 360;
                neckHeading += Constants.DriveTrainConstants.controlHubOffset;
                if (fv > 0) {
                    angle1 = 0;
                } else {
                    angle1 = 180;
                }
                if (sv > 0) {
                    angle2 = 90;
                } else {
                    angle2 = 270;
                }
                double[] totals = {neckHeading - angle1, neckHeading - angle2};
                for (int i = 0; i < totals.length; i++) {
                    if (totals[i] > 360) {
                        totals[i] -= 360;
                    }
                    if (totals[i] < 0) {
                        totals[i] *= -1;
                    }
                }
                double angle1Cos = fv * Math.cos(Math.toRadians(totals[0] + angle1));
                ;
                double angle2Cos = sv * Math.cos(Math.toRadians(totals[1] + angle2));
                ;
                if (totals[0] <= totals[1]) {
                    frontV = angle1Cos;
                    sideV = angle2Cos;
                } else {
                    frontV = angle2Cos;
                    sideV = angle1Cos;
                }
                detectedX = detection.ftcPose.x;
                distance = detection.ftcPose.range + (Constants.ShooterConstants.centerOffset * 0.0254);
                turretNeck.setPosition(turretNeck.getPosition() + xTurn(detectedX - (resX / 2), sideV, distance));
                turretHead.setPosition(turretHead.getPosition() + yTurn(distance, frontV));
                /*
                Telemetry scanned info
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                 */
            } else if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                Constants.VisionConstants.colours = setColours(currentDetections, colours);
            }
        }
    }

    public double getShooterVelocity(int DCMotorMax, int gearRatio) {
        if (leftShooter.getPower() != 0 && rightShooter.getPower() != 0) {
            return (leftShooterVelocity(leftShooter, DCMotorMax, gearRatio) + rightShooterVelocity(rightShooter, DCMotorMax, gearRatio)) / 2;
        } else {
            return 0;
        }
    }

    public void transfer() {
        transferServo.setPosition(1);
    }

    private void setShooterVelocity(double x, double y, double currentVelocity) {
        //ChatGPT generated formula, using this prompt: "Physics and math question: Not accounting for air resistance, use the vertex of a parabola (being named x and y respectively), and the use of a slope (gravity/9.8) to find the velocity of a thrown ball needed for the slope."
        double neededVelocity = Math.sqrt((9.8 * (Math.pow(x, 2) + (4 * Math.pow(y, 2)))) / (2 * y));
        neededVelocity -= currentVelocity;
        double power = neededVelocity / (4 * Math.PI * 0.0254 * Constants.ShooterConstants.shooterGearRatio);
        shoot(power);
    }

    public void periodic(Telemetry telemetry) {
        telemetry.addLine("Shooter");
        telemetry.addData("Left Shooter Power: ", leftShooter.getPower());
        telemetry.addData("Right Shooter Power: ", rightShooter.getPower());
        telemetry.addLine("Turret neck position: " + turretNeck.getPosition());
        telemetry.addLine("Turret head position: " + turretHead.getPosition());
        telemetry.update();
    }

    private double xTurn(double xOffset, double velocity, double distance) {
        double angleToTurn = (Constants.VisionConstants.FOV / Constants.VisionConstants.resX) * xOffset;//Degrees to turn
        angleToTurn -= (Math.toDegrees(Math.atan(velocity / distance))) / (180 * Constants.ShooterConstants.turretNeckGearRatio);
        return angleToTurn;
    }

    private double yTurn(double distance, double velocity) {
        //double x = distance/2;
        //double y = -9.8 * (distance/2 - 0) * (distance/2 - ((48-17)*0.0254));
        //ChatGPT gave some assistance with this
        double heightDiffM = (48 - 17 + 5) * 0.0254;
        double c = heightDiffM / 9.8;
        double x = 0.5 * (distance - (c / distance));
        double y = (-98 / 4) * Math.pow((distance - (c / distance)), 2);
        setShooterVelocity(x, y, velocity);
        double angle = Math.toDegrees(Math.atan(2 * y / x));
        return angle / (180 * Constants.ShooterConstants.turretHeadGearRatio);
        //#TODO: This is assuming equal levelling and no air res, fix based on the height difference
    }

    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(resX, resY));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);
    }

    public String[] setColours(List<AprilTagDetection> currentDetections, String[] colours) {
        String[] blank = {"N", "N", "N"};
        if (Arrays.equals(colours, blank)) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 21) {
                    return new String[]{"G", "P", "P"};
                } else if (detection.id == 22) {
                    return new String[]{"P", "G", "P"};
                } else if (detection.id == 23) {
                    return new String[]{"P", "P", "G"};
                }
            }
        }
        return blank;
    }

    private double leftShooterVelocity(DcMotor leftShooter, int DCMotorMax, double gearRatio) {
        try {
            int initReading = leftShooter.getCurrentPosition();
            Thread.sleep(10);                      // not recommended — use a longer window
            int finalReading = leftShooter.getCurrentPosition();

            double deltaTicks = (double) (finalReading - initReading);
            double deltaTimeSec = 0.01;
            double motorRevs = deltaTicks / (double) DCMotorMax;
            double wheelRevs = motorRevs * gearRatio;          // if your gearRatio is motorRev/wheelRev
            double wheelCircum = Math.PI * 4 * 0.0254;      // 4 * pi * 0.0254 (4in diameter)
            double meters = wheelRevs * wheelCircum;
            double velocity = meters / deltaTimeSec;
            return velocity;

        } catch (Exception e) {
            return 0;
        }
    }
}
