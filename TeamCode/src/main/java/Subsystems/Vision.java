package Subsystems;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;
import Utilities.Constants;

public class Vision {
    private Limelight3A limelight;
    LLResult result;
    int currentPipeline = 0;

    public Vision(HardwareMap hardwareMap) {
        //13 deg high
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        result = limelight.getLatestResult();
    }

    public LLResult getDetections() {
        return result;
    }
    public boolean hasTarget(){
        return result.isValid();
    }

    public void updateAprilTags() {
        result = limelight.getLatestResult();
    }



    public Vector2d getPose(double neckHeading) {
        limelight.updateRobotOrientation(Math.toDegrees(neckHeading));
        Pose3D botpose = result.getBotpose();
        return new Vector2d(botpose.getPosition().x / 0.0254, botpose.getPosition().y / 0.0254);
    }

    public String[] setColours() {
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                switch (fr.getFiducialId()) {
                    case 21:
                        return new String[]{"G", "P", "P"};
                    case 22:
                        return new String[]{"P", "G", "P"};
                    case 23:
                        return new String[]{"P", "P", "G"};
                }
            }
        }
        return new String[]{"N", "N", "N"};
    }

    public void telemetry(Telemetry telemetry) {
        updateAprilTags();
        result = limelight.getLatestResult();
        telemetry.addData("Status: ", limelight.getStatus());
        //telemetry.addData("Colours: ", Constants.VisionConstants.colours[0] + Constants.VisionConstants.colours[1] + Constants.VisionConstants.colours[2]);
        telemetry.addData("Current pipeline: ", currentPipeline);
        telemetry.addData("Intended pipeline: ", Constants.VisionConstants.pipeline);
        telemetry.addData("Running: ", limelight.isRunning());
        telemetry.addData("Connected: ", limelight.isConnected());
        telemetry.addData("Reading tag: ", result.isValid());

        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose_MT2();
            /*
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();

            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());
             */
            telemetry.addData("Botpose", botpose.getPosition().x / 0.0254 + " " + botpose.getPosition().y / 0.0254 + " " + botpose.getPosition().z / 0.0254);
            telemetry.addData("Heading: ", botpose.getOrientation());
            /*
            // Access barcode results
            List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
            for (LLResultTypes.BarcodeResult br : barcodeResults) {
                telemetry.addData("Barcode", "Data: %s", br.getData());
            }

            // Access classifier results
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
            }

            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
            }

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
             */
        }
    }
}