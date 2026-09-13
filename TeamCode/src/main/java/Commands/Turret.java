package Commands;

import Subsystems.PIDFController;
import Subsystems.Vision;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class Turret {
    private PIDFController neckController;
    double shooterToBotCenter = 1.541, neckHeading, offsetAngle = 0;
    double targetNeckPos;
    final double totalTicks = Constants.TurretConstants.turretNeckGearRatio * Constants.GoBildaMotorMax;
    final double radianMax = 2 * Math.PI;
    int lastOffset = 0;
    boolean visionUpdate;
    Vision vision;

    public Turret(Vision vision) {
        neckController = new PIDFController(ConfigVariables.neckp, ConfigVariables.necki, ConfigVariables.neckd, ConfigVariables.neckf, 0);
        this.vision = vision;
    }

    public double aim(double xChange, double yChange, double neckCurrentPos, int offset) {
        double botHeading = Constants.OdometryConstants.fieldPos.getHeading();
        if (botHeading < 0) {
            botHeading += radianMax;
        }

        neckHeading = (botHeading - (neckCurrentPos / totalTicks * radianMax)) % radianMax;

        if (visionUpdate) {
            if (offset != lastOffset) offsetAngle = (offset / totalTicks * radianMax);
            Constants.OdometryConstants.fieldPos = vision.getPose(neckHeading + offsetAngle);

        }

        double headingTarget = Math.atan2(xChange, yChange);
        double targetNeckPos = (int) (neckCurrentPos + wrapHeading(headingTarget + neckHeading)) % totalTicks;
        if (targetNeckPos > totalTicks / 2) targetNeckPos -= (int) totalTicks;
        if (targetNeckPos < -totalTicks / 2) targetNeckPos += (int) totalTicks;
        lastOffset = offset;
        return targetNeckPos;

    }

    public double calculateNeckPower(double targetNeckPos, double neckCurrentPos) {
        return neckController.calculate(targetNeckPos, neckCurrentPos);
    }

    private double wrapHeading(double angleToTurnDeg) {
        if (angleToTurnDeg < 0) {
            angleToTurnDeg += radianMax;
        }
        return angleToTurnDeg * Constants.GoBildaMotorMax * Constants.TurretConstants.turretNeckGearRatio / radianMax;
    }
}
