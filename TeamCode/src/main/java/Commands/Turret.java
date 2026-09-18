package Commands;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Subsystems.PIDFController;
import Subsystems.Vision;
import Utilities.ConfigVariables;
import Utilities.Constants;

public class Turret {
    private PIDFController neckController;
    private double shooterToBotCenter = 1.541, offsetAngle = 0, neckHeading;
    final double totalTicks = Constants.TurretConstants.turretNeckGearRatio * Constants.GoBildaMotorMax;
    final double radianMax = 2 * Math.PI;
    int lastOffset = 0;
    boolean visionUpdate = true;

    public Turret(HardwareMap hardwareMap) {
        neckController = new PIDFController(ConfigVariables.neckp, ConfigVariables.necki, ConfigVariables.neckd, ConfigVariables.neckf, 0);
    }

    public double aim(double xChange, double yChange, double neckCurrentPos, int offset) {
        double botHeading = Constants.OdometryConstants.fieldPos.heading();
        if (botHeading < 0) {
            botHeading += radianMax;
        }

        neckHeading = (botHeading - (neckCurrentPos / totalTicks * radianMax)) % radianMax;

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

    public double getNeckHeading(){
        return neckHeading;
    }

    private double wrapHeading(double angleToTurnDeg) {
        if (angleToTurnDeg < 0) {
            angleToTurnDeg += radianMax;
        }
        return angleToTurnDeg * Constants.GoBildaMotorMax * Constants.TurretConstants.turretNeckGearRatio / radianMax;
    }
}
