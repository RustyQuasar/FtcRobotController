package Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class HeadingPIDVController {

    private double kP, kI, kD, kV;

    private double lastError = 0;
    private double integralSum = 0;
    private double lastMeasurement = 0;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private ElapsedTime timer;

    public HeadingPIDVController(double kP, double kI, double kD, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.timer = new ElapsedTime();
        timer.reset();
    }

    public double calculate(double target, double current, double angularVelocity) {

        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) dt = 1e-6;

        double error = angleWrap(target - current);

        // --- P ---
        double proportional = kP * error;

        // --- I ---
        integralSum += error * dt;
        double integral = kI * integralSum;

        // --- D (measurement-based to avoid kick) ---
        double derivative = -(current - lastMeasurement) / dt;
        double derivativeTerm = kD * derivative;

        // --- V (this replaces your "F") ---
        double velocityTerm = -kV * angularVelocity;

        double output = proportional + integral + derivativeTerm + velocityTerm;

        double clippedOutput = Range.clip(output, minOutput, maxOutput);

        // Anti-windup
        if (output != clippedOutput) {
            integralSum -= error * dt;
        }

        lastError = error;
        lastMeasurement = current;

        return clippedOutput;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastMeasurement = 0;
        timer.reset();
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}