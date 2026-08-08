package Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class WrappingPIDFController {

    private double kP, kI, kD, kF, tolerance;

    private double lastError = 0;
    private double integralSum = 0;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    /**
     * Size of one full revolution.
     * <p>
     * Examples:
     * radians -> 2*Math.PI
     * degrees -> 360
     * encoder -> 8192
     */
    private final double wrapRange;

    private final ElapsedTime timer;

    public WrappingPIDFController(double kP, double kI, double kD, double kF, double tolerance, double wrapRange) {

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.tolerance = tolerance;
        this.wrapRange = wrapRange;

        timer = new ElapsedTime();
        timer.reset();
    }

    /**
     * Returns the shortest signed distance between two cyclic values.
     */
    private double wrappingError(double target, double current) {

        double error = target - current;

        error %= wrapRange;

        if (error > wrapRange / 2.0) error -= wrapRange;

        if (error < -wrapRange / 2.0) error += wrapRange;

        return error;
    }

    public double calculate(double target, double current) {

        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 1e-6;

        double error = wrappingError(target, current);

        // Proportional
        double proportional = kP * error;

        // Integral
        integralSum += error * dt;
        double integral = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double derivativeTerm = kD * derivative;

        // Feedforward
        double feedforward = (Math.abs(error) > tolerance) ? Math.signum(error) * kF : 0;

        double output = proportional + integral + derivativeTerm + feedforward;

        double clippedOutput = Range.clip(output, minOutput, maxOutput);

        if (output != clippedOutput) {
            integralSum -= error * dt;
        }

        lastError = error;

        return clippedOutput;
    }

    public void setOutputLimits(double min, double max) {
        minOutput = min;
        maxOutput = max;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}