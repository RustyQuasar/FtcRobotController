package Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {

    private double kP, kI, kD, kF;

    private double lastError = 0;
    private double integralSum = 0;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private ElapsedTime timer;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.timer = new ElapsedTime();
        timer.reset();
    }

    public double calculate(double target, double current) {

        double dt = timer.seconds();
        timer.reset();

        if (dt == 0) dt = 1e-6;

        double error = target - current;

        // Proportional
        double proportional = kP * error;

        // Integral
        integralSum += error * dt;
        double integral = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double derivativeTerm = kD * derivative;

        // Feedforward
        double feedforward = kF * target;

        double output = proportional + integral + derivativeTerm + feedforward;

        // Clamp output
        double clippedOutput = Range.clip(output, minOutput, maxOutput);

        // Anti-windup: only accumulate integral if not saturated
        if (output != clippedOutput) {
            integralSum -= error * dt;  // undo last integration step
        }

        lastError = error;

        return clippedOutput;
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
