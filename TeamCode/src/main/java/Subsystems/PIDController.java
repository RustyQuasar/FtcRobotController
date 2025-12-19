package Subsystems;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.timer = new ElapsedTime();
    }

    public double calculate(double target, double current) {
        // Calculate error
        double error = target - current;

        // Calculate proportional term
        double proportional = error * kP;

        // Calculate integral term
        integralSum += error * timer.seconds();
        double integral = integralSum * kI;

        // Calculate derivative term
        double derivative = (error - lastError) / timer.seconds();
        double derivativeTerm = derivative * kD;

        // Update last error and reset timer
        lastError = error;
        timer.reset();

        // Return the sum of the terms
        return proportional + integral + derivativeTerm;
    }

    // You might add methods to reset the integral sum, change gains, etc.
    public void resetIntegralSum() {
        integralSum = 0;
    }
}