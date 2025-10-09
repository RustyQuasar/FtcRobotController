package Utilities;

import com.acmerobotics.roadrunner.Action;

public class ActionUtil {
    public static void runBlocking(Action action) {
        double dt = 0.02; // 20ms loop typical of FTC
        while (!Thread.currentThread().isInterrupted() && !action.isFinished()) {
            action.update(dt);
            try {
                Thread.sleep((long) (dt * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
