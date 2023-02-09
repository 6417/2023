package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

public class MotionMagic {
    private double maxVel;
    private double maxAccel;
    private double minAccel;
    private Supplier<Double> position;
    private Optional<Double> setPoint;
    private Optional<Double> startPoint;

    public MotionMagic(double maxVel, double maxAccel, double minAccel, Supplier<Double> position) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.minAccel = minAccel;
        this.setPoint = Optional.empty();
        this.position = position;
        this.startPoint = Optional.empty();
    }

    private double breakDistance() {
        return startPoint.get() + (maxAccel * (setPoint.get() - startPoint.get()) - maxVel * maxVel) / (maxAccel)
                + maxVel * maxVel / (2 * maxAccel);
    }
    
    private double accelDistance() {
        return startPoint.get() + maxVel * maxVel / (2 * maxAccel);
    }

    public void goTo(double setPoint) {
        this.setPoint = Optional.of(setPoint);
        startPoint = Optional.of(position.get());
        System.out.print("accelerating until: ");
        System.out.print(accelDistance());
        System.out.print(", deccelerating from: ");
        System.out.print(breakDistance());
        System.out.print(", start point: ");
        System.out.println(this.startPoint.get());
        System.out.print(", set point: ");
        System.out.println(this.setPoint.get());
        
        if (breakDistance() < accelDistance()) {
            throw new Error("Not implemented");
        }
    }

    public double getAccel() {
        double currentPos = position.get();

        if (currentPos <= accelDistance()) {
            return maxAccel + minAccel;
        } else if (currentPos >= breakDistance()) {
            return -maxAccel + minAccel;
        } else {
            return minAccel;
        }
    }

    public void reset() {
        this.setPoint = Optional.empty();
        this.startPoint = Optional.empty();
    }
}
