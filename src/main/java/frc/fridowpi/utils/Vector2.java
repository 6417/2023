package frc.fridowpi.utils;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Vector2 {
    public double x;
    public double y;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Vector2 zero()  {
        return new Vector2(0.0, 0.0);
    }

    public Vector2 add(Vector2 other) {
        return new Vector2(x + other.x, y+other.y);
    }

    public Vector2 smul(double s) {
        return new Vector2(s*x, s*y);
    }

    public double magnitude() {
        return Math.hypot(x, y);
    }

    public double dot(Vector2 other) {
        return x*other.x + y*other.y;
    }

    public double cross(Vector2 other) {
        return x*other.y - other.x * y;
    }

    public double angleTo(Vector2 other) {
        return Math.acos(this.dot(other) / (other.magnitude() * this.magnitude()));
    }
}
