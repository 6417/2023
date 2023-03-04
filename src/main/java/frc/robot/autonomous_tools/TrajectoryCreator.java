package frc.robot.autonomous_tools;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.Drive.Autonomous;


public class TrajectoryCreator {
    private static class Datapoint {
        private double acceleration;
        private double curvature;
        Pose pose;
        private double time;
        private double velocity;

        // Getter Methods

        public double getAcceleration() {
            return acceleration;
        }

        public double getCurvature() {
            return curvature;
        }

        public Pose getPose() {
            return pose;
        }

        public double getTime() {
            return time;
        }

        public double getVelocity() {
            return velocity;
        }

        // Setter Methods

        public void setAcceleration(double acceleration) {
            this.acceleration = acceleration;
        }

        public void setCurvature(double curvature) {
            this.curvature = curvature;
        }

        public void setPose(Pose poseObject) {
            this.pose = poseObject;
        }

        public void setTime(double time) {
            this.time = time;
        }

        public void setVelocity(double velocity) {
            this.velocity = velocity;
        }
    }

    private static class Pose {
        Rotation rotation;
        Translation translation;

        // Getter Methods

        public Rotation getRotation() {
            return rotation;
        }

        public Translation getTranslation() {
            return translation;
        }

        // Setter Methods

        public void setRotation(Rotation rotationObject) {
            this.rotation = rotationObject;
        }

        public void setTranslation(Translation translationObject) {
            this.translation = translationObject;
        }
    }

    private static class Translation {
        private double x;
        private double y;

        // Getter Methods

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        // Setter Methods

        public void setX(double x) {
            this.x = x;
        }

        public void setY(double y) {
            this.y = y;
        }
    }

    private static class Rotation {
        private double radians;

        // Getter Methods

        public double getRadians() {
            return radians;
        }

        // Setter Methods

        public void setRadians(double radians) {
            this.radians = radians;
        }
    }

    private File file;
    private FileWriter writer;
    private Timer timer;
    private double previousTime;
    private double previousVelocity;
    private boolean firstDatapoint = true;

    private GsonBuilder builder;
    Gson gson;

    private List<Datapoint> datapoints;

    public TrajectoryCreator(String filename) {
        try {
            // Create a file
            file = new File(filename);
            file.createNewFile();

            // Create a writer to write to the file
            writer = new FileWriter(filename);

            // Create the json builders
            builder = new GsonBuilder();
            builder.setPrettyPrinting();

            gson = builder.create(); 

            datapoints = new ArrayList<Datapoint>();

            timer = new Timer();
            timer.reset();
            timer.start();

            previousTime = 0;
            previousVelocity = 0;
            
        } catch (IOException e) {
            DriverStation.reportError("Failed to open file: " + file.getAbsolutePath(), e.getStackTrace());
        }
    }

    private double calculateAcceleration(double velocity, double time) {
        return (velocity - previousVelocity) / (time - previousTime);
    }

    private double calculateCurvature(double velocity, double angularVelocity) {
        if (velocity == 0) {
            return 0;
        }
        return angularVelocity / velocity;
    }

    public void addDatapoint(ChassisSpeeds speeds, Pose2d pose) {
        try {
            // Conditions to stop and start
            if ((Math.abs(speeds.vxMetersPerSecond) >= Autonomous.velocityThresholdStart)
                    && firstDatapoint) {
                timer.stop();
                timer.reset();
                timer.start();
                previousTime = -0.1;
            } else if ((Math.abs(speeds.vxMetersPerSecond) < Autonomous.velocityThresholdStart)
                    && firstDatapoint) {
                return;
            } else if (Math.abs(speeds.vxMetersPerSecond) < Autonomous.velocityThresholdEnd) {
                timer.stop();
                return;
            }

            double cTime = timer.get();

            // Create the datapoint object to later store in the json
            Rotation rotation = new Rotation();
            rotation.setRadians(pose.getRotation().getRadians());

            Translation translation = new Translation();
            translation.setX(pose.getX() * Autonomous.positionCorrection);
            translation.setY(pose.getY() * Autonomous.positionCorrection);

            Pose position = new Pose();
            position.setRotation(rotation);
            position.setTranslation(translation);

            Datapoint point = new Datapoint();
            point.setAcceleration(calculateAcceleration(speeds.vxMetersPerSecond, cTime));
            point.setCurvature(calculateCurvature(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond));
            point.setPose(position);
            point.setTime(cTime);
            point.setVelocity(speeds.vxMetersPerSecond);

            // Add the datapoint to the trajectory
            datapoints.add(point);

            previousTime = cTime;
            previousVelocity = speeds.vxMetersPerSecond;
            firstDatapoint = false;

        } catch (Exception e) {
            DriverStation.reportError("Failed to write datapoint: " + file, e.getStackTrace());
        }
    }

    public void close() {
        try {
            // Writing the content to the file
            String jsonContent = gson.toJson(datapoints);
            writer.write(jsonContent);
            writer.close();
        } catch (IOException e) {
            DriverStation.reportError("Unable to write file: " + file.getAbsolutePath(), e.getStackTrace());
        }
    }
}