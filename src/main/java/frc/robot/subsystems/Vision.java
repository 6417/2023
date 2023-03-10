// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.apache.logging.log4j.core.util.SystemNanoClock;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import java.lang.Math;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.OwnCommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Constants;
import frc.robot.autonomous_tools.RamseteCommandGenerator;
import frc.robot.commands.autonomous.ChargeAutonomous;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private DoubleArrayTopic r;
  private DoubleArrayTopic d;
  private DoubleArrayTopic p;
  private DoubleArrayTopic i;
  private DoubleArrayTopic rp;
  private DoubleArrayEntry entryr;
  private DoubleArrayEntry entryd;
  private DoubleArrayEntry entryp;
  private DoubleArrayEntry entryi;
  private DoubleArrayEntry entryrp;
  private double[] doublearray = {0.0};
  private double[] tagPos = {5.0, 1.07, 6.80, 6.0, 1.07,4.545, 7.0, 1.07, 2.75, 8.0,  1.07, 0.955, 4.0, 15.47, 6.80, 3.0, 15.47, 4.545, 2.0, 15.47, 2.75, 1.0, 15.47, 0.955}; 
  //private double[] checkpoints = {1, 13.46, 1.2, 2, 8.25 ,1.0 ,3 ,4.58 ,4.5 ,4, 2.18, 4.55, 5, 2.18, 2.75, 6, 2.18, 0.9};
  private int[] tagblue = {4,6,7,8};
  private int[] tagred = {5,3,2,1};
  // tagid, xcord, ycord

  TrajectoryConfig config;
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  Trajectory exampleTrajectory;
  private Command cmd;
  private int modus;
  private long time;

  public static Vision instance;

  private Vision() {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    r = instance.getDoubleArrayTopic("/Vision/rotation");
    d = instance.getDoubleArrayTopic("/Vision/distance");
    p = instance.getDoubleArrayTopic("/Vision/position");
    i = instance.getDoubleArrayTopic("/Vision/tag_id");
    rp = instance.getDoubleArrayTopic("/Vision/relpos");
    entryr = r.getEntry(doublearray,PubSubOption.keepDuplicates(false));
    entryd = d.getEntry(doublearray, PubSubOption.keepDuplicates(false));
    entryp = p.getEntry(doublearray,PubSubOption.keepDuplicates(false));
    entryi = i.getEntry(doublearray, PubSubOption.keepDuplicates(false));
    entryrp = rp.getEntry(doublearray, PubSubOption.keepDuplicates(false));
    modus = 0;
    autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(
                  //DriveConstants.ksVolts,
                  //Constants.Drive.PathWeaver.kI,
                  0.22,
                  //DriveConstants.kvVoltSecondsPerMeter,
                  //Constants.Drive.PathWeaver.kvMetersPerSecoond,
                  1.89,
                  //DriveConstants.kaVoltSecondsSquaredPerMeter),
                  //Constants.Drive.PathWeaver.kvMetersPerSecoond* Constants.Drive.PathWeaver.kvMetersPerSecoond),
                  0.2),
              //DriveConstants.kDriveKinematics,
              new DifferentialDriveKinematics(Constants.Drive.Odometry.trackWidthMeters),
              10);
              
    config =
    new TrajectoryConfig(
            //Constants.Drive.PathWeaver.kvMetersPerSecoond,
            3.3,
            //Constants.Drive.PathWeaver.kMaxAcceleration)
            1.2)
            //AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(new DifferentialDriveKinematics(Constants.Drive.Odometry.trackWidthMeters))
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
    exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1,0, new Rotation2d(0)),
            // Pass config
            config);
    cmd = RamseteCommandGenerator.generateRamseteCommand(exampleTrajectory);
            
  }

  public double getabsd(double d){
    int data;
    double dd;
    if (d == 0){
      data = 0;
    }else{
      data = 1;
    }
    dd = d + 0.1*data + (d-1)*0.1;
    //System.out.println(dd);
    return dd;
  }

  public double[] calculatePath(){
    Drive.getInstance().drive(0, 0, 0);
    double[] pos = getPos();
    double[] rot = getRot();
    double[] dis = getDis();
    int id = getTagId();
    double[] zielpos = {1.07,1.9};
    double[] drivelange = {pos[0], pos[2]};
    double angle = getAngle(rot, dis, pos, zielpos, id, drivelange);
    double[] data = {drivelange[0], drivelange[1], angle};
    return data;
  }


  public void createTrajcetory(){
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(getabsd(1), getabsd(0.3)), new Translation2d(getabsd(2), getabsd(-0.3))),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(getabsd(3), 0, new Rotation2d(0)),
        // Pass config
        config);
  }

  public void aimetag(int tag){
    /* 
    double[] p = getPos();
    double[] r = getRot();
    double[] tagp = getTagData(tag);
    double[] rp = getrPos();
    double x = tagp[0] - p[0];
    double y = tagp[1] - p[1];
    //System.out.println(tagp[0] + " - " + p[0] + " = " + (tagp[0]-p[0]));
    //System.out.println(tagp[1] + " - " + p[1] + " = " + (tagp[1]-p[1]));
    //System.out.println(rp[0]);
    */
    double[] rp = getrPos();
    double[] r = getRot();
    //System.out.println(rp[0]);
    //System.out.println(rp[2]);

    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(rp[2]-0.5, rp[0], new Rotation2d(0)),
        // Pass config
        config);
  }

  public void drive(){
    //createTrajcetory();
    //aimetag(1);
    
    cmd = RamseteCommandGenerator.generateRamseteCommand(exampleTrajectory);

    CommandScheduler.getInstance().schedule(cmd);
  }

  public void aimeline(Pose2d apos){
    double x = 2.29;
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(x-apos.getX(),0, new Rotation2d(0)),
        // Pass config
        config);
  }

  public void driveForward(){
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.9,0, new Rotation2d(0)),
        // Pass config
        config);
  }
  
  public void turn(double rot){
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1,0, new Rotation2d(rot)),
        // Pass config
        config);
  }

  public void driveline(Pose2d pose){
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.2,0, new Rotation2d(0)),
        // Pass config
        config);
  }

  public void rotnintydegrees(){
    exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0.2,0, new Rotation2d(Math.PI/2)),
        // Pass config
        config);
  }

  public void selectstation(int modus){
    Pose2d pos = Drive.getInstance().getPosition();
    //System.out.println(pos.getY());
    double rot = FridoNavx.getYawOffset();
    rot /= 360;
    rot *= 2*Math.PI;
    turn(rot);
    drive();
    aimeline(pos);
    drive();
    driveline(pos); //drive in right pos
    drive();
    rotnintydegrees();
    drive();
    driveForward();
    drive();
  }

  public void selectstationcomand(){
    Pose2d pos = Drive.getInstance().getPosition();
    double rot = FridoNavx.getYawOffset();
    rot /= 360;
    rot *= 2*Math.PI;
    //boolean x = cmd.isScheduled();
    boolean x = false;
    System.out.println(System.currentTimeMillis() - time);
    System.out.println(modus);
    if (modus == 0){
      time = System.currentTimeMillis();
    }
    if (modus == 0 && System.currentTimeMillis() - time > 10){
      System.out.println("test");
      modus += 1;
      turn(Math.PI);
      drive();
      
    }
    if(modus == 1 && System.currentTimeMillis() - time > 300000){
      modus += 1;
      aimeline(pos);
      drive();
    }
    if (modus == 2 && System.currentTimeMillis() - time > 6000){
      modus += 1;
      driveline(pos);
      drive();
    }
    if (modus == 3 && System.currentTimeMillis() - time > 9000){
      modus += 1;
      rotnintydegrees();
      drive();
    }
    if (modus == 4 && System.currentTimeMillis() - time > 12000){
      modus += 1;
      driveForward();
      drive();
      //modus = 0;
    }
  }

  public double[] getPos(){
    return entryp.get();
  }

  public double[] getRot(){
    return entryr.get();
  }

  public double[] getDis(){
    return entryd.get();
  }

  public double[] getrPos(){
    return entryrp.get();
  }

  public int getTagId(){
    double[] data = entryi.get();
    double d = data[0];
    return (int) Math.round(d);
  }

  public double getl(double[] x, double[] y){
    return Math.sqrt((x[0] - y[0]) * (x[0] - y[0]) + (x[1] - y[1]) * (x[1] - y[1]));
  }

  public double[] getTagData(int id){
    double id2 = (double) id;
    for (int i = 0; i < tagPos.length; i++){
        if (id2 == tagPos[i]){
            double[] data = {tagPos[i+1], tagPos[i+2]};
            return data;
        }
    }
  double[] data = {0.0,0.0};
    return data;
  }

  public double getAngle(double[] rot, double[] dis, double[] pos, double[] zielpos, int tagId, double[] drivelange){
    double a = getl(zielpos, drivelange);
    double b = getl(zielpos, getTagData(tagId));
    double c = getl(getTagData(tagId), drivelange);
    return Math.acos(a*a + b*b - c*c / 2 * a * b);
  }

  public static Vision getInstance(){
    if (instance == null){
        instance = new Vision();
    }
    return instance;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
