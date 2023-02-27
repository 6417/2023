// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import java.lang.Math;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private DoubleArrayTopic r;
  private DoubleArrayTopic d;
  private DoubleArrayTopic p;
  private DoubleArrayTopic i;
  private DoubleArrayEntry entryr;
  private DoubleArrayEntry entryd;
  private DoubleArrayEntry entryp;
  private DoubleArrayEntry entryi;
  private double[] doublearray = {0.0};
  private double[] tagPos = {5.0, 1.07, 6.80, 6.0, 1.07,4.545, 7.0, 1.07, 2.75, 8.0,  1.07, 0.955, 4.0, 15.47, 6.80, 3.0, 15.47, 4.545, 2.0, 15.47, 2.75, 1.0, 115.47, 0.955}; 
  // tagid, xcord, ycord
  private double time;

  public static Vision instance;

    private Vision() {
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    r = instance.getDoubleArrayTopic("/Vision/rotation");
    d = instance.getDoubleArrayTopic("/Vision/distance");
    p = instance.getDoubleArrayTopic("/Vision/position");
    i = instance.getDoubleArrayTopic("/Vision/tag_id");
    entryr = r.getEntry(doublearray,PubSubOption.keepDuplicates(false));
    entryd = d.getEntry(doublearray, PubSubOption.keepDuplicates(false));
    entryp = p.getEntry(doublearray,PubSubOption.keepDuplicates(false));
    entryi = i.getEntry(doublearray, PubSubOption.keepDuplicates(false));
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



  public double[] getPos(){
    return entryp.get();
  }

  public double[] getRot(){
    return entryr.get();
  }

  public double[] getDis(){
    return entryd.get();
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
