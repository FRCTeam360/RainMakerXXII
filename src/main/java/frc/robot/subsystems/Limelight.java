// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  private NetworkTableEntry tl = table.getEntry("tl");
  private NetworkTableEntry snap = table.getEntry("snapshot");

  private NetworkTableEntry camMode = table.getEntry("camMode");
  private static Limelight instance;


  private double latency;
  private int latencyCounter = 0;

  private Limelight() {}

  public static Limelight getInstance(){
    if(instance == null){
      instance = new Limelight();
    }
    return instance;
  }

  public void changeCamMode() {
    if(camMode.getDouble(0.0) == 1.0){
      table.getEntry("camMode").setNumber(0.0);
      table.getEntry("ledMode").setNumber(1);
    } else {
      table.getEntry("camMode").setNumber(1.0);
      table.getEntry("ledMode").setNumber(1);
    }
  }

  public double getCamMode() {
    return camMode.getDouble(0.0);
  }

  public boolean validTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public boolean isOnTarget(){
    return validTarget() && Math.abs(this.getX()) <= 4;
  }

  public void takeSnapshot(){
    snap.setNumber(1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("isOnTarget", this.isOnTarget());
    SmartDashboard.putNumber("lime tY", this.getY());
    SmartDashboard.putNumber("aimError", this.getX());
    SmartDashboard.putBoolean("valid target", this.validTarget());

    double latency = table.getEntry("tl").getDouble(0) / 1000.0 + (11.0/1000.0);
    if(latency == this.latency){
      latencyCounter++;
    } else {
      latencyCounter = 0;
    }

    this.latency = latency;

    SmartDashboard.putBoolean("limelight comms", latencyCounter < 10);
  }
}
