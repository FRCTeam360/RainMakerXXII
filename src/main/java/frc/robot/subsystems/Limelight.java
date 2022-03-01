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

  private NetworkTableEntry camMode = table.getEntry("camMode");

  public Limelight() {}

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

  // public void updateShooterVelocity () {
  //   if ( validTarget() == true ) {
  //     double yLime = getY();
  //     targetVelocity = (aVal * yLime * yLime) + ( +bVal * yLime) + ( cVal ) ;
  //   } else {
  //     targetVelocity = backupTargetVelocity;
  //   }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lime tY", this.getY());
    // System.out.println("lime tY: " + this.getY());
  }
}
