// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallCounter extends SubsystemBase {

  private Integer ballCount;
  private static BallCounter instance;

  /** Creates a new BallCounter. */
  private BallCounter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ball Count", ballCount);
  }

  public static BallCounter getInstance() {
    if (instance == null) {
      instance = new BallCounter();
    }
    return instance;
  }

  public void addBallToCount() {
    ballCount += 1;
  }

  public void subtractBallFromCount() {
    if (ballCount > 0) {
      ballCount -= 1;
    }
  }

  public Integer getBallCount() {
    return this.ballCount;
  }

  public void resetCount() {
    ballCount = 0;
  }
}
