// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CANIds.*;
import com.revrobotics.CANSparkMax;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonFX myShooterFollow;
  private WPI_TalonFX myShooterLead;
  private static Shooter instance;
  public static Shooter getInstance(){
    if(instance == null) {
      instance = new Shooter();
    }
    return instance;
  }
  private Shooter() {
    myShooterFollow = new WPI_TalonFX(shooterFollowId);
    myShooterLead = new WPI_TalonFX(shooterLeadId);
    myShooterLead.configFactoryDefault();
    myShooterFollow.configFactoryDefault();
    myShooterFollow.follow(myShooterLead);
  }

  public void myShooterLead(double speed) {
    myShooterLead.set(speed);
  }

  public void stopShooter() {
    myShooterLead.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
