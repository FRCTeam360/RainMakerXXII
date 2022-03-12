// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SetShoot extends CommandBase {

  private final Shooter shooter;
  // private final Feeder feeder;
  private final Limelight myLimelight;
  private final DriverControl driverCont;
  private final OperatorControl operatorCont;

  /** Creates a new setShoot. */
  public SetShoot(Limelight limelight) {
    myLimelight = limelight;
    shooter = Shooter.getInstance();
    driverCont = DriverControl.getInstance();
    operatorCont = OperatorControl.getInstance();
    addRequirements(shooter); 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("Shoot Goal", 2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   //SmartDashboard.getNumber("Shoot Goal", 3500);

    double shootGoal = shooter.getShootGoal();

    if(driverCont.getRightTrigger() || operatorCont.getRightTrigger()){
      shooter.setVelocity(shootGoal);
    } else {
      shooter.coast();
    }

    // SmartDashboard.putNumber("Shoot Goal", shootGoal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.coast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
