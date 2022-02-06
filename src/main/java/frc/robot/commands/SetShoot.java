// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.operatorInterface.DriverControl;
import frc.robot.operatorInterface.OperatorControl;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class SetShoot extends CommandBase {

  private final Shooter shooter;
  private final Feeder feeder;
  private final Limelight myLimelight;
  private final DriverControl driverCont;
  private final OperatorControl operatorCont;

  public double shootGoal;
  /** Creates a new setShoot. */
  public SetShoot(Limelight limelight) {
    myLimelight = limelight;
    shooter = Shooter.getInstance();
    feeder = Feeder.getInstance();
    driverCont = DriverControl.getInstance();
    operatorCont = OperatorControl.getInstance();
    addRequirements(shooter, feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shoot Goal", 2000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // shootGoal = myLimelight.getY();

    shootGoal = SmartDashboard.getNumber("Shoot Goal", 2000);

    if(driverCont.getRightTrigger() || operatorCont.getRightBumper()){
      shooter.setVelocity(shootGoal);
    } else {
      shooter.setSpeed(0.0);
    }

    // System.out.println("shootgoal" + shootGoal);

    double shootError = Math.abs(shooter.getVelocity() * -1 - shootGoal);

    if(shootError <= 100){
      System.out.println("tower time");
      feeder.runBoth(1);
    }else{
      feeder.runBoth(0);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
    feeder.runBoth(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
