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

  private static final double a = -0.002182938;
  private static final double b = -0.0146528457;
  private static final double c = 2.862058996;
  private static final double d = -46.47695902;
  private static final double e = 3261.531163;

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

    double shootGoal = getShootGoal();

    if(driverCont.getRightTrigger() || operatorCont.getRightTrigger()){
      shooter.setVelocity(shootGoal);
    } else {
      shooter.setSpeed(0.0);
    }

    // System.out.println("shootgoal" + shootGoal);

    SmartDashboard.putNumber("Shoot Goal", shootGoal);

    double shootError = Math.abs(shooter.getVelocity() * -1 - shootGoal);

  }


  /**
   * gets shoot goal as determined by our quartic regression, using limelight y-value
   * @return shootGoal
   */
  public double getShootGoal(){
    double limedY = myLimelight.getY();

    return (a * Math.pow(limedY, 4)) + (b * Math.pow(limedY, 3) + (c * Math.pow(limedY, 2)) + (d * limedY) + e);  

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
