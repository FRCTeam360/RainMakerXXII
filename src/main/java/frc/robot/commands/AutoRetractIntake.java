/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class AutoRetractIntake extends CommandBase {
  private final Intake intake = Intake.getInstance();
  private boolean runIntake;
  private Timer myTimer;

  /**
   * Creates a new AutoExtendIntake.
   */
  public AutoRetractIntake(boolean runIntake) {
    // Use addRequirements() here to declare subsystem dependencies.x
    addRequirements(intake);

    this.runIntake = runIntake;
    myTimer = new Timer();
  }

  /**
   * Creates a new AutoExtendIntake.
   */
  public AutoRetractIntake() {
    // Use addRequirements() here to declare subsystem dependencies.x
    addRequirements(intake);
    myTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intakeIn();

    if (runIntake == true) {
      intake.run(1);
    }
    ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myTimer.stop();
    myTimer.reset();

    intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !runIntake || myTimer.get() > 0.25;
  }
}
