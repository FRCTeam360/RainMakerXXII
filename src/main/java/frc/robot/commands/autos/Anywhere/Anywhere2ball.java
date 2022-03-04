// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.Anywhere;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoExtendIntake;
import frc.robot.commands.AutoMoveOnTicks;
import frc.robot.commands.AutoRunFeederAndTower;
import frc.robot.commands.AutoRunIntake;
import frc.robot.commands.AutoSetShoot;
import frc.robot.commands.TurretAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Anywhere2ball extends ParallelRaceGroup {

  Turret turret = Turret.getInstance();
  Limelight limelight = Limelight.getInstance();
  Intake intake = Intake.getInstance();

  /** Creates a new T_R_2ball. */
  public Anywhere2ball(DriveTrain driveTrain, double offset) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    driveTrain.setAngleOffset(offset);

    addCommands(

        new TurretAuto(limelight, turret),

        new AutoSetShoot(),

        new SequentialCommandGroup(

            // new AutoRunFeederAndTower(),

            new AutoExtendIntake(),

            new ParallelRaceGroup(

                new AutoRunIntake(),

                new SequentialCommandGroup(

                    new AutoMoveOnTicks(driveTrain, 1.1),

                    new AutoRunFeederAndTower(),

                    new AutoRunFeederAndTower()
                )
            )
        )
    );
  }
}
