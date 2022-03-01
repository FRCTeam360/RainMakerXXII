// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.PneumaticConstants.*;

/** Add your docs here. */
public class Intake extends SubsystemBase {
    private static Intake instance;
    public static boolean isIntakeOut;

    /**
     * gets instance for the singleton
     * 
     * @return instance
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private CANSparkMax intake;
    private DoubleSolenoid intakeMover;

    private Intake() {
        this.intake = new CANSparkMax(intakeId, MotorType.kBrushless);
        this.intakeMover = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, intakeForwardChannel, intakeReverseChannel);

        intake.setSmartCurrentLimit(20);

        intake.setInverted(true);
    }

    // motor speed
    public void run(double speed) {
        intake.set(speed);
    }

    public void intakeIn() {
        intakeMover.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeOut() {
        intakeMover.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean getIsIntakeOut() {
        return isIntakeOut;
    }

    @Override
    public void periodic() {
    }

}
