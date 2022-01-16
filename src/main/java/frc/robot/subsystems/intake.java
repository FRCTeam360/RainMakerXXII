// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

/** Add your docs here. */
public class Intake extends SubsystemBase{

    private TalonSRX intake;

    public Intake() {
        intake = new TalonSRX(intakeId);
    }

    //motor speed
    public void run (double speed){
        intake.set(ControlMode.PercentOutput, -speed);
    }

    @Override
    public void periodic() {}

    
}
