// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import static frc.robot.Constants.CANIds.*;
import static frc.robot.Constants.PneumaticConstants.*;

/** Add your docs here. */
public class Intake extends SubsystemBase{
   
    private static Intake instance;
    /**
     * gets instance for the singleton
     * @return instance
     */
    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    private TalonSRX intake;
    private DoubleSolenoid intakeMover;

    private Intake() {
        this.intake = new TalonSRX(intakeId);
        this.intakeMover = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, intakeForwardChannel, intakeReverseChannel);
    }

    //motor speed
    public void run (double speed){
        intake.set(ControlMode.PercentOutput, -speed);
    }

    public void intakeUp() {
        intakeMover.set(DoubleSolenoid.Value.kForward);
    }
    public void intakeDown() {
        intakeMover.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic() {}

    
}
