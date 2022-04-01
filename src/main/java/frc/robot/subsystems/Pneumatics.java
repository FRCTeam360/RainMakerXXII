// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Add your docs here. */
public class Pneumatics extends SubsystemBase{

    public Compressor comp;

    public Pneumatics() {
        //remains unresolved
        comp = new Compressor(20, PneumaticsModuleType.CTREPCM);
    }

    public void pressurize(){
        comp.enableDigital();
    }
    public void stop(){
        comp.disable();
    }
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
