// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ClimberConstants.*;

/** Add your docs here. */
public class Climber extends SubsystemBase {

    private static CANSparkMax motorLeft;
    private static CANSparkMax motorRight;
    
    public Climber() {
        motorLeft = new CANSparkMax(motorLeftId, MotorType.kBrushless);
        motorRight = new CANSparkMax(motorRightId, MotorType.kBrushless);
        
        motorLeft.setInverted(true);
        motorRight.setInverted(true);

        motorLeft.setIdleMode(IdleMode.kBrake);
        motorRight.setIdleMode(IdleMode.kBrake);

        //Motors will either be falcon or neo, not sure yet but i can change the code if need be
    }
    public void runLeftClimber (double pPow) { motorLeft.set( Math.abs(pPow));}
    public void runRightClimber (double pPow) { motorRight.set( Math.abs(pPow));}


    public void resetClimberEncoders() { motorLeft.getEncoder().setPosition(0); motorRight.getEncoder().setPosition(0); }
   
    public double getLeftPos () { return motorLeft.getEncoder().getPosition();}
    public double getRightPos() { return motorRight.getEncoder().getPosition();}

    public void printouts() {
        SmartDashboard.putNumber("LC Temp", motorLeft.getMotorTemperature());
        SmartDashboard.putNumber("RC Temp", motorRight.getMotorTemperature());
        SmartDashboard.putNumber("LC Temp", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("RC Temp", motorRight.getOutputCurrent());
        //not sure if these will be needed, but these are SmartDashboard values in the RainMakerXXI code
    }

}

