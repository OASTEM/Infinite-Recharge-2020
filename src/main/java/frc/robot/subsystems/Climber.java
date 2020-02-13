/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  CANSparkMax frontMotor;
  CANSparkMax backMotor;

  CANPIDController frontController;
  CANPIDController backController;

  CANEncoder frontEncoder;
  CANEncoder backEncoder;

  public Climber() {
    frontMotor = new CANSparkMax(5, MotorType.kBrushless);
    backMotor = new CANSparkMax(6, MotorType.kBrushless);

    frontController = frontMotor.getPIDController();
    backController = backMotor.getPIDController();

    frontEncoder = frontMotor.getEncoder();
    frontEncoder = frontMotor.getEncoder(EncoderType.kHallSensor, 42);

    backEncoder = backMotor.getEncoder();
    backEncoder = backMotor.getEncoder(EncoderType.kHallSensor, 42);

    frontController.setFeedbackDevice(frontEncoder);
    backController.setFeedbackDevice(backEncoder);

    frontController.setP(Constants.cPos_kP, Constants.cPos_Slot); 
    frontController.setI(Constants.cPos_kP, Constants.cPos_Slot); 
    frontController.setD(Constants.cPos_kP, Constants.cPos_Slot); 
    frontController .setFF(Constants.cPos_kP, Constants.cPos_Slot);
    backController.setP(Constants.cPos_kP, Constants.cPos_Slot); 
    backController.setI(Constants.cPos_kP, Constants.cPos_Slot); 
    backController.setD(Constants.cPos_kP, Constants.cPos_Slot); 
    backController .setFF(Constants.cPos_kP, Constants.cPos_Slot);
    
    frontMotor.setOpenLoopRampRate(0.5);
    backMotor.setOpenLoopRampRate(0.5);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double power) {
    runFrontMotor(power);
    runBackMotor(power);
  }

  public void runFrontMotor(double power)
  {
    frontMotor.set(power);
  }

  public void runBackMotor(double power)
  {
    backMotor.set(power);
  }

  public void setPosition(double pos) {
    frontController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
    backController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public void setFrontPosition(double pos) {
    frontController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public void setBackPosition(double pos) {
    backController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public double getFrontPosition() {
    return frontEncoder.getPosition();
  }

  public double getBackPosition() {
    return backEncoder.getPosition();
  }

  public void reset() {
    frontEncoder.setPosition(0);
    backEncoder.setPosition(0);
  }

  public void stop() {
    runFrontMotor(0);
    runBackMotor(0);
  }
}
