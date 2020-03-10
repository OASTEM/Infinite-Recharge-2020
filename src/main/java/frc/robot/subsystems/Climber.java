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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.GamepadClimb;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  CANSparkMax leftMotor;
  CANSparkMax rightMotor;

  CANPIDController leftController;
  CANPIDController rightController;

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  boolean isStart;

  public Climber() {
    leftMotor = new CANSparkMax(5, MotorType.kBrushless);
    rightMotor = new CANSparkMax(6, MotorType.kBrushless);

    leftController = leftMotor.getPIDController();
    rightController = rightMotor.getPIDController();

    leftEncoder = leftMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder(EncoderType.kHallSensor, 42);

    rightEncoder = rightMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder(EncoderType.kHallSensor, 42);

    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    leftController.setP(Constants.cPos_kP, Constants.cPos_Slot); 
    leftController.setI(Constants.cPos_kP, Constants.cPos_Slot); 
    leftController.setD(Constants.cPos_kP, Constants.cPos_Slot); 
    leftController .setFF(Constants.cPos_kP, Constants.cPos_Slot);
    rightController.setP(Constants.cPos_kP, Constants.cPos_Slot); 
    rightController.setI(Constants.cPos_kP, Constants.cPos_Slot); 
    rightController.setD(Constants.cPos_kP, Constants.cPos_Slot); 
    rightController .setFF(Constants.cPos_kP, Constants.cPos_Slot);
    
    leftMotor.setOpenLoopRampRate(0.5);
    rightMotor.setOpenLoopRampRate(0.5);

    isStart = true;

    reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if(leftEncoder.getPosition() > 50) {
      runLeftMotor(0.0);
    }
    
    if(rightEncoder.getPosition() > 50 && rightMotor.get() > 0) {
      runRightMotor(0.0);
    }*/

    //System.out.println("Left Encoder Position: " + leftEncoder.getPosition() + " Left Power: " + leftMotor.get());
    //System.out.println("Right Encoder Position: " + rightEncoder.getPosition() + " Right Power: " + rightMotor.get());
  
    if(getLeftPosition() > 10 && getRightPosition() < -10) {
      isStart = false;
    }
  }

  public void gamepadClimb(double leftInput, double rightInput) {
    if(Math.abs(leftInput) >= 0.25) {
      if(leftEncoder.getPosition() > 250 && leftInput < 0) {
        runLeftMotor(0.0);
      }
      else {
      runLeftMotor(-leftInput);
      }
    }
    else {
      runLeftMotor(0);
    }

    if(Math.abs(rightInput) >= 0.25) {
      if(rightEncoder.getPosition() < -250 && rightInput < 0) {
        runRightMotor(0.0);
      }
      else {
        runRightMotor(rightInput);
      }
    }
    else{
      runRightMotor(0);
    }
  }

  public void testClimb(double leftInput, double rightInput) {
    if(leftInput > 0) {
      runLeftMotor(-leftInput);
    }
    else runLeftMotor(0);

    if(rightInput > 0) {
      runRightMotor(rightInput);
    }
    else runRightMotor(0);
  }

  public void runLeftMotor(double power)
  {
    leftMotor.set(power);
  }

  public void runRightMotor(double power)
  {
    rightMotor.set(power);
  }

  public void setPosition(double pos) {
    leftController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
    rightController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public void setLeftPosition(double pos) {
    leftController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public void setRightPosition(double pos) {
    rightController.setReference(pos, ControlType.kPosition, Constants.cPos_Slot);
  }

  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
  }

  public void reset() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public void stop() {
    runLeftMotor(0);
    runRightMotor(0);
  }
}
