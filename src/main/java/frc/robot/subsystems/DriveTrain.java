/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.GamepadDrive;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  CANSparkMax frontLeft; 
  CANSparkMax frontRight;
  CANSparkMax backLeft;
  CANSparkMax backRight;

  CANPIDController leftController;
  CANPIDController rightController;

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  public DriveTrain() {
    frontLeft = new CANSparkMax(1, MotorType. kBrushless);
    backLeft = new CANSparkMax(2, MotorType.kBrushless);
    frontRight = new CANSparkMax(3, MotorType.kBrushless);
    backRight = new CANSparkMax(4, MotorType.kBrushless);

    //leftEncoder = frontLeft.getEncoder();
    //rightEncoder = frontRight.getEncoder();

    leftEncoder = frontLeft.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    rightEncoder = frontRight.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);

    backLeft.follow(frontLeft);
    backRight.follow(frontRight);
    
    frontLeft.setInverted(false);
    frontRight.setInverted(false);

    leftController = frontLeft.getPIDController();
    rightController = frontRight.getPIDController();

    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    /*//sets PID gains for position control for the left pid controller
    leftController.setP(Constants.dPos_kP, Constants.dPos_Slot);
    leftController.setI(Constants.dPos_kI, Constants.dPos_Slot);
    leftController.setD(Constants.dPos_kD, Constants.dPos_Slot);
    leftController.setFF(Constants.dPos_kF, Constants.dPos_Slot);
 
    //sets PID gains for position control for the right pid controller
    rightController.setP(Constants.dPos_kP, Constants.dPos_Slot);
    rightController.setI(Constants.dPos_kI, Constants.dPos_Slot);
    rightController.setD(Constants.dPos_kD, Constants.dPos_Slot);
    rightController.setFF(Constants.dPos_kF, Constants.dPos_Slot);

    frontLeft.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
    frontRight.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);

    frontLeft.setClosedLoopRampRate(Constants.dClosedLoop_Ramp);
    frontRight.setClosedLoopRampRate(Constants.dClosedLoop_Ramp);

     //configure necessary smart motion parameters for the left PID controllers
     leftController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.dSmart_Motion_Slot);
     leftController.setSmartMotionMaxAccel(Constants.dSmart_Motion_Min_Accel, Constants.dSmart_Motion_Slot);
     leftController.setSmartMotionMinOutputVelocity(Constants.dSmart_Motion_Min_Velocity, Constants.dSmart_Motion_Slot);
     leftController.setSmartMotionMaxVelocity(Constants.dSmart_motion_Max_Velocity, Constants.dSmart_motion_Max_Velocity);
 
     //configure necessary smart motion parameters for the right PID controllers
     rightController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, Constants.dSmart_Motion_Slot);
     rightController.setSmartMotionMaxAccel(Constants.dSmart_Motion_Min_Accel, Constants.dSmart_Motion_Slot);
     rightController.setSmartMotionMinOutputVelocity(Constants.dSmart_Motion_Min_Velocity, Constants.dSmart_Motion_Slot);
     rightController.setSmartMotionMaxVelocity(Constants.dSmart_motion_Max_Velocity, Constants.dSmart_motion_Max_Velocity);
 
     //set PID gains for smart motion
     leftController.setP(Constants.dSmart_Motion_kP, Constants.dSmart_Motion_Slot);
     leftController.setI(Constants.dSmart_Motion_kI, Constants.dSmart_Motion_Slot);
     leftController.setD(Constants.dSmart_Motion_kD, Constants.dSmart_Motion_Slot);
     leftController.setFF(Constants.dSmart_Motion_KF, Constants.dSmart_Motion_Slot);
 
     rightController.setP(Constants.dSmart_Motion_kP, Constants.dSmart_Motion_Slot);
     rightController.setI(Constants.dSmart_Motion_kI, Constants.dSmart_Motion_Slot);
     rightController.setD(Constants.dSmart_Motion_kD, Constants.dSmart_Motion_Slot);
     rightController.setFF(Constants.dSmart_Motion_KF, Constants.dSmart_Motion_Slot);
 
     leftController.setSmartMotionAllowedClosedLoopError(Constants.dSmart_Motion_Allowed_Error, Constants.dSmart_Motion_Slot);
     rightController.setSmartMotionAllowedClosedLoopError(Constants.dSmart_Motion_Allowed_Error, Constants.dSmart_Motion_Slot);
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivePercentOutput(RobotContainer.gamepad.getLeftAnalogY(), RobotContainer.gamepad.getRightAnalogY());

    System.out.println(leftEncoder.getPosition());
  }

  public void drivePercentOutput(double left, double right) {
    frontLeft.set(left); 
    frontRight.set(right);
  }

  public void drivePosition(double goal) {
    leftController.setReference(goal, ControlType.kPosition, Constants.dPos_Slot);
    rightController.setReference(goal, ControlType.kPosition, Constants.dPos_Slot);
  }

  public void driveSmartMotion(double goal) {
    leftController.setReference(goal, ControlType.kSmartMotion, Constants.dSmart_Motion_Slot);
    rightController.setReference(goal, ControlType.kSmartMotion, Constants.dSmart_Motion_Slot);
  }
  
  public void stop() {
    drivePercentOutput(0.0,0.0);
  }

  public double getFrontLeftCurrent() {
    return frontLeft.getOutputCurrent();
  }

  public double getFrontRightCurrent() {
    return frontRight.getOutputCurrent();
  }

  public double getBackLeftCurrent() {
    return backLeft.getOutputCurrent();
  }

  public double getBackRightCurrent() {
    return backRight.getOutputCurrent();
  }
  
  public void getLeftEncoderCount() {
    //return leftEncoder.get();
  }

  public double getLeftMotorOutput() {
    return frontLeft.get();
  }

  public double getRightMotorOutput() {
    return frontRight.get();
  }
  
  public void reset() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
}


