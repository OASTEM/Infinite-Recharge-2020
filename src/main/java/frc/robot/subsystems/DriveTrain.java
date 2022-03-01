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
import frc.robot.LogitechGamingPad;
import frc.robot.RobotContainer;
import frc.robot.commands.GamepadDrive;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  CANSparkMax frontLeft; 
  CANSparkMax frontRight;
  public CANSparkMax backLeft;
  public CANSparkMax backRight;

  CANPIDController leftController;
  CANPIDController rightController;

  CANEncoder leftEncoder;
  CANEncoder rightEncoder;

  Timer timer;
  double isForwardTime;
  double isBackwardTime;

  public DriveTrain() {
    timer = new Timer();
    timer.start();

    frontLeft = new CANSparkMax(2, MotorType.kBrushed); //2 4
    backLeft = new CANSparkMax(1, MotorType.kBrushed); //1   3
    frontRight = new CANSparkMax(3, MotorType.kBrushed); //3  1 
    backRight = new CANSparkMax(4, MotorType.kBrushed); //4 2

    //leftEncoder = backLeft.getEncoder();
    //rightEncoder = backRight.getEncoder();

    backLeft.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
    backRight.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);

    leftEncoder = backLeft.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);
    rightEncoder = backRight.getAlternateEncoder(AlternateEncoderType.kQuadrature, 4096);

    frontLeft.follow(backLeft, false);
    frontRight.follow(backRight, false);
    
    frontLeft.setInverted(true);
    frontRight.setInverted(true);
    backLeft.setInverted(false);
    backRight.setInverted(false);
    leftController = backLeft.getPIDController();
    rightController = backRight.getPIDController();

    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);

    reset();

    
    //sets PID gains for position control for the left pid controller
    leftController.setP(0.40, Constants.dPos_Slot);
    leftController.setI(Constants.dPos_kI, Constants.dPos_Slot);
    leftController.setD(Constants.dPos_kD, Constants.dPos_Slot);
    leftController.setFF(Constants.dPos_kF, Constants.dPos_Slot);
    
    //sets PID gains for position control for the right pid controller
    rightController.setP(0.25, Constants.dPos_Slot);
    rightController.setI(Constants.dPos_kI, Constants.dPos_Slot);
    rightController.setD(Constants.dPos_kD, Constants.dPos_Slot);
    rightController.setFF(Constants.dPos_kF, Constants.dPos_Slot);

    backLeft.setClosedLoopRampRate(Constants.dClosedLoop_Ramp);
    backRight.setClosedLoopRampRate(Constants.dClosedLoop_Ramp);
    
    leftController.setOutputRange(Constants.dMinOutput, Constants.dMaxOutput, Constants.dPos_Slot);
    rightController.setOutputRange(Constants.dMinOutput, Constants.dMaxOutput, Constants.dPos_Slot);
  
    /*
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
    //System.out.println("Left: " + leftEncoder.getPosition());
    //System.out.println("Right: " + rightEncoder.getPosition());
    /*System.out.println("Back Left: " + backLeft.getOutputCurrent());
    System.out.println("Back Right: " + backRight.getOutputCurrent());
    System.out.println("Front Left: " + frontLeft.getOutputCurrent());
    System.out.println("Front Right: " + frontRight.getOutputCurrent());
    */
    //System.out.println("Encoder:" +  backRight.getEncoder().getPosition());
    //System.out.println("forward: " + isForwardTime);
    //System.out.println("reverse: " + isBackwardTime);
  }

  public void drivePercentOutput(double left, double right) {
    if (Math.abs(RobotContainer.navX.getLinearAccelY()) < .4) {
    if(Math.abs(left) >= 0.1) {
      backLeft.set(left);
    }
    else {
      backLeft.set(0);
    }

    if(Math.abs(right) >= 0.1) {
      backRight.set(-right);
    }
    else {
      backRight.set(0.0);
    }
  }
    else {
      stop();
    }
  }

  public void drivePercentOutput(double left, double right, double turnPower) {
    if(Math.abs(left) >= 0.1) {
      if(left > 0 && right < 0) {
        backLeft.set(0.35);
        backRight.set(0.35);
      }
      else if(right > 0 && left < 0) {
        backLeft.set(-0.35);
        backRight.set(-0.35);
      }
      else {
        backLeft.set(left);
      }
    }
    else {
      backLeft.set(0.0);
    }

    if(Math.abs(right) >= 0.1) {
        if(right > 0 && left < 0) {
          backLeft.set(-0.35);
          backRight.set(-0.35);
        }
        else {
          backRight.set(right);
        }
      }
    else {
      backRight.set(0.0);
    }
  }

  /*public void drivePercent(double left, double right) {
    boolean leftPlus = (left < 0);
    boolean rightPlus = (right < 0);

    if(leftPlus && rightPlus) {
      isForwardTime = timer.getFPGATimestamp();
    }

    boolean leftNeg = (left > 0);
    boolean rightNeg = (right > 0);

    if(leftNeg && rightNeg) {
      isBackwardTime = timer.getFPGATimestamp();
    }

    if(Math.abs(isForwardTime - isBackwardTime) < 0.5) {
      /*backLeft.set(left * 0.5);
      backRight.set(right * -0.5);
      backLeft.set(left);
      backRight.set(-right);
    }
    else {
      if (leftPlus == rightPlus) {
        backLeft.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
        backRight.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
        if(Math.abs(left) >= 0.1) {
          //backLeft.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
          backLeft.set(left);
        }
        else {
          backLeft.set(0);
        }
  
        if(Math.abs(right) >= 0.1) {
          //backRight.setOpenLoopRampRate(Constants.dOpenLoop_Ramp);
          backRight.set(-right);
        }
        else {
          backRight.set(0);
        }
      }
      else {
        backLeft.setOpenLoopRampRate(.15);
        backRight.setOpenLoopRampRate(.15); 
        if(Math.abs(left) >= 0.1) {
          //backLeft.setOpenLoopRampRate(.15);
          backLeft.set(left * .4);
        }
        else {
          backLeft.set(0);
        }
  
        if(Math.abs(right) >= 0.1) { 
          //backRight.setOpenLoopRampRate(.15); 
          backRight.set(-right * .4);
        }
        else {
          backRight.set(0);
        }
      }
    }
  }*/ 

  public void drivePercent(double left, double right) {
    backLeft.set(left);
    backRight.set(-right);
  }
  

  public void drivePosition(double goal) {
    goal /= 18;
    //System.out.println(goal);
    leftController.setReference(-goal, ControlType.kPosition, Constants.dPos_Slot);
    rightController.setReference(goal, ControlType.kPosition, Constants.dPos_Slot);
  }

  public void driveLeftPosition(double goal) {
    goal /= 19;
    leftController.setReference(-goal/2, ControlType.kPosition, Constants.dPos_Slot);
  }

  public void driveRightPosition(double goal) {
    goal /= 19;
    rightController.setReference(goal, ControlType.kPosition, Constants.dPos_Slot);
  }

  public void driveSmartMotion(double goal) {
    leftController.setReference(goal, ControlType.kSmartMotion, Constants.dSmart_Motion_Slot);
    rightController.setReference(goal, ControlType.kSmartMotion, Constants.dSmart_Motion_Slot);
  }
  
  public void stop() {
    drivePercent(0.0, 0.0);
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
  
  public double getLeftPosition() {
    return leftEncoder.getPosition();
  }

  public double getRightPosition() {
    return rightEncoder.getPosition();
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

  public void setOpenRampRate(double rate) {
    backLeft.setOpenLoopRampRate(rate);
    backRight.setOpenLoopRampRate(rate);
  }
}


