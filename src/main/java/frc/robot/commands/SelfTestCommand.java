/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.SelfTest;

public class SelfTestCommand extends CommandBase {
  /**
   * Creates a new SelfTest.
   */
  private ShuffleboardTab selfTest;
  private NetworkTableEntry frontLeftCurrent;
  private NetworkTableEntry frontRightCurrent;
  private NetworkTableEntry backLeftCurrent;
  private NetworkTableEntry backRightCurrent;

  private SelfTest selfTestObj;


  public SelfTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    addRequirements(RobotContainer.climber);
    addRequirements(RobotContainer.cp_Man);
    addRequirements(RobotContainer.highShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.selfTestCount++;
    try{
      if(Robot.selfTestCount >= 5) {
        selfTest = Shuffleboard.getTab("SelfTest");
        frontLeftCurrent = selfTest.add("FrontLeft Current", 0.0).getEntry();
        frontRightCurrent = selfTest.add("FrontRight Current", 0.0).getEntry();
        backLeftCurrent = selfTest.add("BackLeft Current", 0.0).getEntry();
        backRightCurrent = selfTest.add("BackRight Current", 0.0).getEntry();
      
        selfTestObj = new SelfTest();
      }
    }
    catch(Exception e) {
      end(false);
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //frontLeftCurrent.setDouble(selfTestObj.getOutputCurrent());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.selfTestCount < 5;
  }
}
