/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class GamepadClimb extends CommandBase {
  /**
   * Creates a new GamepadClimb.
   * 
   */

  Climber climber;
  public GamepadClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = RobotContainer.climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.gamepadClimb(RobotContainer.opPad.getLeftAnalogY(), RobotContainer.opPad.getRightAnalogY());
    
    //System.out.println("Left Joystick: " + RobotContainer.drivePad.getLeftAnalogY());
    //System.out.println("Right Joystick: " + RobotContainer.drivePad.getRightAnalogY());
    //System.out.println("Left Encoder: " + climber.getLeftPosition());
    //System.out.println("Right Encoder: " + climber.getRightPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
