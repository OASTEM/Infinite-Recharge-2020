/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;

import org.opencv.core.Mat;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Camera extends SubsystemBase {

  Thread m_axisCamThread;

  public Camera() {
    m_axisCamThread = new Thread(() -> {
      AxisCamera axisCamera = CameraServer.getInstance().addAxisCamera("axis-camera.local");
      axisCamera.setResolution(640, 480);

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource cvSource = CameraServer.getInstance().putVideo("Rectangle", 640, 480);

      Mat mat = new Mat();
      while (!Thread.interrupted()) {
        if (cvSink.grabFrame(mat) == 0) {
          cvSource.notifyError(cvSink.getError());
          continue;
        }

        cvSource.putFrame(mat);
      }
    });

    m_axisCamThread.setDaemon(true);
    m_axisCamThread.start();
  }

  @Override
  public void periodic() {
    // Put code here to be run every loop

  }

  // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

  // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

}