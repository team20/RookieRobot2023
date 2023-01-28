// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class CameraSubsystem extends SubsystemBase {
  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    UsbCamera c = CameraServer.startAutomaticCapture();
    ShuffleboardTab tab = Shuffleboard.getTab("Camera view");
    //Configure Camera mode
    c.setFPS(CameraConstants.kFPS);
    c.setResolution(CameraConstants.kWidth, CameraConstants.kHeight);
    c.setVideoMode(new VideoMode(CameraConstants.kCameraFormat, CameraConstants.kWidth, CameraConstants.kHeight, CameraConstants.kFPS));
    UsbCameraInfo[] cameraData = UsbCamera.enumerateUsbCameras();
    //Display camera data on shuffleboard
    if(cameraData.length > 0){
      UsbCameraInfo cam = cameraData[0];
      SmartDashboard.putString("Name", cam.name);
      SmartDashboard.putString("Path", cam.path);
      SmartDashboard.putString("Vendor Id", String.valueOf(cam.vendorId));
      SmartDashboard.putString("Product Id", String.valueOf(cam.productId));
    }
    tab.add(c);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
