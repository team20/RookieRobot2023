package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CameraConstants;

public class CameraSystem {
    public static void initCamera(){
        //Stream from default camera
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
}