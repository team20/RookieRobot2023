/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 // we also dont know what to do with this subsystem because we dont really need it but we dont want to get rid of it so deal with it (for now)

 package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PneumaticConstants;

public class PneumaticSubsystem extends SubsystemBase {

    public static enum Device {
        PIVOT, BRAKE, GRIPPER
    }

    Device m_device;
    public static DoubleSolenoid m_solenoid;
    Value m_state;
    PS4Controller m_controller = new PS4Controller(0);

    public PneumaticSubsystem(Device dev) {
        m_device = dev;
        switch(m_device){
            case PIVOT:
                m_solenoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kPivotFwdPort , PneumaticConstants.kPivotRevPort);
                break;
            case BRAKE:
                m_solenoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kBrakeFwdPort, PneumaticConstants.kBrakeRevPort);
                break;
            case GRIPPER:
                m_solenoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kClawFwdPort, PneumaticConstants.kClawRevPort);
                break;
        }

        m_state = m_solenoid.get();
    }

    public void armBrake() {
        if (){
            m_solenoid.disable();
        } else {
            m_solenoid.enable();
        }
    }

    public void gripperControl() {
        if (m_controller.getL1ButtonPressed()) {
            m_solenoid.set(DoubleSolenoid.Value.kForward);
        } else if (m_controller.getR1ButtonPressed()) {
            m_solenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void stop() {
        //compressor.disable();
    }



    public void setForward() {
        m_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setReverse() {
        m_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void setToggle() {
        m_solenoid.toggle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}