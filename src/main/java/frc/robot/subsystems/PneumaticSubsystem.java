/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

 // we also dont know what to do with this subsystem because we dont really need it but we dont want to get rid of it so deal with it (for now)

 package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PneumaticConstants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PneumaticSubsystem extends SubsystemBase {
    
    public static enum Device {
        PIVOT, BRAKE, CLAW
    }

    Device m_device;
    DoubleSolenoid m_solonoid;
    Value m_state;

    public PneumaticSubsystem(Device dev) {
        m_device = dev;
        switch(m_device){
            case PIVOT:
                m_solonoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kPivotFwdPort , PneumaticConstants.kPivotRevPort);
                break;
            case BRAKE:
                m_solonoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kBrakeFwdPort, PneumaticConstants.kBrakeRevPort);
                break;
            case CLAW:
                m_solonoid = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kClawFwdPort, PneumaticConstants.kClawRevPort);
                break;
        }

        m_state = m_solonoid.get();
    }

    public void armBrake() {
        if (ControllerConstants.Button.kCircle){
            m_solonoid.disable();
        } else {
            m_solonoid.enable();
        }

    }
    
    public void stop() {
        //compressor.disable();
    }

    public void setForward() {
        m_solonoid.set(DoubleSolenoid.Value.kForward);
    }

    public void setReverse() {
        m_solonoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void setToggle() {
        m_solonoid.toggle();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}