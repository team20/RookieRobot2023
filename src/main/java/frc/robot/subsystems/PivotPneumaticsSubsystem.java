// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class PivotPneumaticsSubsystem extends SubsystemBase {
    private DoubleSolenoid m_pivot;
    private Value m_state;
    private static PivotPneumaticsSubsystem m_subsystem;

    public PivotPneumaticsSubsystem() {
        // Singleton
        if (m_subsystem != null) {
            try {
            throw new Exception("Pivot subsystem already initalized!");
            } catch (Exception e) {
            e.printStackTrace();
            }
        }else{
            m_subsystem = this;
        }

        m_pivot = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kPivotFwdPort , PneumaticConstants.kPivotRevPort);
        m_state = m_pivot.get();
    }

    public static PivotPneumaticsSubsystem get(){
        return m_subsystem;
    }

    public void setRaise() {
        m_pivot.set(DoubleSolenoid.Value.kForward);
    }

    public void setLower() {
        m_pivot.set(DoubleSolenoid.Value.kReverse);
    }

    public void setToggle() {
        m_pivot.toggle();
    }

    /**
   * Gets the current state of the wrist pneumatic
   * @return the curent double solenoid value
   * @see Value
   */
    public Value getValue(){
        return m_state;
    }
}