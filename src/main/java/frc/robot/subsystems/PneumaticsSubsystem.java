/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticConstants;

public class PneumaticsSubsystem extends SubsystemBase {

    public static class ClawPneumatics extends PneumaticsSubsystem {
        public DoubleSolenoid m_claw;
        Value m_state;

        public ClawPneumatics() {
            m_claw = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kClawFwdPort, PneumaticConstants.kClawRevPort);
            m_state = m_claw.get();
        }

        public void openClaw() {
            m_claw.set(DoubleSolenoid.Value.kForward);
        }

        public void closeClaw() {
            m_claw.set(DoubleSolenoid.Value.kReverse);
        }

        public void toggleClaw() {
            m_claw.toggle();
        }
    }
    public static class PivotPneumatics extends PneumaticsSubsystem {
        public DoubleSolenoid m_pivot;
        Value m_state;

        public PivotPneumatics() {
            m_pivot = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kPivotFwdPort , PneumaticConstants.kPivotRevPort);
            m_state = m_pivot.get();
        }

        public void setRaise() {
            System.out.println("raise claw");
            m_pivot.set(DoubleSolenoid.Value.kForward);
        }
    
        public void setLower() {
            System.out.println("lower claw");
            m_pivot.set(DoubleSolenoid.Value.kReverse);
        }

        public void setToggle() {
            System.out.println("toggle claw");
            m_pivot.toggle();
        }
    }
    public static class BrakePneumatics extends PneumaticsSubsystem {
        public DoubleSolenoid m_brake;
        Value m_state;

        public BrakePneumatics() {
            m_brake = new DoubleSolenoid(PneumaticConstants.kPneumaticHubID, PneumaticsModuleType.REVPH, PneumaticConstants.kBrakeFwdPort, PneumaticConstants.kBrakeRevPort);
            m_state = m_brake.get();
        }

        public void armBrake() {
            m_brake.set(DoubleSolenoid.Value.kOff);
        }
    }   

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}