package org.usfirst.frc.team2228.robot.oi;

import edu.wpi.first.wpilibj.GenericHID.Hand;

public class DriverIF {
	XboxIF xboxIF;
	// Constructor
	public DriverIF() {
		xboxIF = new XboxIF();
	}
	// driver game controller methods
	public double throttleAxis() {
		return xboxIF.LEFT_JOYSTICK_Y();	
	}
	public double turnAxis() {
		return xboxIF.RIGHT_JOYSTICK_X();
	}
	public double wheelAxis() {
		return xboxIF.RIGHT_JOYSTICK_Y();
	}
	public boolean ShiftSidewaysBtn() {
		return xboxIF.LEFT_TRIGGER() {;
	}
	
}
