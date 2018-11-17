package org.usfirst.frc.team2228.robot;



import org.usfirst.frc.team2228.robot.oi.DriveBaseTeleopControl;
import org.usfirst.frc.team2228.oi.DriverIF;
import org.usfirst.frc.team2228.robot.test.SRXDriveBaseTest;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	// define object instances
	private SRXDriveBase driveBase;
	private SRXDriveBaseTest testSRXDriveBase;
	private DriverIF driverIF;
	private DriveBaseTeleopControl driveBaseTelop;
	
	private boolean isConsoleDataEnabled = false;
	private String lastMsgString = " ";

	
	// This function is run when the robot is first started up and should be
	// used for any initialization code.
	@Override
	public void robotInit() {

		// Create object instances
		driverIF = new DriverIF();
		driveBase = new SRXDriveBase();
		driveBaseTelop = new DriveBaseTeleopControl(driverIF, driveBase);
		testSRXDriveBase = new SRXDriveBaseTest;

	}

	
	@Override
	public void autonomousInit() {

		// init drive base
		driveBase.Init();
	}

	
	// This function is called periodically during autonomous
	@Override
	public void autonomousPeriodic() {
		
	}

	@Override
	public void teleopInit() {
		System.out.println("teleopInit() fi!");
		driveBase.Init();
		driveBaseTelop.Init();
		System.out.println("Teleop Init done");
	}
	
	// This function is called periodically during operator control
	@Override
	public void teleopPeriodic() {
		driveBaseTelop.Periodic();
		
	}
	
	
	// This function is called once during test mode
	@Override
	public void testInit() {
		testSRXDriveBase.init();
	}
	
	
	// This function is called periodically during test mode
	@Override
	public void testPeriodic() {
		testSRXDriveBase.testMethodSelection();
	}
	
	private void msg(String _msgString){
		if (isConsoleDataEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}			
}

