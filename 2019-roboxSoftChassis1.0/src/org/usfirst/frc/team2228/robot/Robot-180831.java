package org.usfirst.frc.team2228.robot;


import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBaseCfg;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.DriveBaseTeleopControl;
import org.usfirst.frc.team2228.robot.driver.DriverIF;
import org.usfirst.frc.team2228.robot.test.Test_SRXDriveBase;
// change


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	private String input = "";
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	// define object instances
	private SRXDriveBase base;
	private TestSRXDriveBase testSRXDriveBase;
	private DriverIF driverIF;
	private TeleopController telopDriver;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		driverIF = new DriverIF();
		base = new SRXDriveBase();
		cube = new CubeManipulator(driverIF);
		telopDriver = new TeleopController(driverIF, base);
		
		// Set up automonous chooser
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
	}

	
	@Override
	public void autonomousInit() {

		// init drive base
		base.setSRXDriveBaseInit();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
	}

	public void teleopInit() {
		System.out.println("teleopInit() fi!");
		base.setSRXDriveBaseInit();
		//telopDriver.teleopInit();
		System.out.println("Teleop Init done");
	}
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		telopDriver.teleopPeriodic();
		
	}
	
	/**
	 * This function is called once during test mode
	 */
	@Override
	public void testInit() {
		base.setSRXDriveBaseInit();
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		base.testMethodSelection();
	}
				
}

