package org.usfirst.frc.team2228.robot.test;

import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBase;
import org.usfirst.frc.team2228.robot.subsystems.drvbase.SRXDriveBaseCfg;
import org.usfirst.frc.team2228.robot.util.DebugLogger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;




public class SRXDriveBaseTest {

	// objects
	private DebugLogger log;
	private SRXDriveBase driveBase;
	private SRXDriveBaseCfg SRXdriveBaseCfg; 

    private double CAL_kDriveStraightFwdCorrection = SRXDriveBaseCfg.kDriveStraightFwdCorrection;
	private double CAL_kDriveStraightRevCorrection = SRXDriveBaseCfg.kDriveStraightRevCorrection;
	private double CAL_RightDriveCmdLevel = 0;
	private double CAL_LeftDriveCmdLevel = 0;
	private double CAL_Throttle = 0;
    private double CAL_turn = 0;
    
	private boolean isTestBtnActive = false;
	private boolean isMtrTestBtnActive = false;
	private boolean isTestStepFunctionActive = false;
	private boolean isTestMoveForStraightCalActive = false;
	private boolean isTestMethodSelectionActive = false;
	private boolean isConsoleDataEnabled = true;
	private String lastMsgString = " ";

	

    // =====================
    // Constructor
    public SRXDriveBaseTest(SRXDriveBase _driveBase, SRXDriveBaseCfg _SRXdriveBaseCfg, DebugLogger _logger){
		driveBase = _driveBase;
		SRXdriveBaseCfg = _SRXdriveBaseCfg;
		log = _logger;
    }

    public void init(){

		isTestBtnActive = false;
		isMtrTestBtnActive = false;
		isTestStepFunctionActive = false;
		isTestMoveForStraightCalActive = false;
		isTestMethodSelectionActive = false;
		
        // Drivebase setup methods
        SmartDashboard.putBoolean("TstBtn-StepFnc:", false);
		SmartDashboard.putBoolean("TstBtn-DrvStraightCal:", false);
		SmartDashboard.putBoolean("TstBtn-MotorEncoderTest:", false);

		// DriveBase autonomous motion methods
		SmartDashboard.putBoolean("TstBtn-MoveToPos:", false);
		SmartDashboard.putBoolean("TstBtn-RotateToAngle:", false);
		SmartDashboard.putBoolean("TstBtn-ProfileMove:", false);

		SmartDashboard.putBoolean("TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleDataEnabled);

    }

	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		
		if(!isTestMethodSelectionActive){
			isTestMethodSelectionActive = true;
			
			Kp_encoderHeadingPID = SmartDashboard.getNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
			//Ki_encoderHeadingPID = SmartDashboard.getNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
			//Kd_encoderHeadingPID = SmartDashboard.getNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
			
		} else {
			
			if (SmartDashboard.getBoolean("TstBtn-MotorEncoderTest:", true)){
				if(!isMtrTestBtnActive){
					msg("START MOTOR ENCODER TEST=============");
					methodStartTime = Timer.getFPGATimestamp();
					isMtrTestBtnActive = true;	
					driveBase.setRightEncPositionToZero();
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftEncPositionToZero();
					driveBase.setLeftSensorPositionToZero();
						
				} else if(Timer.getFPGATimestamp() - methodStartTime > .2){
				
					CAL_RightDriveCmdLevel = SmartDashboard.getNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel);
					CAL_LeftDriveCmdLevel = SmartDashboard.getNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel);
					
					// checkTestMotorEncoderSetUp()
					checkTestMotorEncoderSetUp();
				}
			} else if(isMtrTestBtnActive) {
				msg("END MOTOR ENCODER TEST=============");
				isTestMethodSelectionActive = false;
				isMtrTestBtnActive = false;
				driveBase.setStopMotors();
			}
			
			if(SmartDashboard.getBoolean("TstBtn-StepFnc:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START TEST STEP FUNCTION=============");
					isTestBtnActive = true;
					if (isLoggingDataEnabled){
						log.fopencsv("/home/lvuser/log/SRXDrvStepFnc-" + loggingDataIncrement);
					}
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftSensorPositionToZero();
					driveBase.setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive)
				if(!testStepFunction(.3, 2, false)){
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					if(isLoggingDataEnabled){
						isLoggingActive = false;
						loggingDataIncrement += 1;;
						log.closecsv();
					}
					SmartDashboard.putBoolean("TstBtn-StepFnc:", false);
					msg("SHUFFLE END TEST STEP FUNCTION=============");
				}	
			}
			
			if(SmartDashboard.getBoolean("TstBtn-DrvStraightCal:", false)){
				
				CAL_kDriveStraightFwdCorrection = SmartDashboard.getNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
				if(!isTestBtnActive){
					msg("SHUFFLE START DRIVE STRAIGHT CAL=============");
					isTestBtnActive = true;
					if(isLoggingDataEnabled){
						log.fopencsv("/home/lvuser/log/SRXDrvDrvStght-" + loggingDataIncrement);
					}
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftSensorPositionToZero();
					driveBase.setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				
				// testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel, boolean isDirectionRev)
				if(!testDriveStraightCalibration(50.0, .3, true)){
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					if(isLoggingDataEnabled){
						isLoggingActive = false;
						loggingDataIncrement += 1;
						log.closecsv();
					}
					SmartDashboard.putBoolean("TstBtn-DrvStraightCal:", false);
					msg("SHUFFLE END DRIVE STRAIGHT CAL============");
				}	
			}
			
			
			
			if(SmartDashboard.getBoolean("TstBtn-RotateToAngle:", false)){
				if(!isTestBtnActive){
					msg("SHUFFLE START ROTATE TO ANGLE=============");
					isTestBtnActive = true;
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftSensorPositionToZero();
					driveBase.setDriveTrainRamp(2);
					Timer.delay(0.2);
				}
				// rotateToAngle(double _rotateToAngle, double _rotatePowerLevel)
				if(!rotateToAngle(-90, .3)) {
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-RotateToAngle:", false);
					msg("SHUFFLE END ROTATE TO ANGLE=============");
				}	
			}
			
			
			if (SmartDashboard.getBoolean("TstBtn-DriveCmdLevel:", false)){
				// SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel)
				SetDriveTrainCmdLevel
				(SmartDashboard.getNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel), SmartDashboard.getNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel)); 
			} else {
				isTestMethodSelectionActive = false;
			}
			
			if (SmartDashboard.getBoolean("TstBtn-MagicMotion:", false)){
				if(!isTestBtnActive){
					msg("START MAGIC MOTION TEST=============");
					isTestBtnActive = true;
				
			  // SRXCalStdTrapezoidMoveMagic(double _SRXRightDistanceIn, double _SRXRightTimeSec, double _SRXLeftDistanceIn, double _SRXLeftTimeSec) 
			  // SRXBaseMoveMagic(double _SRXBaseMoveDistanceIn, double _SRXBaseMoveTimeSec)
			  // SRXRotateMagic(double _RotateAngleDeg, double _SRXRotateTimeSec)
			  //} else if(!SRXCalStdTrapezoidMoveMagic(60, 2.5, 60, 2.5)){
			  } else if(!SRXBaseMoveMagic(60, 2)){
		      //} else if(!SRXRotateMagic(90, 2)){	
					isTestMethodSelectionActive = false;
					isTestBtnActive = false;
					SmartDashboard.putBoolean("TstBtn-MagicMotion:", false);
					msg("END MAGIC MOTION TEST======================");
				}
			}
			
			
		
		}
	}
	//===============================
	// TEST STEP FUNCTION
	//===============================
	// This provides a pulse(low-High-low) and stays in lowpower. Need to stop motors or call constantly to produce a square wave
	public boolean testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive) {
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			cycleCount += 1;
			// initialize and start at low speed
			if (!isTestStepFunctionActive) {
				isTestStepFunctionActive = true;
				methodStartTime = Timer.getFPGATimestamp();
				msg("START TEST STEP FUNCTION ===============================");
				cycleCount = 0;
				stepFunctionSpeed = _stepFunctionPower * SRXDriveBaseCfg.MaxVel_VelNativeUnits;
				// (sec / 20ms) [iterative robot scan time]
				stepFunctionStopCount = (int)(_stepFunctionTimeSec / 0.02);
			} else if(cycleCount > stepFunctionStopCount) {
				msg("TEST STEP FUNCTION AT TIME STOP=======================");
				stepFunctionSpeed = 0;
				
				// Delay for a specified time to have motion stopped
				if(!delay(3)){
					isTestStepFunctionActive = false;
					msg("TEST STEP FUNCTION DONE=======================");
					methodTime = Timer.getFPGATimestamp() - methodStartTime;
					msg("Step Function Time(Sec) = " + methodTime);
				}
			}
			//todo -  this needs to be in drivebase as a set
			rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
			leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
			
			if (_isTestForRightDrive){
				rightMasterMtr.set(ControlMode.Velocity, stepFunctionSpeed);
				leftMasterMtr.set(ControlMode.Velocity, 0);
			} else {
				leftMasterMtr.set(ControlMode.Velocity, stepFunctionSpeed);
				rightMasterMtr.set(ControlMode.Velocity, 0);
			}
				
			// +++++++++++++++++++++++++++++++++++++
			// Display data
			System.out.printf("StepVel:%-8.3f==RightVel:%-8.2f==RightErr:%-8.2f==LeftVel:%-8.2f==LeftErr:%-8.2f%n",
					stepFunctionSpeed,
					driveBase.getRightSensorVelocity(),
					driveBase.getRightCloseLoopError(),
					driveBase.getLeftSensorVelocity(),
					driveBase.getLeftCloseLoopError());
		} 
		return isTestStepFunctionActive;
	}
	
		//===================================
		// TEST - MOTOR-ENCODER SETUP TEST
		//===================================
		public void checkTestMotorEncoderSetUp(){ 
			
			rightEncoderPosition = rightMasterMtr.getSensorCollection().getQuadraturePosition();
			rightSensorPositionRaw = rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
			rightSensorPositionRead = driveBase.getRightSensorPosition();
			
			leftEncoderPosition = leftMasterMtr.getSensorCollection().getQuadraturePosition();
			leftSensorPositionRaw = leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
			leftSensorPositionRead = driveBase.getLeftSensorPosition();
		
			SetDriveTrainCmdLevel(CAL_RightDriveCmdLevel, CAL_LeftDriveCmdLevel);
			
			// +++++++++++++++++++++++++++++++++
			// DATA DISPLAY
			System.out.printf("REnc:%-4.0f =RESen:%-4.0f =RESenRd:%-4.0f =LEnc:%-4.0f =LESen:%-4.0f =LESenRd:%-4.0f%n", 
								rightEncoderPosition, 
								rightSensorPositionRaw, 
								rightSensorPositionRead,
								leftEncoderPosition,
								leftSensorPositionRaw,
								leftSensorPositionRead);
		}
	
	//===================================
	// TEST DRIVE STRAIGHT CALIBRATION
	//===================================
	public boolean testDriveStraightCalibration(double _testDistanceIn, double _pwrLevel, boolean _isDirectionRev){
		
		leftSensorPositionRead = driveBase.getLeftSensorPosition();
		rightSensorPositionRead = driveBase.getRightSensorPosition();
		
		if (!isTestMoveForStraightCalActive){
			msg("START CALIBARTION======================================");
			methodStartTime = Timer.getFPGATimestamp();
			isTestMoveForStraightCalActive = true;
			CAL_kDriveStraightFwdCorrection = SmartDashboard.getNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
			CAL_kDriveStraightRevCorrection = SmartDashboard.getNumber("CAL_kDriveStraightRevCorrection:", CAL_kDriveStraightRevCorrection);
			
			if(_isDirectionRev){
				leftCmdLevel = -_pwrLevel;
				rightCmdLevel = (-_pwrLevel * CAL_kDriveStraightRevCorrection); 
				leftEncoderStopCount = -(_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
			}
			else{
				leftCmdLevel = _pwrLevel;
				rightCmdLevel = (_pwrLevel * CAL_kDriveStraightFwdCorrection); 
				leftEncoderStopCount = (_testDistanceIn / SRXDriveBaseCfg.kLeftInchesPerCount);
			}
			
			System.out.printf("StopCnt:%-8.0f+++LftEnc:%-8.0f +++RgtEnc:%-8.0f+++LftCmd:%-8.4f+++RgtCmd:%-8.4f%n", 
								leftEncoderStopCount, 
								leftSensorStartPositionRead, 
								rightSensorStartPositionRead,
								leftCmdLevel,
								rightCmdLevel);
			
	
		// Test for stopping movement
		} else {
			isCalAtStop = ((_isDirectionRev)? (leftSensorPositionRead <= leftEncoderStopCount) : (leftSensorPositionRead >= leftEncoderStopCount));	
		}
		
		if(isCalAtStop)	{
			msg("CALIBRATION AT STOP ===========================================");
			
			setDriveTrainRamp(0);
			// Apply power level in opposite direction for 1 second to brake
			rightCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			leftCmdLevel = -SRXDriveBaseCfg.kAutoRightMoveStopBrakeValue;
			if (!delay(1)) {
				msg("CALIBRATION END ==================================");
				isTestMoveForStraightCalActive = false;
				isCalAtStop = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Drive Straight Calibration Time(Sec) = " + methodTime);
			}	
		}
		
		calCorrectionFactor = leftSensorPositionRead / rightSensorPositionRead;
		headingDeg = (leftSensorPositionRead - rightSensorPositionRead) / SRXDriveBaseCfg.kTrackWidthIn;
		
		// Output to SRX drive modules
		driveBase.SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++
		// LOGGING AND DISPLAY
		if (isLoggingDataEnabled) {
			if(!isLoggingActive){
				logString = ",leftEncStopCnt, leftPos, RightPos, corFactor, headingDeg";
				log.data(logString);	
			} else {
				logString = String.format(",%8.0f,%8.0f,%8.0f,%8.4f,%8.4f", 
											leftEncoderStopCount, 
											leftSensorPositionRead, 
											rightSensorPositionRead,
											calCorrectionFactor,
											headingDeg);
				log.data(logString);
			}
		}
		//Print on console data
		System.out.printf("StopCnt:%-8.0f===LftEnc:%-8.0f ===RgtEnc:%-8.0f===Correction:%-8.4f==Heading:%-8.2f%n", 
							leftEncoderStopCount, 
							leftSensorPositionRead, 
							rightSensorPositionRead,
							calCorrectionFactor,
							headingDeg);

		return isTestMoveForStraightCalActive;
		
	}	
	private void msg(String _msgString){
		if (isConsoleDataEnabled){
			if (_msgString != lastMsgString){
				System.out.println(_msgString);
				lastMsgString = _msgString;
			}
		}
	}	
	//===================
	// DELAY
	//===================
	// This delay is looked at each scan so delay = seconds + scan(~20ms)
	public boolean delay(double _seconds){
		if (!isDelayActive) {
			isDelayActive = true;
			delayStartTime = Timer.getFPGATimestamp();
		} else if (Timer.getFPGATimestamp() >= (delayStartTime + _seconds)){
			isDelayActive = false;
		}
		return isDelayActive;
	}
}