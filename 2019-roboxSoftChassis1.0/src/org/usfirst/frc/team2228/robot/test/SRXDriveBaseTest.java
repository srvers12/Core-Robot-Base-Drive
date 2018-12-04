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

    private double Tst_kDriveStraightFwdCorrection = SRXDriveBaseCfg.kDriveStraightFwdCorrection;
	private double Tst_kDriveStraightRevCorrection = SRXDriveBaseCfg.kDriveStraightRevCorrection;
	private double Tst_RightDriveCmdLevel = 0;
	private double Tst_LeftDriveCmdLevel = 0;
	private double CAL_Throttle = 0;
	private double CAL_turn = 0;
	private double methodStartTime = 0;
    
	private boolean isTestBtnActive = false;
	private boolean isMtrTestBtnActive = false;
	private boolean isStepFncTestBtnActive = false;
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
		
        // Drivebase setup method calls
        SmartDashboard.putBoolean("drvB-TstBtn-StepFnc:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-DrvStraightCal:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-MotorEncoderTest:", false);

		// DriveBase autonomous motion method calls
		SmartDashboard.putBoolean("drvB-TstBtn-MoveToPos:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-RotateToAngle:", false);
		SmartDashboard.putBoolean("drvB-TstBtn-ProfileMove:", false);

		SmartDashboard.putBoolean("drvB-TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleDataEnabled)
		
		SmartDashboard.putNumber("drvB-Left Encoder:", leftSensorPositionRead);
		SmartDashboard.putNumber("drvB-Right Encoder:", rightSensorPositionRead);
		
		SmartDashboard.putNumber("drvB-Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
		SmartDashboard.putNumber("drvB-Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
		
		SmartDashboard.putNumber("drvB-kDriveStraightFwdCorrection:", drvBtst-kDriveStraightFwdCorrection);
		SmartDashboard.putNumber("drvB-kDriveStraightRevCorrection:", drvBtst-kDriveStraightRevCorrection);
		
		SmartDashboard.putNumber("drvB-Tst_RightDriveCmdLevel:", Tst_RightDriveCmdLevel);
		SmartDashboard.putNumber("drvB-Tst_LeftDriveCmdLevel:", Tst_LeftDriveCmdLevel);
		
		SmartDashboard.putNumber("drvB-CAL_Throttle:", CAL_Throttle);
		SmartDashboard.putNumber("drvB-CAL_turn:", CAL_turn);
		
		SmartDashboard.putNumber("drvB-Val_encoderHeadingDeg:", encoderHeadingDeg);
		SmartDashboard.putNumber("drvB-Val_encoderPIDCorrection:", encoderPIDCorrection);
		

    }

	
	//==============================
	// TEST METHOD SELECTION
	//==============================
	public void testMethodSelection(){
		
		if(!isTestMethodSelectionActive){
			isTestMethodSelectionActive = true;
			
			
		} else {

			//=====================
			// MOTOR ENCODER TEST
			// ====================
			// Run motors to in open loop to test encoder directions
			if (SmartDashboard.getBoolean("drvB-TstBtn-MotorEncoderTest:", true)){
				if(!isMtrTestBtnActive){
					msg("START MOTOR ENCODER TEST=============");
					methodStartTime = Timer.getFPGATimestamp();
					isMtrTestBtnActive = true;
					
					// clear encoder registers
					driveBase.setRightEncPositionToZero();
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftEncPositionToZero();
					driveBase.setLeftSensorPositionToZero();
					driveBase.setTestEnable(true);

				} else if(Timer.getFPGATimestamp() - methodStartTime > .2){
				
					Tst_RightDriveCmdLevel = SmartDashboard.getNumber("drvB-Tst_RightDriveCmdLevel:", Tst_RightDriveCmdLevel);
					Tst_LeftDriveCmdLevel = SmartDashboard.getNumber("drvB-Tst_LeftDriveCmdLevel:", Tst_LeftDriveCmdLevel);
					
					driveBase.SetDriveTrainCmdLevel(Tst_RightDriveCmdLevel, Tst_LeftDriveCmdLevel);
				}
			} else if(isMtrTestBtnActive) {
				msg("END MOTOR ENCODER TEST=============");
				isTestMethodSelectionActive = false;
				isMtrTestBtnActive = false;
				driveBase.StopMotors();
				driveBase.setTestEnable(false);
			}
			// =========================
			// STEP FUNCTION FOR PID TEST
			//==========================
			if(SmartDashboard.getBoolean("drvB-TstBtn-StepFnc:", false)){
				if(!isStepFncTestBtnActive ){
					msg("SHUFFLE START TEST STEP FUNCTION=============");
					isTestBtnActive = true;
					driveBase.setRightSensorPositionToZero();
					driveBase.setLeftSensorPositionToZero();
					Timer.delay(0.2);
				}
				// testStepFunction(double _stepFunctionPower, double _stepFunctionTimeSec, boolean _isTestForRightDrive)
				if(!testStepFunction(.3, 2, false)){
					isTestMethodSelectionActive = false;
					isStepFncTestBtnActive = false;
					SmartDashboard.putBoolean("drvB-TstBtn-StepFnc:", false);
					msg("SHUFFLE END TEST STEP FUNCTION=============");
				}	
			}
			
			// =====================================
			// DRIVE STRAIGHT CORRECTION FACTOR TEST
			// =====================================
			if(SmartDashboard.getBoolean("drvB-TstBtn-DrvStraightCal:", false)){
				
				Tst_kDriveStraightFwdCorrection = SmartDashboard.getNumber("Tst_kDriveStraightFwdCorrection:", Tst_kDriveStraightFwdCorrection);
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
					SmartDashboard.putBoolean("drvB-TstBtn-DrvStraightCal:", false);
					msg("SHUFFLE END DRIVE STRAIGHT CAL============");
				}	
			}
			
			// ======================
			// SRX MOVE
			// ======================
			if(SmartDashboard.getBoolean(""drvB-TstBtn-MoveToPos:", false)){
			
			}
			
			// =====================
			// SRX ROTATE
			// =====================
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
	public boolean testStepFunction(double _stepFunctionPower, double _stepFnctHighTimeSec, boolean _isTestForRightDrive) {
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			// initialize and step to a speed level
			if (!isTestStepFunctionActive) {
				isTestStepFunctionActive = true;
				methodStartTime = Timer.getFPGATimestamp();
				msg("START TEST STEP FUNCTION ===============================");
				if(_isTestForRightDrive){
					Tst_RightDriveCmdLevel = _stepFunctionPower;
					Tst_LeftDriveCmdLevel = 0;
				} else {
					Tst_RightDriveCmdLevel = 0;
					Tst_LeftDriveCmdLevel = _stepFunctionPower;
				}
				driveBase.SetDriveTrainCmdLevel( _rightCMDLevel, _leftCMDLevel);
				
			} else if(!delay(_stepFnctHighTimeSec)) {
				driveBase.stopMotors();
				msg("TEST STEP FUNCTION DONE=======================");
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("Step Function Time(Sec) = " + methodTime);
				isTestStepFunctionActive = false;
			}
			return isTestStepFunctionActive;
			
		} else {
			return isTestStepFunctionActive;

		}
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