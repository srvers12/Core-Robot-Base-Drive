package org.usfirst.frc.team2228.robot.subsystems.drvbase;
// 
//  Class SRXBaseDrive
//  RELEASE: 2019 
//  Team 2228
// REVISIONS:
// 181106 - removed test methods
// 181102 - updated header, added SRX motion profile
// 181024 - removed time/velocity move commands in 2018 version
// 181024 - added SRX turn, SRX motion profile, removed timed/velocity motion, renamed motion to move, rotate, turn
// 180831 - timer autonomous methods
// 180327 - added config init command to load config vaules specific to a robot chassis
// 180108 - original

// ===================================
// SET COMMANDS
// ===================================
// public void Init()

// public void setEnableConsoleData(boolean _consoleData)
// public void setMecanumShiftEnable(boolean _mecanumShiftState)

// public void setRightSensorPositionToZero()
// public void setLeftSensorPositionToZero()
// public void setRightEncPositionToZero()
// public void setLeftEncPositionToZero()
// public void setCorrectionSensor(int _CorrectionSensorSelect)

// public void setBrakeMode(boolean _isBrakeEnabled)
// public void setStopMotors()
// public void setDriveBaseRamp(double _SecToMaxPower)

// public void clearSRXDriveBasePrgFlgs()
		
// ===================================
// GET COMMANDS
// ===================================
// public double getRightSensorPosition()
// public double getRightSensorVelocity()
// public double getRightEncoderPosition()
// public double getRightEncoderVelocity()
// public double getRightMstrMtrCurrent()
// public double getRightFollowerMtrCurrent()
// public double getRightCloseLoopError()

// public double getLeftSensorPosition(
// public double getLeftSensorVelocity()
// public double getLeftEncoderPosition()
// public double getLeftEncoderVelocity()
// public double getLeftMstrMtrCurrent()
// public double getLeftFollowerMtrCurrent()
// public double getLeftCloseLoopError()

// public double getBusVoltage()
// public double getDriveStraightCorrection()
// public boolean getIsDriveMoving()


// ===================================
//	TELEOP MOTION COMMANDS
// ===================================
	
// *****
// public void SetDriveTrainCmdLevel(double _rightCMDLevel, 
//									 double _leftCMDLevel)
// *****

// *****
//	public void setThrottleTurn(double _throttleValue, 
//								double _turnValue) 
// *****

// =====================================
//	AUTONOMOUS MOTION COMMANDS
// =====================================

// *******
// public boolean move(double _MoveDistanceIn, 
//					   double _MovePwrlevel)
// *******
// @parm  _MoveDistanceIn - move distance in inches
// @parm  _MovePwrlevel - power level 0 - 1, 
//						  program converts to VelocityNativeUnits as (0 to 1)* maxVelocityNativeUnits

// *******
// public boolean move(double _MoveDistanceIn, 
//					   double _MovePwrLevel,
//					   boolean _MoveSideways)
// *******
//
// ******
// public boolean SRXmove(int    _rightCruiseVel, 
//					   int    _rightAccel, 
//					   double _rightDistance, 
//					   int    _leftCruiseVel,	
//					   int    _leftAccel, 
//					   double _leftDistance)
// ******
//
// *******
// public boolean rotate(double _RotateAngleDeg, 
//						 double _RotatePwrLevel)
// *******


// *******
// public boolean turn(double   _TurnAngleDeg, 
//					   double   _TurnRadiusIn, 
//					   double   _TurnPowerLevel, 
//					   boolean  _isRobotDirectionRev)
// *******



//
// public boolean SRXProfileMove(double[][] ProfileRight, 
//								 double[][] ProfileLeft, 
//								 int totalPointNum)
// ******


// ===================================


//Carrying over the classes from other libraries
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	// The following is for the addition of a Navx and ultrasonic sensors
	// public class SRXDriveBase(AngleIF _angle, DistanceIF _distance)
	// AngleIF robotAngle = _angle;
	// DistanceIF robotDistance = _distance;
	
	
	// DifferentialDrive or tank motors
	private WPI_TalonSRX rightMasterMtr;
	private WPI_TalonSRX rightFollowerMtr;
	private WPI_TalonSRX leftMasterMtr;
	private WPI_TalonSRX leftFollowerMtr;

	
	// ===================================
	// SRX MOTION PROFILE

	// The status of the motion profile executer and buffer inside the Talon.
	// Instead of creating a new one every time we call getMotionProfileStatus,
	//  keep one copy.	
	private MotionProfileStatus SRXProfileStatusRight = new MotionProfileStatus();
	private MotionProfileStatus SRXProfileStatusLeft = new MotionProfileStatus();

	private double[][] pointsRight;
	private double[][] pointsLeft;
	// How many trajectory points do we wait for before firing the motion
	// profile.
	private static final int kMinPointsInTalon = 50;

	private int bufferLoadCnt = 0;
	private int profileNumPoints = 0;
	private int SRXBufferSize = 128;
	private int profileIndexPointer = 0;
	private int profileCountAccumulator = 0;

	// additional cache for holding the active trajectory point	
	private double SRXTrajectoryPosition=0;
	private double SRXTrajectoryVelocity=0; 
	private double SRXTrajectoryHeading=0;
	private double kBaseTrajPeriodMs = 0;
	private boolean isSRXProfileMoveActive = false;

	// ====================================	
	// Variables
	private int cycleCount = 1;
	private int SRXTimeoutValueMs = 10;
	private int correctionSensorType = 1;
	private int stepFunctionStopCount = 0;
	private int autoCmdSequence = 1;
	
	private int loggingDataIncrement = 1;
	
	private int RightAccelNativeUnits = 0;
	private int LeftAccelNativeUnits = 0;
	private double RightDistanceCnts = 0;
	private double LeftDistanceCnts = 0;
	
	private double SRXMotionLeftPos =0;
	private double SRXMotioRightPos =0;
	private double SRXMotionLeftVel = 0;
	private double SRXMotioRightVel = 0;
	
	private double leftEncoderStopCount = 0;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double rotationEncoderStopCount = 0;
	private double driveStraightDirCorrection = 0;
	private double wheelToCenterDistanceIn = 0;
	private double outerDistanceStopCnt = 0;
	private double headingDeg =0;
	private double calCorrectionFactor = 0;
	private double delayStartTime = 0;
	private double methodStartTime = 0;
	private double methodTime = 0;
	private double rightSensorStartPositionRead = 0;
	private double rightSensorPositionRead = 0;
	private double leftSensorStartPositionRead = 0;
	private double leftSensorPositionRead = 0;
	private double Kp_encoderHeadingPID = 0.001;
	private double Ki_encoderHeadingPID = 0;
	private double Kd_encoderHeadingPID = 0;
	
	private double encoderPIDCorrection = 0;
	private double encoderHeadingDeg = 0;
	private double sensorCorrection = 1;
	private double stepFunctionSpeed = 0;
	private double rightEncoderPosition = 0;
	private double rightSensorPositionRaw = 0;
	private double leftEncoderPosition = 0;
	private double leftSensorPositionRaw = 0;
	private double kStdAccelTimeSegment = 3;
	
	private double MinimumRadiusDistanceAddition = 4;
	private double RobotDirectionSign = 0;
		
	//  Program flow switches
	private boolean isConsoleDataEnabled = true;
	private boolean isLoggingDataEnabled = false;
	private boolean islogSRXDriveActive = false;
	
	private boolean isTurnToAngleActive = false;
	private boolean isSRXMoveActive = false;
	private boolean isStdTrapezoidalRotateActive = false;
	private boolean isStdTrapezoidalMoveActive = false;
	private boolean isDriveTrainMoving = false;
	private boolean isSensorCorrectionActive = false;
	private boolean isMecanumShiftEnabled = false;
	
	private String logSRXDriveString = " ";
	private String lastMsgString = " ";
	private String logString = " ";
	
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
	
		
		// Create CAN SRX motor controller objects
		rightMasterMtr = new TalonSRX(RobotMap.RIGHT_MSTR_MTR_CAN_ID);
		rightFollowerMtr = new TalonSRX(RobotMap.RIGHT_FOLLOWER_MTR_CAN_ID);
		leftMasterMtr = new TalonSRX(RobotMap.LEFT_MSTR_MTR_CAN_ID);
		leftFollowerMtr = new TalonSRX(RobotMap.LEFT_FOLLOWER_MTR_CAN_ID);

		// RIGHT MOTORS===========================================
		// =======================================================
		
		// Set min/max output
		rightMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		rightMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
		rightMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		rightMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse motor if necessary
		rightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		rightMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);

		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		rightMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		rightMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		rightMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);

		// Set up stall conditions in SRX for the drive train
		rightMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		rightMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		rightMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		rightMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		rightMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		rightMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		rightMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		rightMasterMtr.setSensorPhase(SRXDriveBaseCfg.isRightEncoderSensorReversed);
		
		// Clear quadrature position
		rightMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		rightMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP RIGHT FOLLOWER =======================
		// Invert SRX output to motors if necessary
		rightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		rightFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
		
		// LEFT MOTORS========================================
		//====================================================
		
		// Set min/max output
		leftMasterMtr.configNominalOutputForward(0.0, SRXTimeoutValueMs);
		leftMasterMtr.configNominalOutputReverse(0.0, SRXTimeoutValueMs);
	    leftMasterMtr.configPeakOutputForward(1, SRXTimeoutValueMs);
		leftMasterMtr.configPeakOutputReverse(-1, SRXTimeoutValueMs);
		
		// Reverse direction if necessary
		leftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		
		// Configure voltage compensation mode and set max voltage to 11 volts
		leftMasterMtr.configVoltageCompSaturation(11.0, SRXTimeoutValueMs);
		
		// tweak the voltage bus measurement filter, default is 32 cells in rolling average (1ms per sample)
		leftMasterMtr.configVoltageMeasurementFilter(32, SRXTimeoutValueMs);
		leftMasterMtr.enableVoltageCompensation(true);
		
		// set output zero (neutral) deadband at 4%
		leftMasterMtr.configNeutralDeadband(0.04, SRXTimeoutValueMs);
		
		// Set up stall conditions in SRX for the drive train
		leftMasterMtr.configPeakCurrentLimit(SRXDriveBaseCfg.kStallCurrentPeakAmps, SRXTimeoutValueMs);
		leftMasterMtr.configPeakCurrentDuration(SRXDriveBaseCfg.kStallTimeMs, SRXTimeoutValueMs);
		leftMasterMtr.configContinuousCurrentLimit(SRXDriveBaseCfg.kStallCurrentContinuousAmps, SRXTimeoutValueMs);
		leftMasterMtr.enableCurrentLimit(true);
		
		// Configure the velocity measurement period and sample window rolling average
		// Sample period in ms from supported sample periods-default 100ms period/64 sample window
		leftMasterMtr.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms,SRXTimeoutValueMs);
		leftMasterMtr.configVelocityMeasurementWindow(SRXDriveBaseCfg.kSRXVelocitySample,SRXTimeoutValueMs);
		
		// Set up encoder input
		leftMasterMtr.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, SRXDriveBaseCfg.kslotIDx, SRXTimeoutValueMs);
		leftMasterMtr.setSensorPhase(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
		
		// Clear quadrature position
		leftMasterMtr.clearStickyFaults(SRXTimeoutValueMs);
		leftMasterMtr.getSensorCollection().setQuadraturePosition(0, SRXTimeoutValueMs);
		
		
		// SET UP LEFT FOLLOWER =======================
		leftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);
		leftFollowerMtr.clearStickyFaults(SRXTimeoutValueMs);
		leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
		
		// zero control output
		rightMasterMtr.setNeutralOutput();
		leftMasterMtr.setNeutralOutput();

		/*
		 * Set Brake-Coast mode to coast
		 */
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);

		
		
		// set timeout to zero to stop waiting for confirmations
		SRXTimeoutValueMs = 0;
	}
	// END OF CONSTRUCTOR
	
	// ======================================================================================
	// =======================================================================================
	// SRXBaseDrive SET/CONFIG METHODS
	// =======================================================================================
	// ======================================================================================
	public void setRightSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		rightMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setLeftSensorPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		leftMasterMtr.setSelectedSensorPosition(0, SRXDriveBaseCfg.kPIDLoopIDx, 25);
	}
	
	public void setRightEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		rightMasterMtr.getSensorCollection().setQuadraturePosition(0, 15);
	}
	
	public void setLeftEncPositionToZero() {
		// SRX API Commands are executed every 10ms
		// Set response back timeout for 15ms to wait up to 15ms for a response back
		leftMasterMtr.getSensorCollection().setQuadraturePosition(0, 25);
	}

	
	public void setCorrectionSensor(int _CorrectionSensorSelect){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		correctionSensorType = _CorrectionSensorSelect;
	}
	
	public void setBrakeMode(boolean _isBrakeEnabled) {
		rightMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		rightFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		leftMasterMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
		leftFollowerMtr.setNeutralMode(_isBrakeEnabled ? NeutralMode.Brake : NeutralMode.Coast);
	}
	
	public void setStopMotors(){
		rightMasterMtr.setNeutralOutput();
		leftMasterMtr.setNeutralOutput();
	}
	
	public void setEnableConsoleData(boolean _consoleData){
		isConsoleDataEnabled = _consoleData;
	}

	public void setMecanumShiftEnable(boolean _mecanumShiftState){
		isMecanumShiftEnabled = _mecanumShiftState;
	}
	public void Init() {
		// Clear SRXDriveBase program control flags
		setInitialStateForSRXDrvBasePrgFlgs();
		
		// Load smart dashboard and shuffle board parameters
		loadSmartDashBoardParmeters();
		
		// Stop motors and clear position counters
		setStopMotors();
		setDriveTrainRamp(0);
		setRightSensorPositionToZero();
		setLeftSensorPositionToZero();
		
		// Load drive train PID values
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			rightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			rightMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			rightMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXTimeoutValueMs);
			rightMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrProportionalGain, SRXTimeoutValueMs);
			rightMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXTimeoutValueMs); 
			rightMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain, SRXTimeoutValueMs);
			rightMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveRightMstrIzone, SRXTimeoutValueMs);
		
			leftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			leftMasterMtr.configAllowableClosedloopError(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kClosedLoopErr, SRXTimeoutValueMs);
			leftMasterMtr.config_kF(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXTimeoutValueMs);
			leftMasterMtr.config_kP(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrProportionalGain, SRXTimeoutValueMs);
			leftMasterMtr.config_kI(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXTimeoutValueMs); 
			leftMasterMtr.config_kD(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain, SRXTimeoutValueMs);
			leftMasterMtr.config_IntegralZone(SRXDriveBaseCfg.kPIDLoopIDx, SRXDriveBaseCfg.kdriveleftMstrIzone, SRXTimeoutValueMs);
		}
		
		// See if console data display is enabled
		isConsoleDataEnabled = SmartDashboard.getBoolean("TstBtn-EnableSRXDriveBaseConsoleDisplay:", isConsoleDataEnabled);
	}
	
	// Clear all program control flags
	public void clearSRXDrvBasePrgFlgs() {
		
		isTurnToAngleActive = false;
		isSRXMoveActive = false;
		isStdTrapezoidalRotateActive = false;
		isStdTrapezoidalMoveActive = false;
		isSensorCorrectionActive = false;
		isMecanumShiftEnabled = false;
		isSRXProfileMoveActive = false;
	}
	
	public void setDriveBaseRamp(double _SecToMaxPower){
		if(SRXDriveBaseCfg.isSRXClosedLoopEnabled){
			rightMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			leftMasterMtr.configClosedloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		} else {
			rightMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
			leftMasterMtr.configOpenloopRamp(_SecToMaxPower, SRXTimeoutValueMs);
		}
	}
	
	// =======================================================================================
	// =======================================================================================
	// SRXBaseDrive GET METHODS
	// =======================================================================================
	//=======================================================================================
	
	// RIGHT MASTER MOTOR ==============
	
	public double getRightSensorPosition(){
		// This value is updated every 20ms
		if(SRXDriveBaseCfg.isRightEncoderSensorReadReversed){
			return -rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else {
			return rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	public double getRightSensorVelocity() {
		return rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getRightEncoderPosition() {
		// This value is updated every 160ms
		return -rightMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getRightEncoderVelocity(){
		// This value is updated every 160ms
		return rightMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	
	public double getRightMstrMtrCurrent() {
		return rightMasterMtr.getOutputCurrent();
	}

	public double getRightFollowerMtrCurrent() {
		return rightFollowerMtr.getOutputCurrent();
	}

	public double getRightCloseLoopError() {
		return rightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	// LEFT MASTER MOTOR ==============================
	
	public double getLeftSensorPosition(){
		// This value is updated every 20ms
		if(SRXDriveBaseCfg.isLeftEncoderSensorReadReversed){
		return -leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		} else {
			return leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx);
		}
	}
	
	
	public double getLeftSensorVelocity() {
		return leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx);
	}
	
	public double getLeftEncoderPosition() {
		// This value is updated every 160ms
		return leftMasterMtr.getSensorCollection().getQuadraturePosition();
	}
	
	public double getLeftEncoderVelocity(){
		// This value is updated every 160ms
		return leftMasterMtr.getSensorCollection().getQuadratureVelocity();
	}
	
	public double getLeftMstrMtrCurrent() {
		return leftMasterMtr.getOutputCurrent();
	}
	
	public double getLeftFollowerMtrCurrent() {
		return leftFollowerMtr.getOutputCurrent();
	}
	
	public double getLeftCloseLoopError() {
		return leftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx);
	}

	public double getBusVoltage() {
		return leftMasterMtr.getBusVoltage();
	}
	
	// use "setCorrectionSensor(int _CorrectionSensorSelect)" or set in variable int correctionSensorType
	public double getDriveStraightCorrection(){
		//0-none, 1-encoder, 2-Distance, 3-IMU
		switch(correctionSensorType){
				case 0:
					sensorCorrection = 0;
					break;
				case 1:
					sensorCorrection = encoderAngleCorrection(isSensorCorrectionActive);
					break;
				case 2:
					// sensorCorrection = robotDistance.getAngleCorrection();
					break;
				case 3:
					// sensorCorrection = robotAngle.getAngleCorrection();
					break;
				default:
					sensorCorrection = 0;
		}	
		return	sensorCorrection;
	}
	
	public boolean getIsDriveMoving() {
		if ((Math.abs(leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1) ||
			(Math.abs(rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx)) > 0.1))
		{
			isDriveTrainMoving = true;
		} else {
			isDriveTrainMoving = false;
		}
		return isDriveTrainMoving;
	}
	
	// ======================================================================================
	// =======================================================================================
	// STATUS-DATA METHODS
	// =======================================================================================
	// ======================================================================================
	
	private void loadSmartDashBoardParmeters() {
	
		
		SmartDashboard.putNumber("Left Encoder:", leftSensorPositionRead);
		SmartDashboard.putNumber("Right Encoder:", rightSensorPositionRead);
		
		SmartDashboard.putNumber("Kp_encoderHeadingPID:", Kp_encoderHeadingPID);
		SmartDashboard.putNumber("Ki_encoderHeadingPID:", Ki_encoderHeadingPID);
		SmartDashboard.putNumber("Kd_encoderHeadingPID:", Kd_encoderHeadingPID);
		
		SmartDashboard.putNumber("CAL_kDriveStraightFwdCorrection:", CAL_kDriveStraightFwdCorrection);
		SmartDashboard.putNumber("CAL_kDriveStraightRevCorrection:", CAL_kDriveStraightRevCorrection);
		
		SmartDashboard.putNumber("CAL_RightDriveCmdLevel:", CAL_RightDriveCmdLevel);
		SmartDashboard.putNumber("CAL_LeftDriveCmdLevel:", CAL_LeftDriveCmdLevel);
		
		SmartDashboard.putNumber("CAL_Throttle:", CAL_Throttle);
		SmartDashboard.putNumber("CAL_turn:", CAL_turn);
		
		SmartDashboard.putNumber("Val_encoderHeadingDeg:", encoderHeadingDeg);
		SmartDashboard.putNumber("Val_encoderPIDCorrection:", encoderPIDCorrection);
	}
	
	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void updateSRXDriveDataDisplay() {

		// Display SRX module values
		SmartDashboard.putNumber("BaseDrive-Right Bus Voltage", rightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Right Output Voltage", rightMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Right Master Current", rightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Right Follower Current", rightFollowerMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive-Left Bus Voltage", leftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("BaseDrive-Left Output Voltage", leftMasterMtr.getMotorOutputVoltage());
		SmartDashboard.putNumber("BaseDrive- Left Master Current", leftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("BaseDrive- Left Follower Current", rightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("BaseDrive-Right Position", rightMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Right Velocity ", rightMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Position", leftMasterMtr.getSelectedSensorPosition(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Left Velocity ", leftMasterMtr.getSelectedSensorVelocity(SRXDriveBaseCfg.kPIDLoopIDx));
		}

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("BaseDrive-Speed Right ClosedLoopErr",	rightMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
			SmartDashboard.putNumber("BaseDrive-Speed Left ClosedLoopErr", leftMasterMtr.getClosedLoopError(SRXDriveBaseCfg.kPIDLoopIDx));
		}

	}

	public void logSRXDriveData(){
		if (!islogSRXDriveActive){
			islogSRXDriveActive = true;
			logSRXDriveString = ",Right Bus Voltage,Right Output Voltage,Right Master Current,Right Encoder Count,Right Follower Current,Left Bus Voltage,Left Output Voltage,Left Master Current,Left Encoder Count,Left Follower Current";
			// Log data
			DebugLogger.data(logSRXDriveString);
		} else {
			logSRXDriveString = String.format(",%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f,%8.2f", 
									rightMasterMtr.getBusVoltage(), 
									rightMasterMtr.getMotorOutputVoltage(),
									rightMasterMtr.getOutputCurrent(),
									rightMasterMtr.getRightSensorVelocity(),
									rightFollowerMtr.getOutputCurrent(),
									leftMasterMtr.getBusVoltage(),
									leftMasterMtr.getMotorOutputVoltage(),
									leftMasterMtr.getOutputCurrent(),
									leftMasterMtr.getLeftSensorVelocity(),
									leftFollowerMtr.getOutputCurrent());
			// Log data
			DebugLogger.data(logSRXDriveString);
		}
	} 
	
	private void msg(String _msgString){
		if (_msgString != lastMsgString){
			System.out.println(_msgString);
			lastMsgString = _msgString;}
		}
		
	// =======================================================================================
	// =======================================================================================
	// TELEOP METHODS
	// =======================================================================================
	// =======================================================================================
	
	
	//
	// NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the motor bus voltage). 
	// Motion command with closed loop reflect speed level => (-1 to 1) * (top motor RPM)
	//
	
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		rightCmdLevel = _rightCMDLevel;
		leftCmdLevel = _leftCMDLevel;
		
		if (isMecanumShiftEnabled) {
			rightFollowerMtr.set(ControlMode.Follower, driveLefttMasterMtr.getDeviceID());
			leftFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
		} else {
			rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
			leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
		}
		
		
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits
			rightMasterMtr.set(ControlMode.Velocity, (rightCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
			leftMasterMtr.set(ControlMode.Velocity, (leftCmdLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits ));
		} else {
			rightMasterMtr.set(ControlMode.PercentOutput,rightCmdLevel);
			leftMasterMtr.set(ControlMode.PercentOutput,leftCmdLevel);
		}
	}

	

	// ======================================
	// SET THROTTLE-TURN
	// ======================================
	//setthrottleturn is both open loop and closed loop control with drive straight correction
	
	public void setThrottleTurn(double _throttleValue, double _turnValue) {
			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			leftCmdLevel = _throttleValue + (_turnValue/2);
			rightCmdLevel = ((_throttleValue* SRXDriveBaseCfg.kDriveStraightFwdCorrection) - (_turnValue/2));
			
			
		// Determine drive straight correction if enabled
		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && Math.abs(_turnValue) < SRXDriveBaseCfg.kSpeedDeadBand){
			
			isSensorCorrectionActive = true;
			driveStraightDirCorrection = getDriveStraightCorrection();
			rightCmdLevel += driveStraightDirCorrection;
		} else{
			isSensorCorrectionActive = false;
			driveStraightDirCorrection = getDriveStraightCorrection();
		}
		
		// Output commands to SRX modules set as [% from (-1 to 1)] x MaxVel_VelNativeUnits for feedback control
		SetDriveTrainCmdLevel(rightCmdLevel, leftCmdLevel);
		
		//++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("LftCmd:%-4.3f=RgtCmd:%-4.3f=LftVel:%-5.2f=RgtVel:%-5.2f=LftCur:%-5.2f=RgtCur:%-5.2f=Hd:%-4.3f=Cor:%-4.3f%n", 
									leftCmdLevel, 
									rightCmdLevel,
									getLeftSensorVelocity(),
									getRightSensorVelocity(),
									getLeftMstrMtrCurrent(),
									getRightMstrMtrCurrent(),
									encoderHeadingDeg,
									driveStraightDirCorrection);
		}
	}

	

	// ======================================================================================
	// =======================================================================================
	// AUTONOMOUS METHODS
	// =======================================================================================
	// ======================================================================================
	
	public boolean move(double _MoveDistanceIn, double _MovePwrlevel) {
		
		if(!isStdTrapezoidalMoveActive){
			msg("START MOTION CALCULATIONS ==================================");
			methodStartTime = Timer.getFPGATimestamp();
			isStdTrapezoidalMoveActive = true;
			LeftDistanceCnts = (int)(_SRXLeftDistanceIn / SRXDriveBaseCfg.kInchesPerCount);
			LeftCruiseVelNativeUnits = (int)(_MovePwrlevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
			LeftMoveTimeSec = (int)(1.5 * (LeftDistanceCnts / LeftCruiseVelNativeUnits)); 
			LeftAccelNativeUnits = (int)(LeftCruiseVelNativeUnits/(LeftMoveTimeSec/kStdAccelTimeSegment));
			
			RightDistanceCnts = LeftDistanceCnts;
			RightCruiseVelNativeUnits = LeftCruiseVelNativeUnits;
			RightAccelNativeUnits = LeftAccelNativeUnits;
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						RightDistanceCnts, 
						RightCruiseVelNativeUnits,
						RightAccelNativeUnits,
						LeftDistanceCnts,
						LeftCruiseVelNativeUnits,
						LeftAccelNativeUnits);
			}
		} else if(!SRXBaseMove(RightCruiseVelNativeUnits, 
							   RightAccelNativeUnits, 
							   RightDistanceCnts, 
							   LeftCruiseVelNativeUnits, 
							   LeftAccelNativeUnits, 
							   LeftDistanceCnts)){
			isStdTrapezoidalMoveActive = false;
			methodTime = Timer.getFPGATimestamp() - methodStartTime;
			msg("Std Trap Move (Sec) = " + methodTime);
			msg("END MOVE================");
		}
			
		return isStdTrapezoidalMoveActive;
	}

	// todo - complete move over load
	public boolean move(double _MoveDistanceIn, double _MovePwrLevel, boolean _MoveSideways){

	}
	public boolean rotate(double _RotateAngleDeg, double _RotatePwrLevel) {
		if(!isStdTrapezoidalRotateActive) {
			msg("START ROTATE CALCULATIONS ==================================");
			isStdTrapezoidalRotateActive = true;
			methodStartTime = Timer.getFPGATimestamp();
			
			// rotationEncoderStopCount = C(=>Pi*D) * (angle as a fraction of C)			                                
			LeftDistanceCnts = (int)(Math.PI * (SRXDriveBaseCfg.kTrackWidthIn) * SRXDriveBaseCfg.kEncoderCountsPerIn * (_RotateAngleDeg / 360));
			LeftCruseVelNativeUnits = (int)(_RotatePwrLevel * SRXDriveBaseCfg.MaxVel_VelNativeUnits);
			LeftRotateTimeSec = (int)(1.5 * (LeftDistanceCnts / LeftCruiseVelNativeUnits)); 
			LeftAccelNativeUnits = (int)(LeftCruiseVelNativeUnits/(LeftRotateTimeSec/kStdAccelTimeSegment));
		
			RightDistanceCnts = -LeftDistanceCnts;
			RightCruiseVelNativeUnits = -LeftCruiseVelNativeUnits;
			RightAccelNativeUnits = -LeftAccelNativeUnits;
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						RightDistanceCnts, 
						RightCruiseVelNativeUnits,
						RightAccelNativeUnits,
						LeftDistanceCnts,
						LeftCruiseVelNativeUnits,
						LeftAccelNativeUnits);
			}
		} else if(!SRXBaseMove(RightCruiseVelNativeUnits, 
							   RightAccelNativeUnits, 
							   RightDistanceCnts, 
							   LeftCruiseVelNativeUnits,	
							   LeftAccelNativeUnits, 
							   LeftDistanceCnts)){
			isStdTrapezoidalRotateActive = false;
			methodTime = Timer.getFPGATimestamp() - methodStartTime;
			msg("SRXRotate (Sec) = " + methodTime);
			msg("END ROTATE MOVE================");
		}
			
		return isStdTrapezoidalRotateActive;
	}
	
	//===================================
	// TURN TO ANGLE
	//===================================

	public boolean turn(double  _TurnAngleDeg, 
						double  _TurnRadiusIn, 
						double  _TurnPowerLevel, 
						boolean _isRobotDirectionRev) {
		
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			methodStartTime = Timer.getFPGATimestamp();
			msg("MAGIC MOVE TURN TO ANGLE ACTIVE===========================");
			
			// Check turn radius add 4 inches if less than 1/2 wheelbase
			wheelToCenterDistanceIn = (SRXDriveBaseCfg.kTrackWidthIn / 2);
			if(_TurnRadiusIn < wheelToCenterDistanceIn){
				_TurnRadiusIn = wheelToCenterDistanceIn + MinimumRadiusDistanceAddition;
			}

			//?if (_isRobotDirectionRev): RobotDirectionSign = 1, RobotDirectionSign = -1;

			// Determine which wheel has to speed up to turn
			// Turn right
			if (_turnAngleDeg > 0) {
				if(_isRobotDirectionRev){
					RightCruiseVelNativeUnits = (-SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel)
													* (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
					RightTurnMoveTimeSec = (int)(1.5 * (RightDistanceCnts / RightCruiseVelNativeUnits));
					RightAccelNativeUnits = (int)(RightCruiseVelNativeUnits/(RightTurnMoveTimeSec/kStdAccelTimeSegment));

					LeftCruiseVelNativeUnits = (-SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel)	
												* (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
					LeftTurnMoveTimeSec = (int)(1.5 * (LeftDistanceCnts / LeftCruiseVelNativeUnits));
					LeftAccelNativeUnits = (int)(LeftCruiseVelNativeUnits/(LeftTurnMoveTimeSec/kStdAccelTimeSegment));
				} else {
					RightCruiseVelNativeUnits = (SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel) 
										* (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
					LeftCruiseVelNativeUnits = (SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel) 
										* (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
				}
				
			// Turn Left
			} else {
				if(_isRobotDirectionRev){
					
					LeftCruiseVelNativeUnits = (-SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel)
										* (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
				} else {
					RightCruiseVelNativeUnits = (SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel) 
										* (1 + (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));					
					LeftCruiseVelNativeUnits = (SRXDriveBaseCfg.MaxVel_VelNativeUnits * _TurnPowerLevel) 
										* (1 - (SRXDriveBaseCfg.kTrackWidthIn / (2 * _TurnRadiusIn)));
				}
				
			}
			
			// Calculations
			
			
			// Convert turn distance in inches to encoder counts(2*PI*Radius)*(turnAngledeg/360deg)*(cnts/in)
			// Turn right
			if (_turnAngleDeg >= 0) {
				if(_isRobotDirectionRev){
					outerDistanceStopCnt = (2 * Math.PI * ((_TurnRadiusIn + wheelToCenterDistanceIn) 
											* (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn);
				} else {
					outerDistanceStopCnt = (2 * Math.PI * ((_TurnRadiusIn + wheelToCenterDistanceIn) 
											* (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn);
				}
			// Turn Left	
			} else {
				if(_isRobotDirectionRev){
					outerDistanceStopCnt = (2 * Math.PI * ((_TurnRadiusIn + wheelToCenterDistanceIn) 
											* (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kLeftEncoderCountsPerIn);
				} else {
					outerDistanceStopCnt = (2 * Math.PI * ((_TurnRadiusIn + wheelToCenterDistanceIn) 
											* (Math.abs(_turnAngleDeg) / 360)) * SRXDriveBaseCfg.kRightEncoderCountsPerIn);
				}
				
			}
			
			if (isConsoleDataEnabled){
				System.out.printf("RgtD:%-8.2f ++RgtV:%-8d ++RgtA:%-8d ++LftD:%-8.2f ++LftV:%-8d ++LftA:%-8d%n", 
						RightDistanceCnts, 
						RightCruiseVelNativeUnits,
						RightAccelNativeUnits,
						LeftDistanceCnts,
						LeftCruiseVelNativeUnits,
						LeftAccelNativeUnits);
			}
		// Active state -  check for end of encoder count
		} else if(!SRXMove(RightCruiseVelNativeUnits, 
								RightAccelNativeUnits, 
								RightDistanceCnts, 
								LeftCruiseVelNativeUnits,	
								LeftAccelNativeUnits, 
								LeftDistanceCnts)) { 
			
					
					if (!delay(1)) {
						isTurnToAngleActive = false;
						RightCruiseVel = 0;
						LeftCruiseVel = 0;
						methodTime = Timer.getFPGATimestamp() - methodStartTime;
						msg("Motion magic Turn to angle Time(Sec) = " + methodTime);
						msg("MOTION MAGIC TURN TO ANGLE DONE===========================");
					}
				}
				
		
		
		//! not right: SetDriveTrainCmdLevel(mgmvRightCruiseVel, mgmvLeftCruiseVel);
		
		// +++++++++++++++++++++++++++++++++++++++
		// Display data
		if (isConsoleDataEnabled){
			System.out.printf("RgtCmd: %-4.3f ===LftCmd: %-4.3f ===StopCnt: %-8.0f ===LftPos: %-8.2f ===RgtPos: %-8.2f%n",
									RightCruiseVel,
									LeftCruiseVel,
									outerDistanceStopCnt,
									leftSensorPositionRead, 
									rightSensorPositionRead);
		}
		return isTurnToAngleActive;
	}
	
	// This method performs a SRX magic motion command from user calculated values
	// All SRXBaseMove parms are in native units 
	
	public boolean SRXMove(int    _rightCruiseVel, 
						   int    _rightAccel, 
						   double _rightDistance, 
						   int    _leftCruiseVel,	
						   int    _leftAccel, 
						   double _leftDistance) {
		
		leftSensorPositionRead = getLeftSensorPosition();
		rightSensorPositionRead = getRightSensorPosition();
		
		if (!isSRXMoveActive) {
			isSRXMoveActive = true;
			msg("START SRX MOTION ==================================");
			methodStartTime = Timer.getFPGATimestamp();
			/* Set relevant frame periods to be at least as fast as periodic rate*/
			rightMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, SRXTimeoutValueMs);
			rightMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);
			rightMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			rightMasterMtr.configMotionCruiseVelocity(_rightCruiseVel, SRXTimeoutValueMs);
			rightMasterMtr.configMotionAcceleration(_rightAccel, SRXTimeoutValueMs);

			// todo -  this does not match rgt mstr
			leftMasterMtr.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, SRXTimeoutValueMs);
			leftMasterMtr.selectProfileSlot(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx);	
			leftMasterMtr.setSelectedSensorPosition(SRXDriveBaseCfg.kslotIDx, SRXDriveBaseCfg.kPIDLoopIDx, SRXTimeoutValueMs);
			leftMasterMtr.configMotionCruiseVelocity(_leftCruiseVel, SRXTimeoutValueMs);
			leftMasterMtr.configMotionAcceleration(_leftAccel, SRXTimeoutValueMs);

			
		} else {
			SRXMotionLeftPos = leftMasterMtr.getActiveTrajectoryPosition();
			SRXMotionLeftVel = leftMasterMtr.getActiveTrajectoryVelocity();
			SRXMotioRightPos = rightMasterMtr.getActiveTrajectoryPosition();
			SRXMotioRightVel = rightMasterMtr.getActiveTrajectoryVelocity();
			
			if ((Math.abs(SRXMotionLeftPos) >= Math.abs(_leftDistance)) && (Math.abs(SRXMotioRightPos) >= Math.abs(_rightDistance))) {
				rightMasterMtr.set(ControlMode.MotionMagic, 0); 
				leftMasterMtr.set(ControlMode.MotionMagic, 0);
				isSRXMoveActive = false;
				methodTime = Timer.getFPGATimestamp() - methodStartTime;
				msg("SRX Motion (Sec) = " + methodTime);
				msg("END SRX MOTION  ========================");
			}
		}
		// Output commands to SRX's
		rightFollowerMtr.set(ControlMode.Follower, rightMasterMtr.getDeviceID());
		leftFollowerMtr.set(ControlMode.Follower, leftMasterMtr.getDeviceID());
		
		rightMasterMtr.set(ControlMode.MotionMagic, _rightDistance); 
		leftMasterMtr.set(ControlMode.MotionMagic, _leftDistance);
		
		if (isConsoleDataEnabled){
			System.out.printf("LftEnc:%-8.0f ==RgtEnc:%-8.0f ==LftSRXVel:%-8.2f ==LftSRXPos:%-8.2f ==RgtSRXVel:%-8.2f ==RgtSRXPos:%-8.2f %n", 
								leftSensorPositionRead, 
								rightSensorPositionRead,
								SRXMotionLeftVel,
								SRXMotionLeftPos,
								SRXMotioRightVel,
								SRXMotioRightPos);
		}
		return isSRXMoveActive;
	}
	
	/**
	
	 * The only routines we call on Talon are....
	 * 
	 * changeMotionControlFramePeriod
	 * 
	 * getMotionProfileStatus		
	 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
	 * 
	 * pushMotionProfileTrajectory
	 * clearMotionProfileTrajectories
	 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
	 * 
	 * getControlMode, to check if we are in Motion Profile Control mode.
	 * 
	 * Example of advanced features not demonstrated here...
	 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
	 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
	 */

	// This code is leveraged from CTRE electronics, Team317, and Team3539
	public boolean SRXProfileMove(double[][] ProfileRight, double[][] ProfileLeft, int totalPointNum) {
		pointsRight = ProfileRight;
		pointsLeft = ProfileLeft;
		profileNumPoints = totalPointNum;

		if(!isSRXProfileMoveActive) {

			// Disable motion profile while setting up a profile move
			rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);
			leftMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable);

			rightMasterMtr.clearMotionProfileHasUnderrun(0);
			leftMasterMtr.clearMotionProfileHasUnderrun(0);

			// todo - set sensor posistion to zero

			// Let's clear the buffer just in case user decided to disable in the
			// middle of an MP, and now we have the second half of a profile just
			// sitting in memory.
			
			rightMasterMtr.clearMotionProfileTrajectories();
			leftMasterMtr.clearMotionProfileTrajectories();

			// create an empty point			
			TrajectoryPoint trajectoryPointRight = new TrajectoryPoint();
			TrajectoryPoint trajectoryPointLeft = new TrajectoryPoint();

			rightMasterMtr.configMotionProfileTrajectoryPeriod(0, 10);
			leftMasterMtr.configMotionProfileTrajectoryPeriod(0, 10);

			// set the base trajectory period to zero, use the individual trajectory period below
			rightMasterMtr.configMotionProfileTrajectoryPeriod(SRXDriveBaseCfg.kBaseTrajPeriodMs, SRXTimeoutValueMs);
			leftMasterMtr.configMotionProfileTrajectoryPeriod(SRXDriveBaseCfg.kBaseTrajPeriodMs, SRXTimeoutValueMs);

			// set profile slot point.profileSlotSelect0 = 0;
			
			//  Lets create a periodic task to funnel our trajectory points into our talon.
			//  It doesn't need to be very accurate, just needs to keep pace with the motion
			//  profiler executer.  Now if you're trajectory points are slow, there is no need
			//  to do this, just call rightMasterMtr.processMotionProfileBuffer() in your teleop loop.
			//  Generally speaking you want to call it at least twice as fast as the duration
			//  of your trajectory points.  So if they are firing every 20ms, you should call 
			//  every 10ms.
		
			// class PeriodicRunnable implements java.lang.Runnable {
			// 	public void run() {  
			// 		rightMasterMtr.processMotionProfileBuffer();
			// 		leftMasterMtr.processMotionProfileBuffer();
			// 	}
			// }
			// Notifier _notifer = new Notifier(new PeriodicRunnable());
		 	// rightMasterMtr.changeMotionControlFramePeriod(5);
		 	// _notifer.startPeriodic(0.005);

			/* When we do start running our state machine start at the beginning. */
			SRXProfileState = 0;
		
			// Any time you have a state machine that waits for external events, its a
			// good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
			// down to '0' which will print an error message. Counting loops is not a
			// very accurate method of tracking timeout, but this is just conservative
			// timeout. Getting time-stamps would certainly work too, this is just
			// simple (no need to worry about timer overflows).
			private int SRXProfileLoopTimeout = -1;

			// Just a state timeout to make sure we don't get stuck anywhere. Each loop is about 20ms.
			private static final int kNumLoopsTimeout = 10;
			SRXProfileLoopTimeout = -1;

			rightMasterMtr.changeMotionControlFramePeriod(5);
			driveLefttMasterMtr.changeMotionControlFramePeriod(5);

			// Additional cache for holding the active trajectory point
			SRXTrajectoryPosition = 0; 
			SRXTrajectoryVelocity = 0; 
			SRXTrajectoryHeading = 0;

		} else {
			
			//Get the motion profile status every loop
			rightMasterMtr.getMotionProfileStatus(SRXProfileStatusRight);
			driveLefttMasterMtr.getMotionProfileStatus(SRXProfileStatusLeft);

			//When we do re-enter motionProfile control mode, stay disabled.
			//_setValue = SetValueMotionProfile.Disable;
			
			//track time, this is rudimentary but that's okay, we just want to make
			//sure things never get stuck.
			if (SRXProfileLoopTimeout < 0) {
				//do nothing, timeout is disabled
			} else {
				// our timeout is nonzero
				if (SRXProfileLoopTimeout == 0) {
					
					// something is wrong. Talon is not present, unplugged, breaker tripped
					// todo error to take care of
					Instrumentation.OnNoProgress();
				} else {
					--SRXProfileLoopTimeout;
				}
			}

			// First check if we are in MP mode
			if (rightMasterMtr.getControlMode() != ControlMode.MotionProfile) {
				
				// we are not in MP mode. We are probably driving the robot around
				// using gamepads or some other mode.
				//SRXProfileState = 0;
				//SRXProfileLoopTimeout = -1;
			} else {	
				// we are in MP control mode. That means: starting Mps, checking Mp
				// progress, and possibly interrupting MPs if thats what you want to do.
				switch (SRXProfileState) {
					case 0: 
						// First load of the profile buffer
						bufferLoadCnt = (profileNumPoints < SRXBufferSize)?  profileNumPoints: 100;
						profileIndexPointer = 0;
						fillProfileBuffer(profileIndexPointer, bufferLoadCnt);
						
						// MP is being sent to CAN bus, wait a small amount of time
						SRXProfileState = 1;
						SRXProfileLoopTimeout = kNumLoopsTimeout;
						profileIndexPointer = 100;
						profileCountAccumulator = 100;
						break;
					case 1: 
						// wait for MP to stream to Talon, really just the first few points
						// do we have a minimum numberof points in Talon
						if (SRXProfileStatusRight.btmBufferCnt > kMinPointsInTalon) {

							// start the motion profile after first load of SRX buffer 
							rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable);
							driveLefttMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Enable);
							
							// State 2 continues loading SRX profile buffer until all Trajectory points are loaded
							SRXProfileState = 2;
							SRXProfileLoopTimeout = kNumLoopsTimeout;
						}
						break;
					case 2: 
						// check the status of the MP
						// if talon is reporting things are good, keep adding to our
						// timeout. Really this is so that you can unplug your talon in
						// the middle of an MP and react to it.
						// did we get an underrun condition since last time we checked ?
						bufferLoadCnt = (profileNumPoints < SRXBufferSize)?  profileNumPoints: 100;
						profileIndexPointer = 101;
						fillProfileBuffer(profileIndexPointer, bufferLoadCnt);
						profileIndexPointer = 100;
						profileCountAccumulator = 100;
						// todo - is this needed
						if (SRXProfileStatusRight.hasUnderrun) { 
							/* better log it so we know about it */
							Instrumentation.OnUnderrun();
							
							// clear the error. This flag does not auto clear, this way 
							// we never miss logging it.
							
							rightMasterMtr.clearMotionProfileHasUnderrun(0);
						}
						if (SRXProfileStatusRight.isUnderrun == false) {
							SRXProfileLoopTimeout = kNumLoopsTimeout;
						}
						
						// If we are executing an MP and the MP finished, start loading
						// another. We will go into hold state so robot servo's position.
						if (SRXProfileStatusRight.activePointValid && SRXProfileStatusRight.isLast) {
							
							// because we set the last point's isLast to true, we will
							// get here when the MP is done
							rightMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold);
							driveLefttMasterMtr.set(ControlMode.MotionProfile, SetValueMotionProfile.Hold);

							isSRXProfileMoveActive = false;

							SRXProfileState = 0;
							SRXProfileLoopTimeout = -1;
						}
					break;
					// todo -  we need to say we are done
					// isSRXProfileMoveActive = false;
				}

				// Get the motion profile status every loop
				rightMasterMtr.getMotionProfileStatus(SRXProfileStatusRight);
				leftMasterMtr.getMotionProfileStatus(SRXProfileStatusLeft);

				SRXTrajectoryHeading = rightMasterMtr.getActiveTrajectoryHeading();
				SRXTrajectoryPosition = rightMasterMtr.getActiveTrajectoryPosition();
				SRXTrajectoryVelocity = rightMasterMtr.getActiveTrajectoryVelocity();

				// todo 
				// printfs and/or logging
				//Instrumentation.process(SRXProfileStatusRight, SRXTrajectoryPosition, SRXTrajectoryVelocity, SRXTrajectoryHeading);
			}
			
		
		}
		return isSRXProfileMoveActive;
	}
		/**
		 * Find enum value if supported.
		 * @param durationMs
		 * @return enum equivalent of durationMs
		 */
		private TrajectoryDuration GetTrajectoryDuration(int durationMs)
		{	 
			/* create return value */
			TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
			/* convert duration to supported type */
			retval = retval.valueOf(durationMs);
			/* check that it is valid */
			if (retval.value != durationMs) {
				DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
			}
			/* pass to caller */
			return retval;
		}
		// /** Start filling the MPs to all of the involved Talons. */
		// private void startFilling() {
		// 	/* since this example only has one talon, just update that one */
		// 	startFilling(GeneratedMotionProfile.Points, GeneratedMotionProfile.kNumPoints);
		// }

		private void fillProfileBuffer(int _profileIndexPtr, int _profileLoadCnt) {

			// for each point, fill trajectory point structure and pass it to API to load Top buffer
			// This will then be loaded into the SRX buffer via API
			for (int i = _profileIndexPtr; i < _profileLoadCnt; ++i) {
				trajectoryPointRight.position = pointsRight[i][0];
				trajectoryPointLeft.position = pointsRight[i][0];

				trajectoryPointRight.velocity = pointsRight[i][1];
				trajectoryPointLeftt.velocity = pointsRight[i][1];

				trajectoryPointRight.headingDeg = 0; // future feature - not used in this example
				trajectoryPointLeft.headingDeg = 0; // future feature - not used in this example

				trajectoryPointRight.profileSlotSelect0 = 0; // which set of gains would you like to use [0,3]?
				trajectoryPointLeft.profileSlotSelect0 = 0; // which set of gains would you like to use [0,3]?

				trajectoryPointRight.profileSlotSelect1 = 0; // future feature  - not used in this example - cascaded PID [0,1], leave zero
				trajectoryPointLeft.profileSlotSelect1 = 0; // future feature  - not used in this example - cascaded PID [0,1], leave zero

				trajectoryPointRight.timeDur = pointsRight[i][2];
				trajectoryPointLeft.timeDur = pointsRight[i][2];

				if ((SRXProfileState == 0) && (i == 0)){
					trajectoryPointRight.zeroPos = true; // set this to true on the first point
					trajectoryPointLeft.zeroPos = true; // set this to true on the first point
				} else {
					trajectoryPointRight.zeroPos = false;
					trajectoryPointLeft.zeroPos = false;
				}
				if ((SRXProfileState == 4) && (i == _bufferLoadCnt)){
					trajectoryPointRight.isLastPoint = true; // set this to true on the last point
					trajectoryPointLeft.isLastPoint = true; // set this to true on the last point
				} else {
					trajectoryPointRight.isLastPoint = false;
					trajectoryPointLeft.isLastPoint = false;
				}
				rightMasterMtr.pushMotionProfileTrajectory(trajectoryPointRight);
				leftMasterMtr.pushMotionProfileTrajectory(trajectoryPointLeft);
			}
		}
	
}