/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final String kDefaultAuto = "Default";
  // private static final String kCustomAuto = "My Auto";
  // private String m_autoSelected;
  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
  // m_motor.restoreFactoryDefaults();
  // m_pidController = m_motor.getPIDController();
  // m_encoder = m_motor.getEncoder();

  CANSparkMax motorLeftMaster = SmartMotionInit(4);
  CANSparkMax motorLeftSlave = SmartMotionInit(3);
  CANSparkMax motorRightMaster = SmartMotionInit(2);
  CANSparkMax motorRighttSlave = SmartMotionInit(1);

  // CANSparkMax motorLeftMaster = new CANSparkMax(4, MotorType.kBrushless);
  // CANSparkMax motorLeftSlave = new CANSparkMax(3, MotorType.kBrushless);
  // CANSparkMax motorRightMaster = new CANSparkMax(2, MotorType.kBrushless);
  // CANSparkMax motorRighttSlave = new CANSparkMax(1, MotorType.kBrushless);

  CANSparkMax motorShootTop = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax motorShootBottom = new CANSparkMax(6, MotorType.kBrushless);

  WPI_TalonSRX motorInfeedIn = new WPI_TalonSRX(1);
  WPI_TalonSRX motorInfeedCross = new WPI_TalonSRX(6);
  WPI_TalonSRX motorInfeedWinch = new WPI_TalonSRX(9);

  WPI_TalonSRX motorMagazine = new WPI_TalonSRX(2);
  WPI_TalonSRX motorColorWheel = new WPI_TalonSRX(5);
  WPI_TalonSRX motorClimb = new WPI_TalonSRX(3);

  SpeedControllerGroup rightDrive = new SpeedControllerGroup(motorRightMaster, motorRighttSlave);
  SpeedControllerGroup leftDrive = new SpeedControllerGroup(motorLeftMaster, motorLeftSlave);
  DifferentialDrive drvMode = new DifferentialDrive(leftDrive, rightDrive);

  XboxController xBoxCtrlr = new XboxController(0);
  Joystick buttonBoard = new Joystick(1);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private static final int deviceID = 63;
  private CANSparkMax m_motor;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  Spark BlinkinMono = new Spark(0);
  Spark BlinkinMulti = new Spark(1);

  // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable table = NetworkTableInstance.getDefault().getTable("10.42.95.11");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
    CameraServer.getInstance().startAutomaticCapture();

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  }

  public CANSparkMax SmartMotionInit(int deviceID) {
    CANSparkMax motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    CANPIDController pid = motor.getPIDController();
    // encoder = motor.getEncoder();

    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    return motor;

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    final Color detectedColor = m_colorSensor.getColor();

    String colorString;
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    m_colorMatcher.setConfidenceThreshold(.97);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
      BlinkinMulti.set(0.87);
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      BlinkinMulti.set(0.61);
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      BlinkinMulti.set(0.77);
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      BlinkinMulti.set(0.69);
    } else {
      colorString = "Unknown";
      BlinkinMulti.set(-0.05);
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    final double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    final double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    final double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);

    // post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // read PID coefficients from SmartDashboard
    final double p = SmartDashboard.getNumber("P Gain", 0);
    final double i = SmartDashboard.getNumber("I Gain", 0);
    final double d = SmartDashboard.getNumber("D Gain", 0);
    final double iz = SmartDashboard.getNumber("I Zone", 0);
    final double ff = SmartDashboard.getNumber("Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Max Output", 0);
    final double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the SetReference()
     * method.
     * 
     * The first parameter is the value of the set point, whose units vary depending
     * on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four
     * parameters: com.revrobotics.ControlType.kDutyCycle
     * com.revrobotics.ControlType.kPosition com.revrobotics.ControlType.kVelocity
     * com.revrobotics.ControlType.kVoltage
     */
    final double setPoint = xBoxCtrlr.getY(Hand.kRight) * maxRPM;
    m_pidController.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = m_chooser.getSelected();
    // // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    // System.out.println("Auto selected: " + m_autoSelected);
  }

  boolean autoComplete;

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    System.out.println("Robot.autonomous()");
    drvMode.setSafetyEnabled(false);

    if (!autoComplete) {
      drvMode.tankDrive(0.5, 0.5); // drive forwards half speed
      Timer.delay(2.0); // for 2 seconds.
      drvMode.tankDrive(0.0, 0.0); // stop motors.
      autoComplete = true;
      System.out.println("Auto Complete");
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // -------------------------------------------- set speed using triggers
    double maxSpd = .5;
    if (xBoxCtrlr.getTriggerAxis(Hand.kLeft) > .5 && xBoxCtrlr.getTriggerAxis(Hand.kRight) < .5) {
      maxSpd = .75;
    }

    if (xBoxCtrlr.getTriggerAxis(Hand.kLeft) > .5 && xBoxCtrlr.getTriggerAxis(Hand.kRight) > .5) {
      maxSpd = .9;
    }
    DriveManual(maxSpd, .08);
    // -------------------------------------------- end set speed using triggers

    InfeedManual();
    ClimbUpDn();
    emptyMagRapidFire();
    colorWheel();
    motorInfeedWinchUpDn();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void DriveManual(final double maximum, final double deadBand) {
    drvMode.setDeadband(deadBand);
    drvMode.setSafetyEnabled(false);

    final double leftSpeed = xBoxCtrlr.getY(Hand.kLeft) * -maximum;
    final double rightSpeed = xBoxCtrlr.getY(Hand.kRight) * -maximum;
    final boolean squareInputs = true;

    drvMode.tankDrive(leftSpeed, rightSpeed, squareInputs);
  }

  private void DriveAuto(final double rampUp, final double rampDown, final double maximum, final double deadBand) {
    // drvMode.setDeadband(deadBand);
    // drvMode.setSafetyEnabled(false);

    // final double leftSpeed = xBoxCtrlr.getY(Hand.kLeft) * maximum;
    // final double rightSpeed = xBoxCtrlr.getY(Hand.kRight) * maximum;
    // final boolean squareInputs = true;

    // drvMode.tankDrive(leftSpeed, rightSpeed, squareInputs);
  }

  boolean infeedOn;

  private void InfeedManual() {
    // drvMode.setDeadband(deadBand);
    // drvMode.setSafetyEnabled(false);

    // final double leftSpeed = xBoxCtrlr.getY(Hand.kLeft) * maximum;
    // final double rightSpeed = xBoxCtrlr.getY(Hand.kRight) * maximum;
    // final boolean squareInputs = true;

    // drvMode.tankDrive(leftSpeed, rightSpeed, squareInputs);
    // System.out.println("Infeed Status:" + infeedOn);
    // System.out.println("Button (7) Infeed Status:" +
    // buttonBoard.getRawButton(7));

    if (buttonBoard.getRawButtonPressed(7)) {
      infeedOn = !infeedOn;
    }

    System.out.println("Infeed Status:" + infeedOn);

    if (infeedOn) {
      motorInfeedCross.set(.5);
      motorInfeedIn.set(.5);
    } else {
      motorInfeedCross.stopMotor();
      motorInfeedIn.stopMotor();
    }

  }

  private void ClimbUpDn() { // buttons 5 and 9 on the buttonBoard
    if (buttonBoard.getRawButton(5)) { // extend climber claw
      motorClimb.set(-.25);
    } else if (buttonBoard.getRawButton(9)) { // contract climber claw
      motorClimb.set(.25);
    } else {
      motorClimb.stopMotor(); // stop climber motor action
    }
  }

  boolean shootAllBalls = false;
  int countCalled = 0;

  private void emptyMagRapidFire() { // shoot all balls in the mag.
    // BUtton ids - 1->Low, 2->Med, 3->high
    double spdTopWheel = .9;
    double spdLowWHeel = .9;
    int maxTimesCalled = 50;

    if (countCalled > maxTimesCalled) {
      countCalled = 0;
      shootAllBalls = false;
    }

    if (buttonBoard.getRawButton(1)) { // shoot at lower target
      shootAllBalls = true;
      spdLowWHeel = .25;
    }

    if (buttonBoard.getRawButton(2)) { // shoot at med target
      shootAllBalls = true;
      spdLowWHeel = .5;
    }

    if (buttonBoard.getRawButton(3)) { // shoot at high target
      shootAllBalls = true;
    }

    if (shootAllBalls && countCalled == 0) {
      motorShootBottom.set(spdLowWHeel); // ramp up wheels
      motorShootTop.set(-spdTopWheel);
      Timer.delay(0.5); // wait half second

      motorMagazine.set(.25); // start feeding balls
      Timer.delay(2); // wait 2 seconds

      motorMagazine.stopMotor(); // stop feeding bals
      motorShootBottom.stopMotor(); // stop shooter wheels
      motorShootTop.stopMotor();
    }
    countCalled = (shootAllBalls ? countCalled++ : 0);
  }

  private void colorWheel() {
    if (buttonBoard.getRawButton(4)) {
      motorColorWheel.set(.5);
    } else if (buttonBoard.getRawButton(8)) {
      motorColorWheel.set(-.5);
    } else {
      motorColorWheel.stopMotor();
    }
  }

  private void motorInfeedWinchUpDn() {  //-- raise / lower intake arm
    double UpDn = buttonBoard.getY();  //-- joystick puller back or pushed forward
    double WinchSpd = .25;   //-- speed to raise / lower arm

    if (UpDn == 1) {
      motorInfeedWinch.set(WinchSpd);
    } else if (UpDn == -1) {
      motorInfeedWinch.set(-WinchSpd);
    } else {
      motorInfeedWinch.stopMotor();
    }
  }

}
