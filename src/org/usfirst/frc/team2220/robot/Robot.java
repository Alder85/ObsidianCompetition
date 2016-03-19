
package org.usfirst.frc.team2220.robot;


//import edu.wpi.first.wpilibj.*;
import org.usfirst.frc.team2220.robot.XBoxController.*;
import org.usfirst.frc.team2220.robot.XBoxController;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CANTalon.*;
import edu.wpi.first.wpilibj.interfaces.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.io.*;
import java.util.ArrayList;

/**
 * Main robot class.<br>
 * Defines everything and puts it to use somewhat efficiently.
 * @author Josh
 *
 */
public class Robot extends SampleRobot {
    SmartDashboard dashboard;

    public Robot() {
        
    }    
    
    TwilightTalon talon2 = new TwilightTalon(2);
	ModuleRotation flModule = new ModuleRotation(talon2);
	
	TwilightTalon talon4 = new TwilightTalon(4);
	ModuleRotation blModule = new ModuleRotation(talon4);
	
	TwilightTalon talon5 = new TwilightTalon(5);
	ModuleRotation brModule = new ModuleRotation(talon5);
	
    TwilightTalon talon7 = new TwilightTalon(7);
	ModuleRotation frModule = new ModuleRotation(talon7);

	TwilightTalon frWheel = new TwilightTalon(8);
	TwilightTalon flWheel = new TwilightTalon(1);
	TwilightTalon blWheel = new TwilightTalon(3);
	TwilightTalon brWheel = new TwilightTalon(6);
	
	TwilightTalon collector = new TwilightTalon(9);
	TwilightTalon rightShooter = new TwilightTalon(10);
	TwilightTalon leftShooter  = new TwilightTalon(12);
	
	
	Gyro gyro = new AnalogGyro(0);
	
	TwilightTalon lifterRelease = new TwilightTalon(13);
	TwilightTalon wench = new TwilightTalon(14);
	
	//POSITIVE SPINS counterclockwise 
	TwilightTalon collectorExtender = new TwilightTalon(11);
	//counterclockwise for backward
	//clockwise for forwards
	
	Drivetrain drivetrain = new Drivetrain();
	
	
	
	//LEFt need to be flipped
	//xbox start 0,0 top left so flip right
	XBoxController driverController = new XBoxController(0);
	XBoxController manipulatorController = new XBoxController(1);
	
	DigitalInput frontCollector = new DigitalInput(0);
	DigitalInput rearCollector  = new DigitalInput(1);
	
	Timer shootTimer = new Timer();
	
	CameraServer server; //.setQuality if necessary
	Image frame;
	boolean isCam1 = true;
	int session0, session1;
	double prevPOVval= 0;
	
	double allTuning;
	ImageProcessor processor;
	Autonomous autonomous;
	SerialCom serial;
	
	boolean overideShooting = false;
	boolean overideHigh = false;

    /**
     * Initializes everything with presets needed for both autonomous and teleop
     */
	public void robotInit()
	{
		serial = new SerialCom();
		
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		session0 = NIVision.IMAQdxOpenCamera("cam3", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		session1 = NIVision.IMAQdxOpenCamera("cam4", NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		NIVision.IMAQdxConfigureGrab(session1);
		processor = new ImageProcessor(session1);
		
		
		collectorExtender.enableBrakeMode(true);
    	
    	frWheel.setMaxCurrent(100);
    	flWheel.setMaxCurrent(100);
    	brWheel.setMaxCurrent(100);
    	blWheel.setMaxCurrent(100);
    	
    	talon2.setMaxCurrent(60);
    	talon4.setMaxCurrent(60);
    	talon5.setMaxCurrent(60);
    	talon7.setMaxCurrent(60);
    	
    	collector.setMaxCurrent(60);
    	rightShooter.setMaxCurrent(70);
    	leftShooter.setMaxCurrent(70);
    	
    	collectorExtender.setMaxCurrent(120);
    	lifterRelease.setMaxCurrent(30);
    	
    	flModule.reverseTalon(false);
    	blModule.reverseTalon(true);
    	
    	frModule.reverseTalon(false);
    	brModule.reverseTalon(true);
    	
    	
    	frModule.setRightWheel(true);
    	brModule.setRightWheel(true);
    	
    	frModule.reverseSensor(false);
    	
    	talon2.enableBrakeMode(false);
    	talon4.enableBrakeMode(false);
    	talon5.enableBrakeMode(false);
    	talon7.enableBrakeMode(false);
    	

    	rightShooter.changeControlMode(TalonControlMode.Voltage);
    	leftShooter.changeControlMode(TalonControlMode.Voltage);
    	leftShooter.reverseOutput(true);
    	
    	autonomous = new Autonomous(drivetrain, gyro, collector, leftShooter, rightShooter, processor, serial, collectorExtender);
    	allTuning = 1.5;
    	
    	frModule.setP(allTuning);
    	brModule.setP(allTuning);
    	flModule.setP(allTuning);
    	blModule.setP(allTuning);
    	
    	drivetrain.setModules(flModule, frModule, brModule, blModule);
    	drivetrain.setWheels(flWheel, frWheel, brWheel, blWheel);
    	
    	lifterRelease.setFeedbackDevice(FeedbackDevice.PulseWidth);
    	lifterRelease.changeControlMode(TalonControlMode.Position);
    	lifterRelease.setPID(1.0, 0.001, 0.0);
    	lifterRelease.reverseOutput(true);
    	lifterRelease.setAllowableClosedLoopErr(30);
    	
    }
	
	boolean autoShooting = false;
	double shotLength = 1.375;
	/**
     * Using the autonomous object, runs a series of function on 
     * the declared drivetrain to complete autonomous goals
     */
    public void autonomous() 
    {
    	
    	if(isEnabled() && isAutonomous())
    	{
	    	frWheel.enableBrakeMode(true);
	    	flWheel.enableBrakeMode(true);
	    	brWheel.enableBrakeMode(true);
	    	blWheel.enableBrakeMode(true);
	    	//lowbar
	    	//TODO change this
	    	
	    	autonomous.extendCollector(0.46);
	    	
	    	autonomous.drive(2.7, 0.6); //prev driveGyro
	    	autonomous.timePointTurn(0.75, 1.0, true);
	    	autonomous.shoot();
	    	//autonomous.uncollect(5.0);
	    	/*
	    	autonomous.timePointTurn(0.75, 1.0, true);
	    	//autonomous.pointTurnGyro(55, 0.6);
	    	
	    	autonomous.lineUpToShoot();
	    	if(autonomous.foundTarget)
	    	{
		    	autonomous.readSerial();
		    	autonomous.shoot();
	    	}
	    	*/
	    	
	    }
	    

    }
	/**
     * Using the previously declared robot objects, caries out the functions of our robot
     * relative to driver commands, using automation when necessary
     */
    public void operatorControl() {
    	//don't declare stuff here
    	frWheel.enableBrakeMode(false);
    	flWheel.enableBrakeMode(false);
    	brWheel.enableBrakeMode(false);
    	blWheel.enableBrakeMode(false);

    	double leftAxis, rightAxis;
    	double wenchAxis;
    	double wheelDZ = 0.15;
    	double tempTune;
    	int dashCount = 0;
    	double shooterVoltage = 10;
    	
        while (isOperatorControl() && isEnabled()) {
			/////////////////////////
			//         LIDAR       //
			/////////////////////////
        	serial.update();
           
            SmartDashboard.putNumber("rawLidar", serial.getLidarValue());
            SmartDashboard.putNumber("lidar", serial.getAverageLidarValue());
            System.out.println("" + serial.getAverageLidarValue() + " " + serial.getLidarValue() +  " " + shooterVoltage);
			/////////////////////////
			//  Primary Controller //
			/////////////////////////
        	driverController.update();
        	
			/////////////////////////
			//       Modules       //
			/////////////////////////
        	if(driverController.onPress(TriggerButton.lTrigger))
        		drivetrain.incrementAllModules(-1);
        	
        	if(driverController.onPress(TriggerButton.rTrigger))
        		drivetrain.incrementAllModules(1);
        	
        	if(driverController.onPress(Button.rBumper))
        		drivetrain.turnOutwards();
        	
        	if(driverController.onPress(Button.lBumper))
        		drivetrain.turnInwards();
        	
        	if(driverController.whileHeld(Button.aButton))
        	{
        		tempTune = 0.4;
        		frModule.setP(tempTune);
        		flModule.setP(tempTune);
        		blModule.setP(tempTune);
        		brModule.setP(tempTune);
        	}
        	else
        	{
        		tempTune = allTuning;
        		frModule.setP(tempTune);
        		flModule.setP(tempTune);
        		blModule.setP(tempTune);
        		brModule.setP(tempTune);
        	}
        	SmartDashboard.putNumber("tempTune", tempTune);
        	
			/////////////////////////
			//     Drive Wheels    //
			/////////////////////////
        	leftAxis = deadZone(driverController.getRawAxis(1), wheelDZ);
        	rightAxis = deadZone(driverController.getRawAxis(5)/* * -1*/, wheelDZ); //unreversed
        	
        	drivetrain.setLeftWheels(leftAxis);
        	drivetrain.setRightWheels(rightAxis);
        	
			//////////////////////////
			// Secondary Controller //
			//////////////////////////
			manipulatorController.update();
			
			/////////////////////////
			//      Collector      //
			///////////////////////// TODO add if becomes a problem
			if(!autoShooting)
			{
				if(manipulatorController.whileHeld(TriggerButton.lTrigger)) //intake
				{
					//if(rearCollector.get())
					SmartDashboard.putString("intakeTest", "intaking");
						collector.set(1.0);
						//else set 0
				}
				else if(manipulatorController.whileHeld(TriggerButton.rTrigger))
				{
					SmartDashboard.putString("intakeTest", "outtaking");
					//if(rearCollector.get())
						collector.set(-1.0);
						//else set 0
				}
				else
				{
					SmartDashboard.putString("intakeTest", "not running");
					collector.set(0);
				}
			}
			
			/////////////////////////
			//    Shooter Wheels   //
			/////////////////////////
			shooterVoltage = (12.2204) * Math.pow((.99989), (serial.getAverageLidarValue()));
			shooterVoltage += 0.5;
			if(shooterVoltage < 8.9)
				shooterVoltage = 8.9;
			
			if(manipulatorController.onPress(Button.rBumper))
			{
				overideShooting = true;
				overideHigh = false;
			}
			if(manipulatorController.onPress(Button.xButton))
			{
				overideShooting = false;
				overideHigh = true;				
			}
			if(manipulatorController.onPress(Button.lBumper))
			{
				overideShooting = false;
				overideHigh = false;
			}
			
			if(overideShooting)
				shooterVoltage = 8.9;
			if(overideHigh)
				shooterVoltage = 11;
			
			if(manipulatorController.whileHeld(Button.aButton))
			{
				autoShooting = true;
				if(shootTimer.get() == 0)
					shootTimer.start();
				rightShooter.set(-shooterVoltage);
				leftShooter.set(shooterVoltage);
				if(shootTimer.get() > shotLength)
				{
					collector.set(1.0);
				}
        	}
        	else
        	{
        		autoShooting = false;
        		rightShooter.set(0);
        		leftShooter.set(0);
        		shootTimer.reset();
        	}
        	
			
			SmartDashboard.putNumber("shooterVoltage", shooterVoltage);
			SmartDashboard.putNumber("shotLength", shotLength);
			SmartDashboard.putBoolean("autoShooting", autoShooting);
			
			SmartDashboard.putNumber("rightShooterVoltage", rightShooter.getOutputVoltage());
			SmartDashboard.putNumber("leftShooterVoltage", leftShooter.getOutputVoltage());
			
			SmartDashboard.putNumber("shootTimer", shootTimer.get());
			/////////////////////////
			//    Camera Toggle    //
			///////////////////////// TODO fix
			
			double currentPOVval = driverController.getPOV();
			if(currentPOVval == 270 && prevPOVval != 270)
			{
				if(isCam1)
				{
					NIVision.IMAQdxStopAcquisition(session1);
					NIVision.IMAQdxConfigureGrab(session0);
					isCam1 = false;
				}
				else if(!isCam1)
				{
					NIVision.IMAQdxStopAcquisition(session0);
					NIVision.IMAQdxConfigureGrab(session1);
					isCam1 = true;
				}
			}
			prevPOVval = currentPOVval;
			if(isCam1)
			{
				NIVision.IMAQdxGrab(session1, frame, 1);
			}
			else
			{
				NIVision.IMAQdxGrab(session0, frame, 1);
			}
			CameraServer.getInstance().setImage(frame);
			
			/////////////////////////
			//  Collector Extender //
			/////////////////////////
			
			if(manipulatorController.getPOV() != -1)
			{
				//change this
				if(manipulatorController.getPOV() == 0) //forwards
				{
					//if(frontCollector.get())
	        			collectorExtender.set(-1.0);
	        		//else
	        		//	collectorExtender.set(0);
				}
				else if(manipulatorController.getPOV() == 180)
				{
					//if(rearCollector.get())
	        			collectorExtender.set(1.0);
	        	//	else
	        	//		collectorExtender.set(0);
				}
				else
	        	{
	        		collectorExtender.set(0);
	        	}
			}
			else
        	{
        		collectorExtender.set(0);
        	}
        	
			
			/////////////////////////
			//        Lifter       //
			/////////////////////////
			
			
			if(manipulatorController.onPress(Button.bButton))
			{
				if(rearCollector.get())
				{
					lifterRelease.set(lifterRelease.get() - 0.1875); //test lifterPosition -0.1875
				}
			}
			
			if(manipulatorController.onPress(Button.yButton))
			{
				if(rearCollector.get())
				{
					lifterRelease.set(lifterRelease.get() + 0.1875);
				}
			}
			
			wenchAxis = deadZone(manipulatorController.getRawAxis(1) * -1, wheelDZ);
			if(wenchAxis > 0)
				wenchAxis = 0;
        	wench.set(wenchAxis);
        	
			       	
        	/////////////////////////
        	//   Print Everything  //
        	/////////////////////////
        	dashCount++;
        	if(dashCount > 20000)
        		dashCount = 0;
        	if(dashCount % 1 == 0) //TODO change this
        	{
        		printManipulatorInfo();
        		//printModuleInfo();
        	}
			/////////////////////////
			//   Test All Modules  //
			/////////////////////////

        	frWheel.test();
        	flWheel.test();
        	blWheel.test();
        	brWheel.test();

        	talon7.test();
        	talon2.test();
        	talon4.test();
        	talon5.test();

        	if((talon7.isDisabled() || talon2.isDisabled() || talon4.isDisabled() || talon5.isDisabled()))
        	{
        		talon7.disable();
        		talon2.disable();
        		talon4.disable();
        		talon5.disable();
        	}
        	
        	if(manipulatorController.onPress(Button.back))
        	{
        		talon7.enable();
        		talon2.enable();
        		talon4.enable();
        		talon5.enable();
        		frModule.resetTarget();
        		flModule.resetTarget();
        		blModule.resetTarget();
        		brModule.resetTarget();
        		
        		collector.enable();
        	}
        	if(manipulatorController.onPress(Button.start))
        	{
        		talon7.disable();
        		talon2.disable();
        		talon4.disable();
        		talon5.disable();
        	}
        	
        	
        	collector.test();
        	
        	//lifterRelease.test();
        	collectorExtender.test();
        	
       

            Timer.delay(0.005);		// wait for a motor update time
        }
    }
    
	/**
	 * Utility print statements
	 */
    public void printManipulatorInfo()
    {
    	SmartDashboard.putNumber("gyro", gyro.getAngle());
    	
    	SmartDashboard.putBoolean("frontCollector", !frontCollector.get());
    	SmartDashboard.putBoolean("rearCollector", !rearCollector.get());	
    	
    	SmartDashboard.putNumber("leftShooterCurrent", leftShooter.getOutputCurrent());
    	SmartDashboard.putNumber("rightShooterCurrent", rightShooter.getOutputCurrent());
    }
    
    public void printModuleInfo()
    {
    	SmartDashboard.putNumber("backRightErr", talon5.getError());
    	SmartDashboard.putNumber("backLeftErr", talon4.getError());
    	SmartDashboard.putNumber("frontRightErr", talon7.getError());
    	SmartDashboard.putNumber("frontLeftErr", talon2.getError());
    	
    	SmartDashboard.putNumber("brPos", brModule.getDoublePosition());
    	SmartDashboard.putNumber("blPos", blModule.getDoublePosition());
    	SmartDashboard.putNumber("frPos", frModule.getDoublePosition());
    	SmartDashboard.putNumber("flPos", flModule.getDoublePosition());
    	
    	SmartDashboard.putNumber("pos2", brModule.getModuloPosition());
    	SmartDashboard.putNumber("pos3", brModule.getDesiredPosition());

    	SmartDashboard.putBoolean("withinStopBR", brModule.withinRange());
    	SmartDashboard.putBoolean("withinStopBL", blModule.withinRange());
    	SmartDashboard.putBoolean("withinStopFR", frModule.withinRange());
    	SmartDashboard.putBoolean("withinStopFL", flModule.withinRange());
    	
    	SmartDashboard.putNumber("powBR", talon5.getOutputCurrent());
    	SmartDashboard.putNumber("powBL", talon4.getOutputCurrent());
    	SmartDashboard.putNumber("powFR", talon7.getOutputCurrent());
    	SmartDashboard.putNumber("powFL", talon2.getOutputCurrent());
    	
    	SmartDashboard.putNumber("voltageFL", talon2.getOutputVoltage());
    	SmartDashboard.putNumber("voltageBL", talon4.getOutputVoltage());
    	SmartDashboard.putNumber("voltageBR", talon5.getOutputVoltage());
    	SmartDashboard.putNumber("voltageFR", talon7.getOutputVoltage());
    	
    	SmartDashboard.putBoolean("disabledFL", talon2.isDisabled());
    	SmartDashboard.putBoolean("disabledBL", talon4.isDisabled());
    	SmartDashboard.putBoolean("disabledBR", talon5.isDisabled());
    	SmartDashboard.putBoolean("disabledFR", talon7.isDisabled());
    	
		/////////////////////////
		//     Max Currents    //
		/////////////////////////
		double[] maxVal = new double[4];
		double[] temp = new double[4];
		
		temp[0] = talon5.getOutputCurrent();
		if(temp[0] > maxVal[0])
		maxVal[0] = temp[0];
		SmartDashboard.putNumber("maxBR", maxVal[0]);
		
		temp[1] = talon4.getOutputCurrent();
		if(temp[1] > maxVal[1])
		maxVal[1] = temp[1];
		SmartDashboard.putNumber("maxBL", maxVal[1]);
		
		temp[2] = talon7.getOutputCurrent();
		if(temp[2] > maxVal[2])
		maxVal[2] = temp[2];
		SmartDashboard.putNumber("maxFR", maxVal[2]);
		
		temp[3] = talon2.getOutputCurrent();
		if(temp[3] > maxVal[3])
		maxVal[3] = temp[3];
		SmartDashboard.putNumber("maxFL", maxVal[3]);
    }
    /**
     * Checks if a value is within a "dead zone"
     * @param val value to check
     * @param zone range to check
     * @return the value if it is outside of the zone, 0 if it is within the deadZone
     */
    public double deadZone(double val, double zone)
    {
    	if(val < zone && val > -zone)
    		return 0;
    	return val;
    }
    
    public void practice()
    {
    	
    }

    public void test() {
    	
    }
}
