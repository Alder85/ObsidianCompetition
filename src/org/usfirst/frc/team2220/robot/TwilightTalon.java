package org.usfirst.frc.team2220.robot;
import edu.wpi.first.wpilibj.*;

/**
 * Extension of TwilightTalon, primarily for check if current limits have been surpassed,
 * and if they have, disabling the motor
 * @author Josh
 *
 */
public class TwilightTalon extends CANTalon{
	private double maxTemp;
	private double maxCurrent;
	private boolean disabled = false;
	private boolean tripped = false;
	private Timer timer = new Timer();
	private Timer resetTimer = new Timer();
	private double tripTime = 0.25; //change this is all motors are trippin
	
	/**
	 * Cast of CANTalon class
	 * Checks maximum current and temperature, and stops motors if they exceed those
	 * @param port CAN Port Number
	 */
	public TwilightTalon(int port) {
		super(port);
		maxCurrent = 30.0;	//Stall Current for RS 775 12V + 10A
		maxTemp = 500.0;	//We don't use this
	}
	
	/**
	 * @param newCurrent maximum allowable current
	 */
	public void setMaxCurrent(double newCurrent) {
		maxCurrent = newCurrent;
	}
	
	/**
	 * @param newTemp maximum allowable temperature
	 */
	public void setMaxTemp(double newTemp) {
		maxTemp = newTemp;
	}
	
	/**
	 * Does not allow you to set new points if the talon is disabled
	 */
	@Override
	public void set(double setpoint)
	{
		if(!disabled)
		{
			super.set(setpoint);
		}
	}
	
	/**
	 * Keeps track of when the talon is disabled
	 */
	@Override
	public void disable()
	{
		super.disable();
		disabled = true;
	}
	
	/**
	 * keeps track of when the talon is enabled
	 */
	@Override
	public void enable()
	{
		super.enable();
		disabled = false;
	}
	
	/**
	 * @return whether or not the talon is disabled
	 */
	public boolean isDisabled()
	{
		return disabled;
	}
	/**
	 * Tests whether Talon is within 'safe' levels<br>
	 * if unsafe levels are surpassed for a time exceeding the tripTime variable, the motor is disabled
	 * @return Whether the test was passed
	 */
	public boolean test() {
		boolean test = true;
		if (isOverMaxCurrent()) 
			test = false;
		if(!test)
		{
			if(!tripped)
			{
				timer.reset();
				timer.start();
				tripped = true;
			}
			
			if(timer.get() > tripTime) 
			{
				timer.stop();
				timer.reset();
				this.disable();
				disabled = true;
			}
			
		}
		else
		{
			tripped = false;
			timer.stop();
			timer.reset();
		}
		return test;
	}
	
	/**
	 * Tests whether Talon is within 'safe' levels<br>
	 * if unsafe levels are surpassed for a time exceeding the tripTime variable, the motor is disabled
	 * @return Whether the test was passed
	 */
	public boolean talon8test() {
		boolean test = true;
		if (isOverMaxCurrent()) 
			test = false;
		if(!test)
		{
			if(!tripped)
			{
				timer.reset();
				timer.start();
				tripped = true;
			}
			
			if(timer.get() > tripTime) 
			{
				timer.stop();
				timer.reset();
				System.out.println("riperoni talon8-oni");
				//this.disable();
				//disabled = true;
			}
			
		}
		else
		{
			tripped = false;
			timer.stop();
			timer.reset();
		}
		return test;
	}
	
	/**
	 * @return current is surpassing maximum
	 */
	public boolean isOverMaxCurrent() {
		double CurrCurrent = this.getOutputCurrent();
		return CurrCurrent > maxCurrent;
	}
	
	/**
	 * @return temperature is surpassing maximum
	 */
	public boolean isOverMaxTemp() {
		double CurrTemp = this.getTemperature();
		return CurrTemp > maxTemp;
	}
	
	/**
	 * Stops the motor
	 */
	public void stop() {
		this.disableControl();
	}
	
	/**
	 * prints warning to the console, not the SmartDashboard
	 */
	public void printWarning () {	//Checks to see if thresholds are passed
		if (isOverMaxCurrent()) {
			System.out.println("Currently over Maximum Current"); //insert method that prints to dashboard
		}
		if (isOverMaxTemp()) {
			System.out.println("Currently over Maximum Temperature"); //insert method that prints to dashboard
		}
	}
	
	/**
	 * String output for printing to the SmartDashboard OR Console
	 * @return String containing the talon's current condition
	 */
	public String toString() {
		double CurrTemp = this.getTemperature(); //Returns Celsius
		double CurrCurrent = this.getOutputCurrent(); //Returns Amperes
		String text = "Current Temperature: " + CurrTemp + ", Current Current: " + CurrCurrent;
		return text;
	}
	
}
