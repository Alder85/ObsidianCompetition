package org.usfirst.frc.team2220.robot;

import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.Joystick;
/**
 * The much needed extension of the inadequate Joystick class, XBoxController.<br>
 * Keeps track of button states and divides them into onPress, whileHeld, and onRelease.<br>
 * Only catch is that XBoxController.update() must be called once per loop
 * @author Josh
 *
 */
public class XBoxController extends Joystick{
	
	private Button[] buttonArray = new Button[13];
	private int joystickNumber;
	
	/**
	 * Initalize button array
	 * @param in the port of the joystick
	 */
	public XBoxController(int in)
	{
		super(in);
		joystickNumber = in;
		buttonArray[1] = Button.aButton;
		buttonArray[2] = Button.bButton;
		buttonArray[3] = Button.xButton;
		buttonArray[4] = Button.yButton;
		buttonArray[5] = Button.lBumper;
		buttonArray[6] = Button.rBumper;
		buttonArray[7] = Button.back;
		buttonArray[8] = Button.start;
		buttonArray[9] = Button.lStickPress;
		buttonArray[10] = Button.rStickPress; //triggers are axes
		
	}
	
	/**
	 * enum for all Button (not axes) types on xbox controller
	 * @author Josh
	 *
	 */
	public enum Button {
	    aButton(1), bButton(2), xButton(3), yButton(4), lBumper(5), 
	    rBumper(6), back(7), start(8), lStickPress(9), rStickPress(10);

	    public int value;
	    public boolean[] pressed  = {false, false};
	    public boolean[] oldValue = {false, false};

	    private Button(int value) {
	      this.value = value;
	    }
	}
		
	/**
	 * 
	 * @param x button number to use
	 * @return true only once when button is pressed
	 */
	public boolean onPress(Button x)
	{
		return (x.pressed[joystickNumber] && !x.oldValue[joystickNumber]);
	}
	
	/**
	 * 
	 * @param x button to read
	 * @return true constantly while button is held
	 */
	public boolean whileHeld(Button x) 
	{
		return x.pressed[joystickNumber];
	}
	
	/**
	 * 
	 * @param x button to check
	 * @return true only once while button is released
	 */
	public boolean onRelease(Button x)
	{
		return (!x.pressed[joystickNumber] && x.oldValue[joystickNumber]);
	}
	
	/**
	 * Treats the triggers like a button, using 85% pressed as the boundary
	 * @author Josh
	 *
	 */
	public enum AxisButton
	{
		rTrigger(3), lTrigger(2), rightY(5);
		
		public int value;
		public double[] currVal = {0, 0};
		public double[] oldVal  = {0, 0};
		
		private AxisButton(int value)
		{
			this.value = value;
		}
	}
	
	/**
	 * 
	 * @param x trigger to check
	 * @return true only once when AxisButton is pressed
	 */
	public boolean onPressPos(AxisButton x)
	{
		return (x.currVal[joystickNumber] > 0.85 && x.oldVal[joystickNumber] < 0.85);
	}
	
	public boolean onPressNeg(AxisButton x)
	{
		return (x.currVal[joystickNumber] < -0.85 && x.oldVal[joystickNumber] > -0.85);
	}

	/**
	 * 
	 * @param x button to check
	 * @return true constantly while AxisButton is held
	 */
	public boolean whileHeldPos(AxisButton x) 
	{
		return x.currVal[joystickNumber] > 0.85;
	}
	public boolean whileHeldNeg(AxisButton x) 
	{
		return x.currVal[joystickNumber] < -0.85;
	}

	/**
	 * 
	 * @param x button to check
	 * @return true only once when AxisButton is released
	 */
	public boolean onReleasePos(AxisButton x)
	{
		return (x.currVal[joystickNumber] < 0.85 && x.oldVal[joystickNumber] > 0.85);
	}
	public boolean onReleaseNeg(AxisButton x)
	{
		return (x.currVal[joystickNumber] > -0.85 && x.oldVal[joystickNumber] < -0.85);
	}
	/**
	 * POV enum to treat the POV like a button
	 * @author Josh
	 *
	 */
	public enum POV
	{
		POV(0);
		
		private int value;
		private double oldVal = -1, currVal = -1;
		private POV(int val)
		{
			this.value = val;
		}
	}
	
	/**
	 * 
	 * @param x POV to read
	 * @param val which POV direction to read
	 * @return true once when POV value is pressed
	 */
	public boolean onPressPOVVal(POV x, double val)
	{
		return (x.oldVal != val) && (x.currVal == val);
	}
	
	/**
	 * 
	 * @param x POV to read
	 * @param val which POV direction to read
	 * @return true constantly while POV value is held
	 */
	public boolean whileHeldPOVVal(POV x, double val)
	{
		return (x.oldVal == x.currVal);
	}
	
	/**
	 * 
	 * @param x POV to read
	 * @param val which POV direction to read
	 * @return true once when POV value is released
	 */
	public boolean onReleasePOVVal(POV x, double val)
	{
		return (x.oldVal == val) && (x.currVal != val);
	}

	/**
	 * Call this once per robot loop
	 * This will be able to differentiate pressed, held and released
	 */
	public void update()
	{
		for(int i = 1;i <= 10;i++)
		{
			buttonArray[i].oldValue[joystickNumber] = buttonArray[i].pressed[joystickNumber];
			buttonArray[i].pressed[joystickNumber] = this.getRawButton(buttonArray[i].value);
		}
		
		
		AxisButton.rTrigger.oldVal[joystickNumber] = AxisButton.rTrigger.currVal[joystickNumber];
		AxisButton.rTrigger.currVal[joystickNumber] = this.getRawAxis(3);
		
		AxisButton.lTrigger.oldVal[joystickNumber] = AxisButton.lTrigger.currVal[joystickNumber];
		AxisButton.lTrigger.currVal[joystickNumber] = this.getRawAxis(2);
		
		AxisButton.rightY.oldVal[joystickNumber] = AxisButton.rightY.currVal[joystickNumber];
		AxisButton.rightY.currVal[joystickNumber] = this.getRawAxis(AxisButton.rightY.value);
		
		POV.POV.oldVal = POV.POV.currVal;
		POV.POV.currVal = this.getPOV();
		
	}
}
