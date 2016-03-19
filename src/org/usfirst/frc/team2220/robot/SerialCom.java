package org.usfirst.frc.team2220.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;

public class SerialCom {
	
	SerialPort serial;
    private static final int RIODUINO_SERIAL_BAUD = 9600;
    private static final SerialPort.Port RIODUINO_SERIAL_PORT = SerialPort.Port.kMXP;
    private static final SerialPort.WriteBufferMode RIODUINO_SERIAL_WRITE_BUFFER_MODE = SerialPort.WriteBufferMode.kFlushOnAccess;
    private static final double RIODUINO_SERIAL_TIMEOUT = 1;
    private static final SerialPort.FlowControl RIODUINO_SERIAL_FLOW_CONTROL = SerialPort.FlowControl.kNone;
    int nbytes;
    byte[] buffer = new byte[8];
    boolean LEDon = false;
    boolean cmd_sent = false;
    
    private double currentLidarValue;
    private double autonomousValue;
    private double LEDValue;
    ArrayList<Double> lidarValues = new ArrayList();
	int lidarSum = 0;
	double lidarAverage = -42;
	
	public SerialCom()
	{
		serial = new SerialPort(RIODUINO_SERIAL_BAUD, RIODUINO_SERIAL_PORT);
		serial.reset();
        serial.setWriteBufferMode(RIODUINO_SERIAL_WRITE_BUFFER_MODE);
        serial.setReadBufferSize(1);
        serial.setWriteBufferSize(1);
        serial.setTimeout(RIODUINO_SERIAL_TIMEOUT);
        serial.setFlowControl(RIODUINO_SERIAL_FLOW_CONTROL);
        serial.disableTermination();
	}
	
	public void update()
	{
		try
		{
			nbytes = serial.getBytesReceived();
	        if (nbytes > 1) //change this as you read more values
	        {

	        	buffer = serial.read(nbytes);
	            currentLidarValue = ((buffer[0] & 0xFF) << 8 | (buffer[1] & 0xFF));
	            //autonomousValue = (buffer[0] & 0xFF);
	            
	            if(lidarValues.size() > 5)
	            {
	            	//if(currentLidarValue < lidarSum + 15000)
	            	if(currentLidarValue< 15000)
	            		lidarValues.add(currentLidarValue);	
	            }
	            else
	            	lidarValues.add(currentLidarValue);
	        }
		}
		catch(Exception e)
		{
			System.out.println(e);
		}
        if(lidarValues.size() > 10)
        	lidarValues.remove(0);
        lidarSum = 0;
        for(int i = 0;i < lidarValues.size();i++)
        	lidarSum += lidarValues.get(i);
        if(lidarValues.size() != 0)
        	lidarAverage = lidarSum / lidarValues.size();
	}
	
	public double getLidarValue()
	{
		return currentLidarValue;
	}
	
	public double getAverageLidarValue()
	{
		return lidarAverage;
	}
}








