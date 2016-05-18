package org.usfirst.frc.team2220.robot;

import java.util.Comparator;
import java.util.Vector;


//import org.usfirst.frc.team2220.robot.Robot.ParticleReport;
//import org.usfirst.frc.team2220.robot.Robot.Scores;

//import org.usfirst.frc.team2220.robot.Robot.ParticleReport; wut
//import org.usfirst.frc.team2220.robot.Robot.Scores;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.vision.AxisCamera.ExposureControl;
import edu.wpi.first.wpilibj.vision.AxisCamera.WhiteBalance;

/**
 * Highly modified version of 2015 Vision Sample Program<br>
 * Currently boxes targets on an image, circles the brightest one, then gives
 * statistics about location of the brightest target
 */
public class ImageProcessorTemp {

	//A structure to hold measurements of a particle
			public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
				double PercentAreaToImageArea;
				double Area;
				double BoundingRectLeft;
				double BoundingRectTop;
				double BoundingRectRight;
				double BoundingRectBottom;
				
				public int compareTo(ParticleReport r)
				{
					return (int)(r.Area - this.Area);
				}
				
				public int compare(ParticleReport r1, ParticleReport r2)
				{
					return (int)(r1.Area - r2.Area);
				}
			};

			//Structure to represent the scores for the various tests used for target identification
			public class Scores {
				double Area;
				double Aspect;
			};
			
	//Images
			Image frame;
			Image binaryFrame;
			int imaqError;
			AxisCamera camera;

			/*
			 HSV
			 57, 255
			 0, 140
			 88, 188
			 HSL
			 43, 231
			 0, 147
			 78, 196
			 RGB
			 66, 255
			 53, 255,
			 100, 176
			 */
			//Constants
			
			
			NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(50, 255);	//Default hue range for yellow tote
			NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(50, 255);	//Default saturation range for yellow tote
			NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(50, 255);	//Default value range for yellow tote
			
			/*
			NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(0, 255);	//Default hue range for yellow tote
			NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(0, 45);	//Default saturation range for yellow tote
			NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(60, 164);	//Default value range for yellow tote
			
			/*
			NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(50, 255);	//Default hue range for yellow tote
			NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(0, 255);	//Default saturation range for yellow tote
			NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(50, 255);	//Default value range for yellow tote
			*/
			double AREA_MINIMUM = 0.1; //Default Area minimum for particle as a percentage of total image area
			double AREA_MAXIMUM = 100.0;
			double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
			double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
			double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
			double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
			NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
			NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
			Scores scores = new Scores();	

	double distance = 0;

	double leftDistance, rightDistance, modHeight;

	NIVision.Rect rect;
	//CameraServer server;
	int session;

	/**
	 * Creates frames an takes a session
	 * 
	 * @param inSession
	 *            session from the main class to use, so cameras can switch fine
	 */
	public ImageProcessorTemp() {
		camera = new AxisCamera("axis-camera.local");
		camera.writeColorLevel(100);
		camera.writeBrightness(26);
		camera.writeWhiteBalance(WhiteBalance.kFixedOutdoor1);
		camera.writeExposureControl(ExposureControl.kAutomatic);
		System.out.println(camera);
		
		
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);


	}

	/**
	 * Takes an image and processes it, looking for targets and finding
	 * information about the brightest target
	 * 
	 * @return if a target was found
	 */
	boolean lookForTarget(boolean favorLeft) {
		camera.getImage(frame);
		
		//Threshold the image looking for yellow (tote color)
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

		//Send particle count to dashboard
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		SmartDashboard.putNumber("Masked particles", numParticles);

		//Send masked image to dashboard to assist in tweaking mask.
		CameraServer.getInstance().setImage(binaryFrame);

		//filter out small particles
		float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
		criteria[0].lower = areaMin;
		imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

		//Send particle count after filtering to dashboard
		numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		SmartDashboard.putNumber("Filtered particles", numParticles);

		if(numParticles > 0)
		{
			//Measure particles and sort by particle size
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				ParticleReport par = new ParticleReport();
				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
				par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
				par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				particles.add(par);
			}
			particles.sort(null);
			if(favorLeft)
			{
				if(particles.get(1).BoundingRectLeft < particles.get(0).BoundingRectLeft)
				{
					ParticleReport temp = particles.get(0);
					particles.set(0, particles.get(1));
					particles.set(1, temp);
				}
			}
				
				leftDistance = (int) particles.elementAt(0).BoundingRectLeft;
				rightDistance = 640 - ((int) particles.elementAt(0).BoundingRectRight);
				SmartDashboard.putNumber("modHeight", modHeight);
				SmartDashboard.putNumber("leftDistance", leftDistance);
				SmartDashboard.putNumber("rightDistance", rightDistance);
				int topBound = (int) particles.elementAt(0).BoundingRectTop;
				int leftBound = (int) particles.elementAt(0).BoundingRectLeft;
				int height = (int) particles.elementAt(0).BoundingRectBottom
						- (int) particles.elementAt(0).BoundingRectTop;
				int width = (int) particles.elementAt(0).BoundingRectRight
						- (int) particles.elementAt(0).BoundingRectLeft;
				rect = new NIVision.Rect(topBound, leftBound, height, width);
				NIVision.imaqDrawShapeOnImage(binaryFrame, binaryFrame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 1.0f);

			//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
			//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
			//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
			scores.Aspect = AspectScore(particles.elementAt(0));
			SmartDashboard.putNumber("Aspect", scores.Aspect);
			scores.Area = AreaScore(particles.elementAt(0));
			SmartDashboard.putNumber("Area", scores.Area);
			boolean isTote = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;

			//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
			SmartDashboard.putBoolean("IsTote", isTote);
			SmartDashboard.putNumber("Distance", computeDistance(binaryFrame, particles.elementAt(0)));
			return true;
		} 
		else 
		{
			SmartDashboard.putBoolean("IsTote", false);
			return false;
		}

		
	}

	/**
	 * Gets the distance from the right side of the frame to the target and
	 * takes away the distance from the left side of the frame to the target.<br>
	 * This tells you whether or not the target is centered
	 * @return right distance minus left distance
	 */
	public double getLeftRightDistance(boolean isAuto) {
		double loopTimes = 1;
		double tempLeft = 0, tempRight = 0;
		for (int i = 0; i < loopTimes; i++) {
			if (lookForTarget(isAuto)) {
				tempLeft += leftDistance;
				tempRight += rightDistance;
			}
		}
		tempLeft /= loopTimes;
		tempRight /= loopTimes;
		return tempRight - tempLeft;
	}

	/**
	 * Height of the target relative to the frame
	 * @return pixels from the bottom of the target to the bottom of the frame
	 */
	public double getHeightDistance() {
		return modHeight;
	}

	/**
	 * Returns the calculated distance to the target, somewhat inaccurately
	 * @return distance value, in feet
	 */
	public double getDistance() {
		return distance;
	}



	// Comparator function for sorting particles. Returns true if particle 1 is
	// larger
	static boolean CompareParticleSizes(ParticleReport particle1, ParticleReport particle2) {
		// we want descending sort order
		return particle1.PercentAreaToImageArea > particle2.PercentAreaToImageArea;
	}

	/**
	 * Converts a ratio with ideal value of 1 to a score. The resulting function
	 * is piecewise linear going from (0,0) to (1,100) to (2,0) and is 0 for all
	 * inputs outside the range 0-2
	 */
	double ratioToScore(double ratio) {
		return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
	}
	
	double AreaScore(ParticleReport report) {
		double boundingArea = (report.BoundingRectBottom - report.BoundingRectTop)
				* (report.BoundingRectRight - report.BoundingRectLeft);
		// Tape is 7" edge so 49" bounding rect. With 2" wide tape it covers 24"
		// of the rect.
		return ratioToScore((49 / 24) * report.Area / boundingArea);
	}

	/**
	 * Method to score if the aspect ratio of the particle appears to match the
	 * retro-reflective target. Target is 7"x7" so aspect should be 1
	 */
	double AspectScore(ParticleReport report) {
		return ratioToScore(((report.BoundingRectRight - report.BoundingRectLeft)
				/ (report.BoundingRectBottom - report.BoundingRectTop)));
	}

	/**
	 * Computes the estimated distance to a target using the width of the
	 * particle in the image. For more information and graphics showing the math
	 * behind this approach see the Vision Processing section of the
	 * ScreenStepsLive documentation.
	 *
	 * @param image
	 *            The image to use for measuring the particle estimated
	 *            rectangle
	 * @param report
	 *            The Particle Analysis Report for the particle
	 * @param isLong
	 *            Boolean indicating if the target is believed to be the long
	 *            side of a tote
	 * @return The estimated distance to the target in feet.
	 */
	double computeDistance(Image image, ParticleReport report) {
		double normalizedWidth, targetWidth;
		NIVision.GetImageSizeResult size;

		size = NIVision.imaqGetImageSize(image);
		normalizedWidth = 2 * (report.BoundingRectRight - report.BoundingRectLeft) / size.width;
		targetWidth = 20;

		return targetWidth / (normalizedWidth * 12 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
	}
}
