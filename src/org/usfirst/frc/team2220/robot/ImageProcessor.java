package org.usfirst.frc.team2220.robot;

import java.util.Comparator;
import java.util.Vector;

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

/**
 * Highly modified version of 2015 Vision Sample Program<br>
 * Currently boxes targets on an image, circles the brightest one, then gives
 * statistics about location of the brightest target
 */
public class ImageProcessor {

	// Images
	Image frame;
	Image binaryFrame;
	int imaqError;

	// Constants
	/*
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(90, 135); // brightgreen/white
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(80, 255);
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(0, 220); //0, 255
	*/
	
	/*
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(26, 135); // brightgreen/white
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(60, 196);
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(52, 231);
	*/
	NIVision.Range targetR = new NIVision.Range(150, 252);
	NIVision.Range targetG = new NIVision.Range(245, 255);
	NIVision.Range targetB = new NIVision.Range(234, 255);
	
	NIVision.Range DB_HUE = new NIVision.Range(59, 137); // lower previously 104
	NIVision.Range DB_SAT = new NIVision.Range(64, 255); //lower previously 128
	NIVision.Range DB_VAL = new NIVision.Range(224, 255);
	
	NIVision.Range HOPE_HUE = new NIVision.Range(110, 134); // brightgreen/white
	NIVision.Range HOPE_SAT = new NIVision.Range(157, 255);
	NIVision.Range HOPE_VAL = new NIVision.Range(245, 255);
	
	NIVision.Range Guess5_HUE = new NIVision.Range(59, 146);
	NIVision.Range Guess5_SAT = new NIVision.Range(0, 255);
	NIVision.Range Guess5_VAL = new NIVision.Range(146, 238);
	
	// use this maybe
	/*
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(113, 124); // brightgreen/white
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(0, 255);
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(240, 255);
	//*/
	/*
	NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(0, 255); // brightgreen/white
	NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(0, 255);
	NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(0, 255);
	*/
	double AREA_MINIMUM = 0.25; // Default Area minimum for particle as a
								// percentage of total image area
	double AREA_MAXIMUM = 10.00; // default max
	double SCORE_MIN = 75.0; // TODO remove this trash
	double VIEW_ANGLE = 60; // View angle for camera, set to Axis m1011 by
							// default, 64 for m1013, 51.7 for 206, 52 for
							// HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0, 0, 1, 1);
	Scores scores = new Scores();
	double distance = 0;

	double leftDistance, rightDistance, modHeight;

	NIVision.Rect rect;
	CameraServer server;
	int session;

	/**
	 * Creates frames an takes a session
	 * 
	 * @param inSession
	 *            session from the main class to use, so cameras can switch fine
	 */
	public ImageProcessor(int inSession) {
		session = inSession;
		// session = NIVision.IMAQdxOpenCamera("cam1",
		// NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		// NIVision.IMAQdxStopAcquisition(inSession);
		// NIVision.IMAQdxConfigureGrab(session);

		// create images
		frame = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM,
				AREA_MAXIMUM, 0, 0);

	}

	/**
	 * Takes an image and processes it, looking for targets and finding
	 * information about the brightest target
	 * 
	 * @return if a target was found
	 */
	boolean lookForTarget() {
		// get image and particle numbers
		NIVision.IMAQdxGrab(session, frame, 1);
		//NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);
		//NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.RGB, targetR, targetB, targetG);
		NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, DB_HUE, DB_SAT, DB_VAL);
		//NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, HOPE_HUE, HOPE_SAT, HOPE_VAL);
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);

		// filter out small particles
		float areaMin = (float) AREA_MINIMUM;
		criteria[0].lower = areaMin;
		imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

		// count relevant particles
		numParticles = NIVision.imaqCountParticles(binaryFrame, 1);

		// processing boundaries
		if (numParticles > 0) {
			// Measure particles and sort by particle size
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			for (int particleIndex = 0; particleIndex < numParticles; particleIndex++) {
				ParticleReport par = new ParticleReport();
				par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_AREA);
				par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
				par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0,
						NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
				// if(par.BoundingRectLeft > 100) //use this to limit frame
				// search size
				if(par.BoundingRectRight < (640.0 * (2.0 / 3.0)) && par.BoundingRectLeft > (640.0 / 3.0) && par.BoundingRectBottom < 240)
					particles.add(par);
			}
			particles.sort(null);

			if (particles.size() == 0) {
				CameraServer.getInstance().setImage(frame);
				return false;
			}

			// figures out which particle is the most OP
			double maxAspect = AspectScore(particles.elementAt(0));

			int maxIndex = 0;
			for (int i = 1; i < particles.size(); i++) {
				if (AspectScore(particles.elementAt(i)) > maxAspect) {
					maxAspect = AspectScore(particles.elementAt(i));
					maxIndex = i;
				}
			}

			for (int i = 0; i < particles.size(); i++) {
				int topBound = (int) particles.elementAt(i).BoundingRectTop;
				int leftBound = (int) particles.elementAt(i).BoundingRectLeft;
				int height = (int) particles.elementAt(i).BoundingRectBottom
						- (int) particles.elementAt(i).BoundingRectTop;
				int width = (int) particles.elementAt(i).BoundingRectRight
						- (int) particles.elementAt(i).BoundingRectLeft;
				rect = new NIVision.Rect(topBound, leftBound, height, width);
				// (int)particles.elementAt(0).BoundingRectLeft
				if (i == maxIndex) {
					NIVision.imaqDrawShapeOnImage(frame, frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
					SmartDashboard.putNumber("frameHeight", height);
					SmartDashboard.putNumber("frameWidth", width);
					SmartDashboard.putNumber("frameLeftBound", leftBound);
					SmartDashboard.putNumber("frameTopBound", topBound);

					modHeight = 480 - (topBound + height);
					leftDistance = leftBound;
					rightDistance = 640 - (leftBound + width);
					SmartDashboard.putNumber("modHeight", modHeight);
					SmartDashboard.putNumber("leftDistance", leftDistance);
					SmartDashboard.putNumber("rightDistance", rightDistance);
				} else
					NIVision.imaqDrawShapeOnImage(frame, frame, rect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
			}
			CameraServer.getInstance().setImage(frame);
			//CameraServer.getInstance().setImage(binaryFrame);
			scores.Aspect = AspectScore(particles.elementAt(maxIndex));
			SmartDashboard.putNumber("Aspect", scores.Aspect);
			scores.Area = AreaScore(particles.elementAt(maxIndex));
			SmartDashboard.putNumber("Area", scores.Area);
			// boolean isTote = scores.Aspect > SCORE_MIN && scores.Area >
			// SCORE_MIN;

			// Send distance and tote status to dashboard. The bounding rect,
			// particularly the horizontal center (left - right) may be useful
			// for rotating/driving towards a tote
			// SmartDashboard.putBoolean("IsTote", isTote);
			distance = computeDistance(binaryFrame, particles.elementAt(0));
			SmartDashboard.putNumber("Distance", distance);
			return true;
		} else {
			// SmartDashboard.putBoolean("IsTote", false);
			CameraServer.getInstance().setImage(frame);
			//CameraServer.getInstance().setImage(binaryFrame);
			return false;
		}
	}

	/**
	 * Gets the distance from the right side of the frame to the target and
	 * takes away the distance from the left side of the frame to the target.<br>
	 * This tells you whether or not the target is centered
	 * @return right distance minus left distance
	 */
	public double getLeftRightDistance() {
		double loopTimes = 1;
		double tempLeft = 0, tempRight = 0;
		for (int i = 0; i < loopTimes; i++) {
			if (lookForTarget()) {
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

	/**
	 * Keeps data on a particle, including a rectangle the bounds it
	 */
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport> {
		double PercentAreaToImageArea;
		double Area;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectRight;
		double BoundingRectBottom;

		public int compareTo(ParticleReport r) {
			return (int) (r.Area - this.Area);
		}

		public int compare(ParticleReport r1, ParticleReport r2) {
			return (int) (r1.Area - r2.Area);
		}
	};

	/**
	 * Structure to represent the scores for the various tests used for target
	 * identification
	 */
	public class Scores {
		double Area;
		double Aspect;
	};

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
