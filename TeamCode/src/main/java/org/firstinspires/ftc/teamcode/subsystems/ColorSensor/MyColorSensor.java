package org.firstinspires.ftc.teamcode.subsystems.ColorSensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyColorSensor{
	// ColorSensor
	ColorSensor colorSensor;

	// Constants
	int colorSensorSamples = 5;
	public double red;
	public double blue;
	public double green;

	// Constructor
	public MyColorSensor(HardwareMap map, String str){
		colorSensor = map.get(ColorSensor.class, str);
	}


	// Get cone color
	// Inputs: None
	// Outputs: Returns 1 for fuchsia, 2 for cyan, 3 for yellow, 4 if unreadable
	public String getConeColor() {
		// Get norms
		getNorms();
		// Compare values to fuchsia cone
		if (isFuchsia(this.red, this.green, this.blue)){
			return "Fuchsia";
		}
		// Compare values to cyan cone
		else if (isCyan(this.red, this.green, this.blue)){
			return "Cyan";
		}
		// Compare values to yellow cone
		else if (isYellow(this.red, this.green, this.blue)){
			return "Yellow";
		}
		else {
			return "Unknown";
		}
	}

	//color sensor for seeing red cone, using a range of RGB values focusing on red and green
	private boolean isFuchsia(double r, double g, double b) {

		// initialize all check variables to false
		boolean redCheck = false;
		boolean greenCheck = false;
		boolean blueCheck = false;

		// if r is in the range
		if(r >= 0.7 && r <= 1.0) {
			redCheck=true;
		}

		// if g is in the range
		if(g >= 0.4 && g <= 0.8) {
			greenCheck=true;
		}

		// if b is in the range
		if(b >= 0.5 && b <= 0.9) {
			blueCheck=true;
		}

		// if all color checks are true, return true
		// otherwise, return false
		if(redCheck && greenCheck && blueCheck){
			return true;
		}
		else return false;
	}
	private boolean isCyan(double r, double g, double b) {

		// initialize all check variables to false
		boolean redCheck = false;
		boolean greenCheck = false;
		boolean blueCheck = false;

		// if r is in the range
		if(r >= 0.0 && r <= 0.4) {
			redCheck=true;
		}

		// if g is in the range
		if(g >= 0.3 && g <= 0.8) {
			greenCheck=true;
		}

		// if b is in the range
		if(b >= 0.7 && b <= 1.0) {
			blueCheck=true;
		}

		// if all color checks are true, return true
		// otherwise, return false
		if(redCheck && greenCheck && blueCheck){
			return true;
		}
		else return false;
	}
	private boolean isYellow(double r, double g, double b) {

		// initialize all check variables to false
		boolean redCheck = false;
		boolean greenCheck = false;
		boolean blueCheck = false;


		// if r is in the range
		if(r >= 0.4 && r <= 0.8) {
			redCheck=true;
		}

		// if g is in the range
		if(g >= 0.7 && g <= 1.0) {
			greenCheck=true;
		}

		// if b is in the range
		if(b >= 0.3 && b <= 0.6){
			blueCheck = true;
		}

		// if all color checks are true, return true
		// otherwise, return false
		if(redCheck && greenCheck && blueCheck){
			return true;
		}
		else return false;
	}

	// Updates this.red, this.blue, this.green values with color sensor
	// Inputs: None
	// Outputs: None
	private void getNorms() {
		// Get red average
		double redAvg = redAverage();
		// Get blue average
		double blueAvg = blueAverage();
		// Get green average
		double greenAvg = greenAverage();
		// Find max of red, blue, and green averages
		double maxAvg = Math.max(Math.max(redAvg, blueAvg), greenAvg);
		// Update this.red to be red average divided by max average
		this.red = redAvg/maxAvg;
		// Update this.blue to be blue average divided by max average
		this.blue = blueAvg/maxAvg;
		// Update this.green to be green average divided by max average
		this.green = greenAvg/maxAvg;
	}


	private double redAverage() {
		int redSum = 0;
		// run for colorSensorSamples iterations
		for (int i = 0; i < colorSensorSamples; i++) {
			// increment redSum by our new red color sensor reading
			redSum += this.colorSensor.red();
		}

		// return the average red value (redSum divided by the number of samples we take)
		return (double) redSum / (double) colorSensorSamples;
	}


	private double greenAverage() {
		// initialize greenSum to 0
		int greenSum = 0;

		// run for colorSensorSamples iterations
		for (int i = 0; i < colorSensorSamples; i++) {
			// increment greenSum by our new green color sensor reading
			greenSum += this.colorSensor.green();
		}

		// return the average green value (greenSum divided by the number of samples we take)
		return (double) greenSum / (double) colorSensorSamples;
	}

	private double blueAverage() {
		// initialize blueSum to 0
		int blueSum = 0;

		// run for colorSensorSamples iterations
		for (int i = 0; i < colorSensorSamples; i++) {
			// increment blueSum by our new blue color sensor reading
			blueSum += this.colorSensor.blue();
		}

		// return the average blue value (blueSum divided by the number of samples we take)
		return (double) blueSum / (double) colorSensorSamples;
	}


	private double alphaAverage() {
		// initialize alphaSum to 0
		int alphaSum = 0;

		// run for NUM_SAMPLES iterations
		for (int i = 0; i < colorSensorSamples; i++) {
			// increment alphaSum by our new alpha color sensor reading
			alphaSum += this.colorSensor.alpha();
		}

		// return the average alpha value (alphaSum divided by the number of samples we take)
		return (double) alphaSum / (double) colorSensorSamples;
	}

}
