package org.firstinspires.ftc.teamcode.Controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.ArrayList;
import java.util.Arrays;


public class DriverController {

	ArrayList<ButtonReader> buttons = new ArrayList<ButtonReader>();
	ArrayList<TriggerReader> triggers = new ArrayList<TriggerReader>();

	public ButtonReader halfSpeedButton; // Y
	public ButtonReader fullSpeedButton; // A
	public ButtonReader threeFourthSpeedButton; // B
	public ButtonReader switchDirectionsButton; // X

	public DriverController(GamepadEx gamepadEx2){

		this.halfSpeedButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.Y);
		this.buttons.add(this.halfSpeedButton);

		this.fullSpeedButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.A);
		this.buttons.add(this.fullSpeedButton);

		this.threeFourthSpeedButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.B);
		this.buttons.add(this.threeFourthSpeedButton);

		this.switchDirectionsButton = new ButtonReader(gamepadEx2, GamepadKeys.Button.X);
		this.buttons.add(this.switchDirectionsButton);


	}
		public void readButtons() {
			for(ButtonReader button: this.buttons) {
				button.readValue();
			}
			for(TriggerReader trigger: this.triggers) {
				trigger.readValue();
			}


	}

}
