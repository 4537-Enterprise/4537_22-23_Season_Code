package org.firstinspires.ftc.teamcode.Controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.ArrayList;

//call back to was just pressed. configure controllers
public class Controller{

		public enum ControlScheme{
			ControlScheme1,
			ControlScheme2
		}
      ArrayList<ButtonReader> buttons = new ArrayList<ButtonReader>();

		//TODO: figure out left trigger
		public ButtonReader clawButton; // button A
		public ButtonReader flipUpButton; // button B
	    public ButtonReader flipDownButton; //button X
	    public ButtonReader flipMiddleButton; // button Y
		public ButtonReader liftUpButton; //D-pad Up
		public ButtonReader liftDownButton; //D-pad Down
		public ButtonReader swingFrontButton; //D-Pad Left
		public ButtonReader swingBackButton; //D-Pad Right
		public ButtonReader liftMoveButton; //  right trigger
		public ButtonReader EmergancyClose;

		// TODO: Make constructor take second parameter ControlScheme
		public Controller(GamepadEx gamepadEx1){
			// TODO: Implement switch case to switch on ControlScheme. If ControlScheme1, run mapControlScheme1. If ControlScheme2, run mapControlScheme2
			// TODO: Move this to a function
			this.clawButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.A );
			this.buttons.add(this.clawButton);

			flipUpButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.B );
			this.buttons.add(this.flipUpButton);

			flipDownButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.X );
			this.buttons.add(this.flipDownButton);

			flipMiddleButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.Y );
			this.buttons.add(this.flipMiddleButton);

			liftUpButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_UP );
			this.buttons.add(this.liftUpButton);

			liftDownButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_DOWN );
			this.buttons.add(this.liftDownButton);

			swingFrontButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_LEFT );
			this.buttons.add(this.swingFrontButton);

			swingBackButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_RIGHT );
			this.buttons.add(this.swingBackButton);

			liftMoveButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER );
			this.buttons.add(this.liftMoveButton);

			EmergancyClose = new ButtonReader (gamepadEx1, GamepadKeys.Button.LEFT_BUMPER);
			this.buttons.add(this.EmergancyClose);
		}

		//TODO: Make a function that takes in a gamepadEx and maps the buttons to pre-named buttonreaders

		public void readButtons() {
			for(ButtonReader button: this.buttons) {
				button.readValue();
			}
		}

	}