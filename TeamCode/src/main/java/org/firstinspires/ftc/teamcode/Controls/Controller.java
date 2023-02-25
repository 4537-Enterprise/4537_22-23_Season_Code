package org.firstinspires.ftc.teamcode.Controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.ArrayList;
import java.util.Arrays;

//call back to was just pressed. configure controllers
public class Controller{

		public enum ControlScheme{
			ControlScheme1,
			ControlScheme2
		}
      	ArrayList<ButtonReader> buttons = new ArrayList<ButtonReader>();
		ArrayList<TriggerReader> triggers = new ArrayList<TriggerReader>();

		public ButtonReader clawButton; // button A
		//public ButtonReader flipUpButton; // button
	    public ButtonReader flipDownButton; //button X
	  //  public ButtonReader armPickUpButton; // button Y
		public ButtonReader liftUpButton; //D-pad Up
		public ButtonReader liftDownButton; //D-pad Down
		//public TriggerReader liftMoveTrigger; //  right trigger
		public ButtonReader EmergancyClose; // B button
		//public ButtonReader lowestLevelButton; // Right Bumper
		public ButtonReader Low; //Left Bumper
		public ButtonReader Medium; //Right Bumper
		public TriggerReader Active; //Left Trigger
		public TriggerReader High; //Right Trigger
	//left trigger activd
	//left bumper is low
	// right bumper is medium
	//right trigger is high

		//TODO: Make constructor take second parameter ControlScheme
		public Controller(GamepadEx gamepadEx1){
			// TODO: Implement switch case to switch on ControlScheme. If ControlScheme1, run mapControlScheme1. If ControlScheme2, run mapControlScheme2
			// TODO: Move this to a function
			this.clawButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.A );
			this.buttons.add(this.clawButton);

			this.EmergancyClose = new ButtonReader( gamepadEx1, GamepadKeys.Button.B );
			this.buttons.add(this.EmergancyClose);

			flipDownButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.X );
			this.buttons.add(this.flipDownButton);

//			armPickUpButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.Y );
//			this.buttons.add(this.armPickUpButton);

			liftUpButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_UP );
			this.buttons.add(this.liftUpButton);

			liftDownButton = new ButtonReader( gamepadEx1, GamepadKeys.Button.DPAD_DOWN );
			this.buttons.add(this.liftDownButton);


			Low = new ButtonReader( gamepadEx1, GamepadKeys.Button.LEFT_BUMPER );
			this.buttons.add(this.Low);

			Medium = new ButtonReader( gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER);
			this.buttons.add(this.Medium);

			High = new TriggerReader( gamepadEx1, GamepadKeys.Trigger.RIGHT_TRIGGER );
			this.triggers.add(this.High);

			Active = new TriggerReader( gamepadEx1, GamepadKeys.Trigger.LEFT_TRIGGER );
			this.triggers.add(this.Active);
//			liftMoveTrigger = new TriggerReader(gamepadEx1, GamepadKeys.Trigg
//			er.RIGHT_TRIGGER);
//			this.triggers.add (this.liftMoveTrigger);
//
//			lowestLevelButton = new ButtonReader (gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER);
//			this.buttons.add(this.lowestLevelButton);


		}



		//TODO: Make a function that takes in a gamepadEx and maps the buttons to pre-named buttonreaders

		public void readButtons() {
			for(ButtonReader button: this.buttons) {
				button.readValue();
			}
			for(TriggerReader trigger: this.triggers) {
				trigger.readValue();
			}
		}

	}