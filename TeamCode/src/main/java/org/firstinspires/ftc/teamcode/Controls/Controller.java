package org.firstinspires.ftc.teamcode.Controls;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//call back to was just pressed. configure controllers
public class Controller{

		public enum Controllers{
			TestControlScheme,
			ControlScheme2
		}

		private GamepadEx gamepadEx1;
	//need to figure out left trigger
		public ButtonReader clawButton; // button A
		public ButtonReader flipUpButton; // button B
	    public ButtonReader flipDownButton; //button X
	    public ButtonReader flipMiddleButton; // button Y

		public ButtonReader liftUpButton; //D-pad Up
		public ButtonReader liftDownButton; //D-pad Down

		public ButtonReader swingFrontButton; //D-Pad Left
		public ButtonReader swingBackButton; //D-Pad Right

		public ButtonReader liftMoveButton; //  right trigger

		public Controller(GamepadEx gamepadEx1){
			clawButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.A
			);

			flipUpButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.B
			);
			flipDownButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.X
			);
			flipMiddleButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.Y
			);
			liftUpButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_UP
			);
			liftDownButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_DOWN
			);
			swingFrontButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_LEFT
			);
			swingBackButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_RIGHT
			);
			liftMoveButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER
			);
		}

		//Renames all of the buttons
		void buttonMapping() {
			clawButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.A
			);

			flipUpButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.B
			);
			flipDownButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.X
			);
			flipMiddleButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.Y
			);
			liftUpButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_UP
			);
			liftDownButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_DOWN
			);
			swingFrontButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_LEFT
			);
			swingBackButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.DPAD_RIGHT
			);
			liftMoveButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.RIGHT_BUMPER
					);
		}

	/*	void controlScheme2() {
			testButton = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.A
			);

			testButton2 = new ButtonReader(
					gamepadEx1, GamepadKeys.Button.B
			);
		} */

		public void readButtons() {
			gamepadEx1.readButtons();
		}
	}