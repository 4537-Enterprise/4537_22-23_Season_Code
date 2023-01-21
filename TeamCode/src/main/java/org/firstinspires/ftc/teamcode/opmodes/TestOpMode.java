package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Claw.LimitSwitch;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;
import org.firstinspires.ftc.teamcode.Controls.Controller;
import org.firstinspires.ftc.teamcode.Controls.DriverController;


@TeleOp(name = "TestOpMode")
public class TestOpMode extends LinearOpMode{
	CompRobot robot;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	@Override
	public void runOpMode() throws InterruptedException{

		while (!opModeIsActive()){
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

//		while (opModeIsActive()){
//			if (robot.limitSwitch.limitSwitchOn == true) {
//				telemetry.addLine ("Claw Close");
//			}
//		}
	}

}
