package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

public class CompRobot{

	public CompMecanumDrive drive;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public CompRobot(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		drive = new CompMecanumDrive(map);

		telemetry.addData("Robot", "Initialized");
		telemetry.update();

		packet.put("Robot", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}

}
