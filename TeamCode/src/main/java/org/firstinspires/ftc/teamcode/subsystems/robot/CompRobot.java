package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flip.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;

public class CompRobot{

	public CompMecanumDrive drive;
	public Lift lift;
	public Flipper flip;


	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public CompRobot(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

		drive = new CompMecanumDrive(map);
		//lift = new Lift(map, telemetry);
		flip = new Flipper(map);

		telemetry.addData("Robot", "Initialized");
		telemetry.update();

		packet.put("Robot", "Initialized");
		dashboard.sendTelemetryPacket(packet);
	}
}
