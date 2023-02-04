package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSwing.ArmSwing;
import org.firstinspires.ftc.teamcode.subsystems.Flip.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.Lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.CompMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor.MyColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Claw.LimitSwitch;

public class CompRobot{

	public CompMecanumDrive drive;
	public Lift lift;
	public Flipper flip;
	public ArmSwing arm;
	public Claw claw;
	public LimitSwitch limitSwitch;
	public MyColorSensor colorSensor1;
	public MyColorSensor colorSensor2;

	Telemetry telemetry;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	public CompRobot(HardwareMap map, Telemetry telemetry) {
		this.telemetry = telemetry;

    	drive = new CompMecanumDrive(map);
		flip = new Flipper(map);
		arm = new ArmSwing(map);
		lift = new Lift(map, telemetry);
    	claw = new Claw(map);
		colorSensor1 = new MyColorSensor(map, "ColorSensor1");
		colorSensor2 = new MyColorSensor(map, "ColorSensor2");
		//limitSwitch = new LimitSwitch(map);
		telemetry.addData("Robot", "Initialized");
		telemetry.update();

		// Close claw at beginning
		this.claw.ClosePosition();

	}
	public void robotInit(){
		packet.put("Robot", "Initialized");
		dashboard.sendTelemetryPacket(packet);
		this.lift.setNextLevel(this.lift.active);
		this.lift.moveLift();}
}
