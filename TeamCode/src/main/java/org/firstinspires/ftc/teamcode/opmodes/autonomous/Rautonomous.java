package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.roadrunner.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.robot.CompRobot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Rautonomous", group = "test")
public class Rautonomous extends LinearOpMode{
//TODO: CONFIGURE THE DRIVER HUB FOR TWO COLOR SENSORS
	//TODO: DON'T FORGET THE ABOVE LINE
	//TODO: BUT ACTUALLY, DON'T FORGET TO DO THIS
	//TODO: YOU DID THIS RIGHT?
	//TODO: CONFIGURE THE DRIVER HUB FOR TWO COLOR SENSORS
	CompRobot robot;
	boolean robotInitFlag = false;
	int counter = 0;

	Pose2d poseEstimate;
	String coneColor1;
	String coneColor2;
	Trajectory moveToCone;
	Trajectory ditchCone;
	Trajectory moveToSpotOne;
	Trajectory moveToSpotTwo;
	Trajectory backUp;
	Trajectory moveToSpotThree;
	Trajectory creep;
	Trajectory moveLeftToJunction;
	Trajectory moveForwardToJunction;
	Trajectory moveRightFromJunction;
	Trajectory moveBackFromJunction;

	enum TrajectoryState{
		MOVE_TO_CONE,
		READ_CONE,
		DITCH_CONE,
		MOVE_TO_SPOT_ONE,
		MOVE_TO_SPOT_TWO,
		MOVE_TO_SPOT_THREE,
		DROP_CONE,
		IDLE
	}

	TrajectoryState trajectoryState = TrajectoryState.MOVE_TO_CONE;
	TrajectoryState desiredState;
	TelemetryPacket packet = new TelemetryPacket();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	Pose2d currPose = new Pose2d(0, 0, Math.toRadians(0.0));

	@Override
	public void runOpMode() throws InterruptedException{
		robot = new CompRobot(hardwareMap, telemetry);

		// Define our start pose
		robot.drive.setPoseEstimate(currPose);

		// Initial trajectory
		moveToCone = robot.drive.trajectoryBuilder(currPose)
				.back(14)
				.build();

		/*Pre-Start/Post-Init Loop*/
		while (!opModeIsActive()){
			telemetry.addData("Robot", "Initialized");
			packet.put("Robot", "Initialized");

			telemetry.update();
			dashboard.sendTelemetryPacket(packet);
		}

		while (opModeIsActive()){
			if (robotInitFlag == false){
				robot.robotInit();
				robotInitFlag = true;
			}
			robot.drive.update();
			telemetry.update();
			poseEstimate = robot.drive.getPoseEstimate();
			PoseStorage.currentPose = poseEstimate;

			switch (this.trajectoryState){
				case MOVE_TO_CONE:
					robot.drive.followTrajectory(moveToCone);
					this.trajectoryState = TrajectoryState.READ_CONE;
					break;

				case READ_CONE:
					coneColor1 = robot.colorSensor1.getConeColor();
					coneColor2 = robot.colorSensor2.getConeColor();
					if (coneColor1 == "Fuchsia" || coneColor2 == "Fuchsia"){
						// Move lift NOW since it happens in parallel
						robot.lift.setNextLevel(robot.lift.highTerminal);
						robot.lift.moveLift();
						desiredState = TrajectoryState.MOVE_TO_SPOT_ONE;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					else if (coneColor1 == "Cyan" || coneColor2 == "Cyan"){
						// Move lift NOW since it happens in parallel
						robot.lift.setNextLevel(robot.lift.highTerminal);
						robot.lift.moveLift();
						desiredState = TrajectoryState.MOVE_TO_SPOT_TWO;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					else if (coneColor1 == "Yellow" || coneColor2 == "Yellow"){
						// Move lift NOW since it happens in parallel
						robot.lift.setNextLevel(robot.lift.lowTerminal);
						robot.lift.moveLift();
						desiredState = TrajectoryState.MOVE_TO_SPOT_THREE;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					else {
						creep = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(1).build();
						robot.drive.followTrajectory(creep);
						counter++;
						if(counter >= 7){
							desiredState=TrajectoryState.MOVE_TO_SPOT_ONE;
							this.trajectoryState = TrajectoryState.DITCH_CONE;
						}
					}
					break;

				case DITCH_CONE:
					ditchCone = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(11).build();
					robot.drive.followTrajectory(ditchCone);
					this.trajectoryState = desiredState;
					break;

				case MOVE_TO_SPOT_ONE:
					moveToSpotOne = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeRight(22).build();
					robot.drive.followTrajectory(moveToSpotOne);
					// AFTER SPOT ONE GO TO HIGH TERMINAL
					this.trajectoryState = TrajectoryState.DROP_CONE;
					break;

				case MOVE_TO_SPOT_TWO:
					moveToSpotTwo = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(28).build();
					robot.drive.followTrajectory(moveToSpotTwo);
					backUp = robot.drive.trajectoryBuilder(moveToSpotTwo.end()).forward(4).build();
					robot.drive.followTrajectory(backUp);
					// AFTER SPOT TWO GO TO HIGH TERMINAL
					this.trajectoryState = TrajectoryState.DROP_CONE;
					break;

				case MOVE_TO_SPOT_THREE:
					moveToSpotThree = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(22).build();
					robot.drive.followTrajectory(moveToSpotThree);
					// AFTER SPOT THREE GO TO LOW TERMINAL
					this.trajectoryState = TrajectoryState.DROP_CONE;
					break;

				case DROP_CONE:
					// Turn robot around
					robot.drive.turn(Math.PI);
					// Strafe to junction
					while(robot.drive.isBusy()){}
					moveLeftToJunction = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(12).build();
					robot.drive.followTrajectory(moveLeftToJunction);
					while(robot.drive.isBusy()){}
					moveForwardToJunction = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).forward(5.25).build();
					robot.drive.followTrajectory(moveForwardToJunction);
					// Lower lift a touch
					while(robot.drive.isBusy()){}
					robot.lift.setNextLevel((int) robot.lift.getCurrentPosition() - 1);
					robot.lift.moveLift();
					// Drop cone
					robot.claw.OpenPosition();
					// Raise lift a touch
					robot.lift.setNextLevel((int) robot.lift.getCurrentPosition() + 1);
					robot.lift.moveLift();
					// Move to parking spot
					moveBackFromJunction = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(5.25).build();
					robot.drive.followTrajectory(moveBackFromJunction);
					while(robot.drive.isBusy()){}
					moveRightFromJunction = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeRight(12).build();
					robot.drive.followTrajectory(moveRightFromJunction);
					while(robot.drive.isBusy()){}
					// Reset robot state
					robot.lift.setNextLevel(robot.lift.active);
					robot.lift.moveLift();
					robot.claw.ClosePosition();
					break;

				case IDLE:
					break;
			}
		}
	}
}





