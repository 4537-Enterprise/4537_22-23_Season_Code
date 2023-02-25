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
	Trajectory moveToTerminal;
	Trajectory moveFromTerminal;

	enum TrajectoryState{
		MOVE_TO_CONE,
		READ_CONE,
		DITCH_CONE,
		MOVE_TO_SPOT_ONE,
		MOVE_TO_SPOT_TWO,
		MOVE_TO_SPOT_THREE,
		LOW_TERMINAL,
		HIGH_TERMINAL,
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
						desiredState = TrajectoryState.MOVE_TO_SPOT_ONE;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					else if (coneColor1 == "Cyan" || coneColor2 == "Cyan"){
						desiredState = TrajectoryState.MOVE_TO_SPOT_TWO;
						this.trajectoryState = TrajectoryState.DITCH_CONE;
					}
					else if (coneColor1 == "Yellow" || coneColor2 == "Yellow"){
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
					this.trajectoryState = TrajectoryState.HIGH_TERMINAL;
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case MOVE_TO_SPOT_TWO:
					moveToSpotTwo = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).back(28).build();
					robot.drive.followTrajectory(moveToSpotTwo);
					backUp = robot.drive.trajectoryBuilder(moveToSpotTwo.end()).forward(4).build();
					robot.drive.followTrajectory(backUp);
					// AFTER SPOT TWO GO TO HIGH TERMINAL
					this.trajectoryState = TrajectoryState.HIGH_TERMINAL;
					break;

				case MOVE_TO_SPOT_THREE:
					moveToSpotThree = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate()).strafeLeft(22).build();
					robot.drive.followTrajectory(moveToSpotThree);
					// AFTER SPOT THREE GO TO LOW TERMINAL
					this.trajectoryState = TrajectoryState.LOW_TERMINAL;
					break;

  					case LOW_TERMINAL:
//					// TODO: DOUBLE CHECK TO MAKE SURE THIS -135 IS COUNTER-CLOCKWISE
//					// TODO: OTHERWISE THIS NEEDS TO BE 135
     				robot.drive.turn(-135);
//					// Set lift and arm before we move
                	robot.lift.setNextLevel(robot.lift.lowTerminal);
					robot.lift.moveLift();
//					// TODO: DOUBLE-CHECK THIS MEASUREMENT
					 moveToTerminal = robot.drive.trajectoryBuilder(currPose).forward(9.5).build();
					 robot.drive.followTrajectory(moveToTerminal);
//					// Drop cone
				robot.claw.OpenPosition();
//					// NOTE: Keep claw open to make grabbing next cone in Tele Op easier
					moveFromTerminal = robot.drive.trajectoryBuilder(currPose).back(9.5).build();
					robot.drive.followTrajectory(moveFromTerminal);
//					// Reset robot to initial state
					robot.lift.setNextLevel(robot.lift.active);
					robot.lift.moveLift();
//					// TODO: DOUBLE CHECK TO MAKE SURE THIS ROTATES THE CORRECT DIRECTION
//					// TODO: OTHERWISE THIS NEEDS TO BE 45
					robot.drive.turn(-45);
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case HIGH_TERMINAL:
//					// TODO: DOUBLE CHECK TO MAKE SURE THIS -135 IS CLOCKWISE
//					// TODO: OTHERWISE THIS NEEDS TO BE 135
					robot.drive.turn(Math.toRadians(-135));
//					// Set lift and arm before we move
					robot.lift.setNextLevel(robot.lift.highTerminal);
					robot.lift.moveLift();
//					// TODO: DOUBLE-CHECK THIS MEASUREMENT
					moveToTerminal = robot.drive.trajectoryBuilder(currPose).back(9.5).build();
					robot.drive.followTrajectory(moveToTerminal);
//					// Drop cone
					robot.claw.OpenPosition();
//					// NOTE: Keep claw open to make grabbing next cone in Tele Op easier
					moveFromTerminal = robot.drive.trajectoryBuilder(currPose).forward(9.5).build();
					robot.drive.followTrajectory(moveFromTerminal);
//					// Reset robot to initial state
					robot.lift.setNextLevel(robot.lift.active);
					robot.lift.moveLift();
//					// TODO: DOUBLE CHECK TO MAKE SURE THIS ROTATES THE CORRECT DIRECTION
//					// TODO: OTHERWISE THIS NEEDS TO BE 45
					robot.drive.turn(-45);
					this.trajectoryState = TrajectoryState.IDLE;
					break;

				case IDLE:
					break;
			}
		}
	}
}





