/*
* Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package teamcode.autocommands;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import teamcode.FtcAuto;
import teamcode.Robot;
import teamcode.RobotParams;

public class CmdAutoHigh implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoHigh";
    private static final boolean visionAssist = true;
    private static final boolean debugCycleTask = false;

    private enum State
    {
        START_DELAY,
        DRIVE_TO_SCORE_POSITION,
        AUTO_SCORE_CONE,
        GO_TO_CONE_STACK,
        AUTO_PICKUP_CONE,
        BACK_TO_SCORE_POSITION,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int signalPos = 0;
    private int conesRemaining = 5;
    private double scanPower = RobotParams.TURRET_SCAN_POWER;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    public CmdAutoHigh(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START_DELAY);
    }   //CmdAutoFarCarousel

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        robot.robotDrive.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(1, "State: disabled or waiting...");
        }
        else
        {
            String msg;
            double turretStartPos;

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    if (debugCycleTask)
                    {
                        robot.robotDrive.driveBase.setFieldPosition(
                            new TrcPose2D(-1 * RobotParams.FULL_TILE_INCHES, -0.5 * RobotParams.FULL_TILE_INCHES, 270));
                    }
                    else
                    {
                        robot.robotDrive.setAutoStartPosition(autoChoices);
                    }
                    // Call vision at the beginning to figure out the signal position.
                    if (robot.vision != null)
                    {
                        signalPos = robot.vision.getLastSignal();
                        if (signalPos > 0)
                        {
                            msg = "Signal found at position " + signalPos;
                            robot.globalTracer.traceInfo(moduleName, msg);
                            robot.speak(msg);
                        }
                    }

                    if (signalPos == 0)
                    {
                        //
                        // We still can't see the signal, set to default position.
                        //
                        signalPos = 2;
                        msg = "No signal found, default to position " + signalPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
                    }
                    //
                    // Do start delay if any.
                    //
                    if (autoChoices.startDelay == 0.0)
                    {
                        //
                        // Intentionally falling through to the next state.
                        //
                        sm.setState(State.DRIVE_TO_SCORE_POSITION);
                    }
                    else
                    {
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_SCORE_POSITION);
                        break;
                    }

                case DRIVE_TO_SCORE_POSITION:
                    if (debugCycleTask)
                    {
                        sm.setState(State.AUTO_PICKUP_CONE);
                    }
                    else
                    {
                        // Prepare all subsystems for pre-conditions of autoScoreCone: arm up, turret at start scan
                        // position.
                        // This operation takes about 5 sec.
                        robot.arm.setPosition(RobotParams.ARM_UP_POS, false);
                        turretStartPos = autoChoices.startPos == FtcAuto.StartPos.LEFT?
                            RobotParams.TURRET_RIGHT: RobotParams.TURRET_LEFT;
                        robot.turret.setTarget(turretStartPos - RobotParams.TURRET_SCAN_OFFSET, true, 0.8, null, 0.0);
                        if (autoChoices.alliance == FtcAuto.Alliance.BLUE_ALLIANCE &&
                            autoChoices.startPos == FtcAuto.StartPos.LEFT)
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.pathPoint(0.6, 2.5, 180.0),
                                robot.robotDrive.pathPoint(0.5, 1.0, 180.0),
                                robot.robotDrive.pathPoint(1.0, 0.5, 90.0));
                        }
                        else
                        {
                            robot.robotDrive.purePursuitDrive.start(
                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.getAutoTargetPoint(-0.6, -2.5, 0.0, autoChoices),
                                robot.robotDrive.getAutoTargetPoint(-0.6, -0.75, -10.0, autoChoices),
                                robot.robotDrive.getAutoTargetPoint(-1.05, -0.55, -90, autoChoices));
                        }
                        sm.waitForSingleEvent(
                            event,
                            autoChoices.strategy == FtcAuto.AutoStrategy.PARKING_ONLY ?
                                State.PARK: State.AUTO_SCORE_CONE);
                    }
                    break;

                case AUTO_SCORE_CONE:
                    turretStartPos =
                        autoChoices.startPos == FtcAuto.StartPos.LEFT?
                            RobotParams.TURRET_RIGHT: RobotParams.TURRET_LEFT;
                    robot.scoreConeTask.autoAssistScoreCone(
                        0.0, 0.0, turretStartPos, RobotParams.HIGH_JUNCTION_SCORING_HEIGHT, scanPower,
                        RobotParams.TURRET_SCAN_DURATION, event);
                    sm.waitForSingleEvent(event, State.GO_TO_CONE_STACK);
                    break;

                case GO_TO_CONE_STACK:
                    // Setup preconditions for auto-pickup: arm at top cone, elevator down, turret front.
                    // This operation takes about 3 sec.
                    robot.arm.setPosition(RobotParams.ARM_PICKUP_PRESETS[5]);
                    robot.elevator.setPosition(RobotParams.ELEVATOR_MIN_POS);
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, true, 0.8, null, 0.0);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(
                            RobotParams.LOOK_FOR_CONE_POS_REDLEFT, FtcAuto.autoChoices));
                    sm.waitForSingleEvent(event, State.AUTO_PICKUP_CONE);
                    break;

                case AUTO_PICKUP_CONE:
                    if (30.0 - TrcTimer.getModeElapsedTime() < RobotParams.CYCLE_TIME + RobotParams.PARK_TIME)
                    {
                        sm.setState(State.PARK);
                    }
                    else
                    {
                        robot.pickupConeTask.autoAssistPickupCone(
                            conesRemaining,
                            visionAssist && robot.vision != null && robot.vision.eocvVision != null,
                            event);
                        conesRemaining--;
                        sm.waitForSingleEvent(event, State.BACK_TO_SCORE_POSITION);
                    }
                    break;

                case BACK_TO_SCORE_POSITION:
                    if (autoChoices.startPos == FtcAuto.StartPos.LEFT)
                    {
                        // Set startPos past 90 and scan backward.
                        turretStartPos = RobotParams.TURRET_RIGHT + RobotParams.TURRET_SCAN_OFFSET;
                        scanPower = -0.2;//RobotParams.TURRET_SCAN_POWER;
                    }
                    else
                    {
                        // Set startPos before 270 and scan forward.
                        turretStartPos = RobotParams.TURRET_LEFT - RobotParams.TURRET_SCAN_OFFSET;
                        scanPower = 0.2;//RobotParams.TURRET_SCAN_POWER;
                    }
                    robot.turret.setTarget(turretStartPos, true, 0.8, null, 0.0);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(-1.05, -0.55, -90, autoChoices));
                    sm.waitForSingleEvent(event, State.AUTO_SCORE_CONE);
                    break;

                case PARK:
                    robot.arm.setPosition(RobotParams.ARM_UP_POS);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        // Park positions are not mirrored between left and right start positions so we need to adjust
                        // it for getAutoTargetPoint because it expects mirror positions.
                        int parkPosIndex = autoChoices.startPos == FtcAuto.StartPos.LEFT?
                            (signalPos - 1): (3 - signalPos);
                        TrcPose2D parkPos = autoChoices.parking == FtcAuto.Parking.FAR_TILE?
                            RobotParams.PARKPOS_REDLEFT_FAR[parkPosIndex]:
                            RobotParams.PARKPOS_REDLEFT_NEAR[parkPosIndex];
                        // Calling PurePursuit to get to the parking position will only work for FAR_TILE.
                        // For NEAR_TILE, this will hit junctions on its path. We decided not to deal with it
                        // because we always choose FAR_TILE. There is no reason to park near. If we decide
                        // to park near, we need to use gridDrive.driveToEndPoint to avoid obstacles (like the
                        // commented lines below). But after cycling, odometry may be off a little and this has
                        // the risk of not parked exactly within the tile. Samuel decided to just use PurePursuit.
                        // But arguably, PurePursuit also uses odometry, so this is not a valid argument. The fact
                        // that park FAR is a straight shot and park NEAR requires a turn is probably the reason
                        // of the inaccuracy. Therefore, I think forgoing park NEAR is a good decision but whether
                        // using PurePursuit or GridDrive should not affect the outcome. Didn't test this theory
                        // because this is a change during competition but I believe GridDrive should have worked.
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(parkPos, autoChoices));
//                        // PurePursuitDrive may hit obstacles especially parking at NEAR_TILE, so call
//                        // gridDrive.driveToEndPoint instead. It will avoid obstacles.
//                        robot.robotDrive.gridDrive.driveToEndPoint(
//                            robot.robotDrive.getAutoTargetPoint(parkPos, autoChoices));

                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DONE:
                default:
                    robot.arm.setPosition(RobotParams.ARM_UP_POS);
                    robot.elevator.setPosition(RobotParams.ELEVATOR_MIN_POS);
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), sm.getState(), robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoHigh
