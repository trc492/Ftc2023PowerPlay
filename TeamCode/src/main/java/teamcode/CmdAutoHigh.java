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

package teamcode;

import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTimer;
import TrcCommonLib.trclib.TrcUtil;

class CmdAutoHigh implements TrcRobot.RobotCommand
{
    private static final String moduleName = "CmdAutoHigh";

    private enum State
    {
        TEST_ALIGN_TO_POLE,
        START_DELAY,
        DRIVE_TO_SCORE_POSITION,
        RAISE_ELEVATOR_TO_SCORE,
        TURN_TO_SCORE_PRELOAD,
        SCORE_PRELOAD_WITH_VISION,
        SCORE_PRELOAD,
        DO_CYCLE,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final FtcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private int signalPos = 0;
    // Tells us number cones left on the conestack.
    private int conesRemaining = 5;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies all the choices from the autonomous menus.
     */
    CmdAutoHigh(Robot robot, FtcAuto.AutoChoices autoChoices)
    {
        robot.globalTracer.traceInfo(moduleName, ">>> robot=%s, choices=%s", robot, autoChoices);

        this.robot = robot;
        this.autoChoices = autoChoices;
        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        robot.robotDrive.purePursuitDrive.setFastModeEnabled(true);
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

            robot.dashboard.displayPrintf(1, "State: %s", state);
            switch (state)
            {
                case START_DELAY:
                    //
                    // Set robot starting position in the field.
                    //
                    robot.robotDrive.setAutoStartPosition(autoChoices);
                    // Call vision at the beginning to figure out the signal position.
                    if (robot.vision != null)
                    {
                        signalPos = robot.vision.getLastSignal();
                        msg = "Signal found at position " + signalPos;
                        robot.globalTracer.traceInfo(moduleName, msg);
                        robot.speak(msg);
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
                    //drive to score position
                    //raise elevator to scoring height
                    //turn turret to score position
                    //wait for pure pursuit to finish
                    // Todo: add option to do center high poles
                    //Points are 6 inches from the high junction, on the line drawn from the the high junction to the
                    //corresponding cone stack, facing the cone stack
                    robot.robotDrive.purePursuitDrive.start(
                        event, 5.0, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(-0.5, -2.5, 0.0, autoChoices),
                        robot.robotDrive.getAutoTargetPoint(-0.5, -0.85, 0.0, autoChoices),
                        robot.robotDrive.getAutoTargetPoint(-1.0, -0.5, -90.0, autoChoices));
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR_TO_SCORE); //State.PARK);
                    break;

                case RAISE_ELEVATOR_TO_SCORE:

                    robot.arm.setTarget(RobotParams.ARM_SCORE_POS);
                    robot.elevator.setTarget(RobotParams.HIGH_JUNCTION_HEIGHT, true, 1.0, event, null, 5.0);
                    sm.waitForSingleEvent(event, State.TURN_TO_SCORE_PRELOAD);
                    break;

                case TURN_TO_SCORE_PRELOAD:
                    robot.elevator.setTarget(robot.elevator.getPosition());
                    robot.turret.setTarget(
                        autoChoices.startPos == FtcAuto.StartPos.LEFT?
                            RobotParams.TURRET_RIGHT : RobotParams.TURRET_LEFT,
                        0.9, event, null, 4, null, null);
                    sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    break;

                case SCORE_PRELOAD_WITH_VISION:
                    robot.cyclingTask.scoreCone(TaskCyclingCones.VisionType.CONE_AND_POLE_VISION, event);
                    sm.waitForSingleEvent(event, State.DONE);//PARK);
                    break;
                //dump the cone with auto-assist
                case SCORE_PRELOAD:
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP, event, null, 0.0);
                    sm.waitForSingleEvent(event, State.PARK);//DO_CYCLE);
                    break;

                //if time >= 26 or no more cones on the conestack, go to park
                //otherwise call the doCycle method for each cone on the stack
                case DO_CYCLE:
                    //if driveOnly, just drive back and forth to simulate it
                    if (TrcUtil.getModeElapsedTime() >= 27 || conesRemaining == 0)
                    {
                        sm.setState(State.PARK);
                    }
                    else
                    {
                        robot.cyclingTask.doFullAutoCycle(TaskCyclingCones.VisionType.NO_VISION, conesRemaining, event);
                        conesRemaining--;
                        sm.waitForSingleEvent(event, State.DO_CYCLE);
                    }
                    break;
//                        if (driveOnly)
//                        {
//                            robot.robotDrive.purePursuitDrive.start(
//                                event, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.getAutoTargetPoint(
//                                    RobotParams.CONE_STACK_RED_LEFT.x - RobotParams.TURRET_PICKUP_OFFSET,
//                                    RobotParams.CONE_STACK_RED_LEFT.y, -90.0, autoChoices),
//                                robot.robotDrive.getAutoTargetPoint(-1, -0.5, -90, autoChoices));
//
//                        }
//                        else{
//                            robot.cyclingTask.doFullAutoCycle(TaskCyclingCones.VisionType.NO_VISION, conesRemaining, event);
//                        }
//                    }

                case PARK:
                    if (autoChoices.parking == FtcAuto.Parking.NO_PARKING)
                    {
                        // We are not parking anywhere, just stop and be done.
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robot.elevator.setTarget(7);
                        robot.arm.setTarget(0);
                        robot.turret.setTarget(RobotParams.TURRET_BACK);
                        // CodeReview: check if there are any obstacles in the path.
                        TrcPose2D parkPos =
                            autoChoices.parking == FtcAuto.Parking.NEAR_TILE?
                                RobotParams.PARKPOS_RED_LEFT_NEAR[signalPos - 1]:
                                RobotParams.PARKPOS_RED_LEFT_FAR[signalPos - 1];
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(parkPos.x, parkPos.y, -90, autoChoices));
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    break;

                case DONE:
                default:
                    //
                    // We are done, zero calibrate the arm will lower it.
                    //
                    cancel();
                    break;
            }

            robot.globalTracer.traceStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdAutoHigh
