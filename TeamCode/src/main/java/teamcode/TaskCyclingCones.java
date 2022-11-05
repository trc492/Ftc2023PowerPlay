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

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcNotifier;
import TrcCommonLib.trclib.TrcOpenCvColorBlobPipeline;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;

/**
 * Robot starts in between cone stack and high goal
 * starts on the middle of the line
 * facing the cone stack
 * score one cone and end where it started
 * turret starts facing backwards (Zero Calibrate is in the back)
 * elevator down
 * arm position doesn't matter
 */
public class TaskCyclingCones
{
    private static final String moduleName = "TaskCyclingCones";

    public enum CycleType
    {
        AUTO_FULL_CYCLE, //does a full cycle(pickup and scoring), used for auto
        SCORING_ONLY,//assumes robot already has a cone, scores it onto the pole
        PICKUP_ONLY//picks up a cone from the stack
    }

    public enum VisionType
    {
        NO_VISION,
        CONE_VISION,//uses vision to horizontally align to target
        CONE_AND_POLE_VISION
    }

    /*
     * 1. PREPARE_PICKUP: turn turret to front,
     * 1. drive to the cone stack and raise elevator and set arm to 90
     * 2. spin turret to the front and spin intake wheels
     * 3. lower the elevator to the correct height
     * 4. raise the elevator up higher than the pole
     * 5. turn turret 180 degrees while driving backwards to the pole
     * 6. set arm to 90 (to reset arm pos after ensuring turret will not bonk, might not be needed)
     * 7. spin intake backwards
     * 8. turn turret 180 degrees and lower elevator while driving back to start position
     */
    public enum State
    {
        START,
        LOOK_FOR_CONE,
        DRIVE_TO_CONE,
        PICKUP_CONE,
        RAISE_ELEVATOR,
        DRIVE_TO_POLE,
        LOOK_FOR_POLE,
        ALIGN_TO_POLE,
        SCORE,
        DONE
    }

    private final Robot robot;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject cycleTaskObj;
    private TrcDbgTrace msgTracer = null;
    private CycleType cycleType = CycleType.AUTO_FULL_CYCLE;
    private VisionType visionType = VisionType.CONE_VISION;
    private TrcEvent onFinishEvent = null;
    private TrcNotifier.Receiver onFinishCallback = null;
    private int conesRemaining;
    private Double visionExpireTime = null;
    //location of the cone or pole relative to the robot
    private TrcPose2D targetLocation = null;
    private Double poleAngle;

    public TaskCyclingCones(Robot robot)
    {
        this.robot = robot;
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        cycleTaskObj = TrcTaskMgr.createTask(moduleName, this::cycleTask);
    }   //TaskCyclingCones

    /**
     * This method enables/disables tracing for the auto-assist task.
     *
     * @param tracer specifies the tracer to use for logging events.
     */
    public void setMsgTracer(TrcDbgTrace tracer)
    {
        msgTracer = tracer;
    }   //setMsgTracer

    public void cancel()
    {
        sm.stop();
        cycleTaskObj.unregisterTask();

        if (onFinishEvent != null)
        {
            onFinishEvent.cancel();
            onFinishEvent = null;
        }

        if (onFinishCallback != null)
        {
            onFinishCallback.notify(null);
            onFinishCallback = null;
        }
    }   //cancel

    //doFullAutoCycle pickups and scores 1 cone
    /* Preconditions(AUTO_FULL_CYCLE)
     * robot facing the cone stack with high and mid junction on either side turret position 90 degrees
     * turret position 90 degrees means intake is right on top of the high pole or mid pole (because we just scored a cone)
     * elevator is height of the high pole
     * arm position extended, intake stopped
     */
    public void doFullAutoCycle(VisionType visionType, int conesRemaining, TrcEvent event)
    {
        startCycling(CycleType.AUTO_FULL_CYCLE, visionType, conesRemaining, event, null);
    }

    //teleop call to do pickupOnly
    public void doPickup(VisionType visionType, int conesRemaining, TrcEvent event)
    {
        startCycling(CycleType.PICKUP_ONLY, visionType, conesRemaining, event, null);
    }
    //teleop call to do scoring, only the cone it currently has
    public void scoreCone(VisionType visionType, TrcEvent event)
    {
        startCycling(CycleType.SCORING_ONLY, visionType, 1, event, null);
    }
    //prepare for cycling, start sm
    public void startCycling(
        CycleType cycleType, VisionType visionType, int conesRemaining, TrcEvent event, TrcNotifier.Receiver callback)
    {
        this.cycleType = cycleType;
        this.visionType = visionType;
        this.conesRemaining = conesRemaining;
        this.onFinishEvent = event;
        this.onFinishCallback = callback;
        cycleTaskObj.registerTask(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK);
        sm.start(State.START);
        if (cycleType == CycleType.SCORING_ONLY)
        {
            //sm.start(cycleType == CycleType.SCORING_ONLY? State.ALIGN_TO_POLE: State.ALIGN_TO_CONE);
        }
    }   //startCycling

    //
    // Assumptions: robot is facing the cone stack approximate, the scoring junction is either at the left or right
    //              of the robot.
    // 1. (DONE)PREPARE_PICKUP: turn turret to front, lower elevator to travel height, extend arm. Go to state 2.
    // 2. (DONE)LOOK_FOR_CONE: If using vision, call vision to detect the cone stack location. If using vision and vision returns null, stay in this state to try again unless retry
    //    count reaches zero in which case we will just use the absolute cone stack location. Go to state 3.
    // 3. (DONE)DRIVE_TO_CONE: Either use the vision result or the known cone stack location, navigate the robot to the cone stack. Set
    //    elevator height to the appropriate pickup height.
    // 4. (DONE) PICKUP_CONE: Start auto-assist pickup and lower the elevator onto the cone. Wait for pickup to signal, go to state 5.
    // 5. (DONE) RAISE_ELEVATOR: Raise elevator to possession height. Go to state 6.
    // 6. (DONE) DRIVE_TO_POLE: Navigate robot back to the junction pole. Turn turret toward the scoring junction. Retract arm. Go to state 7.
    // 7. (DONE) ALIGN_TO_POLE: If using vision, call vision to detect the junction pole else assume we are aligned (go to next state). If
    //    vision returns null stay in this state to try again unless retry count reaches zero in which case, just go
    //    to next state. If pole is detected, call pure pursuit event to turn turret towards it, go to next state when done.
    // 8. (DONE) PREPARE_SCORE: Raise elevator to scoring height, extend arm. Go to state 9.
    // 9. SCORE: Auto-assist release cone. Go to DONE.
    //
    private void cycleTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
//            double matchTime = TrcUtil.getModeElapsedTime();

            switch (state)
            {
                case START: //1. drive forward to the cone and prepare turret to pick it up
                    if (visionType != VisionType.NO_VISION)
                    {
                        robot.vision.frontEocvVision.setDetectObjectType(
                            FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE?
                                EocvVision.ObjectType.RED_CONE: EocvVision.ObjectType.BLUE_CONE);
                    }
                    robot.arm.setTarget(RobotParams.ARM_SCORE_POS);
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, 1.0, event, null);
                    // CodeReview: may have to delay elevator until it clears the pole.
                    robot.elevator.setTarget(RobotParams.ELEVATOR_POS_FOR_TURRET_TURN);
                    sm.waitForSingleEvent(event, State.LOOK_FOR_CONE);
                    break;

                case LOOK_FOR_CONE:
                    if (visionType != VisionType.NO_VISION)
                    {
                        // Use vision to find the relative location of the cone.
                        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> coneInfo =
                            robot.vision.getBestDetectedConeInfo();
                        if (coneInfo != null)
                        {
                            targetLocation = new TrcPose2D(
                                coneInfo.distanceFromCamera.x, coneInfo.distanceFromCamera.y,
                                coneInfo.horizontalAngle);
                        }
                    }

                    if (targetLocation != null)
                    {
                        // We found the target with vision, go to the next state.
                        sm.setState(State.DRIVE_TO_CONE);
                    }
                    else if (visionType != VisionType.NO_VISION)
                    {
                        // Vision did not detect anything, try again with timeout.
                        if (visionExpireTime == null)
                        {
                            visionExpireTime = TrcUtil.getCurrentTime() + 0.5;
                        }
                        else if (TrcUtil.getCurrentTime() >= visionExpireTime)
                        {
                            // Times up, reset expireTime, go to next state.
                            visionExpireTime = null;
                            sm.setState(State.DRIVE_TO_CONE);
                        }
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_CONE);
                    }
                    break;

                case DRIVE_TO_CONE:
                    robot.arm.setTarget(RobotParams.ARM_PICKUP_POS);
                    if (targetLocation != null)
                    {
                        // Vision found the cone, drive to it with incremental pure pursuit.
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), true, targetLocation);
                    }
                    else
                    {
                        // Either we did not use vision or vision did not detect anything. Use the absolute cone
                        // stack location.
                        robot.robotDrive.purePursuitDrive.start(
                            event, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT, FtcAuto.autoChoices));
                    }
                    sm.waitForSingleEvent(event, State.PICKUP_CONE);
                    break;

                case PICKUP_CONE: //2. lower elevator to the cone, wait for intake autoAssist
                    robot.elevator.setPresetPosition(conesRemaining);
                    // CodeReview: give it a timeout to prevent hanging.
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_PICKUP, event, null, 0.0);
                    sm.waitForSingleEvent(event, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR: //3 raise the elevator up higher than the pole
                    robot.elevator.setTarget(RobotParams.HIGH_JUNCTION_HEIGHT, true, 1.0, event);
                    sm.waitForSingleEvent(event, State.DRIVE_TO_POLE);
                    break;

                case DRIVE_TO_POLE:
                    // Turn turret to the right side while driving backwards until intake is right above the pole
                    robot.robotDrive.purePursuitDrive.start(
                        event, robot.robotDrive.driveBase.getFieldPosition(), false,
                        robot.robotDrive.getAutoTargetPoint(-1, -0.5, -90, FtcAuto.autoChoices));
                    robot.turret.setTarget(
                        FtcAuto.autoChoices.startPos == FtcAuto.StartPos.LEFT?
                            RobotParams.TURRET_RIGHT: RobotParams.TURRET_LEFT);
                    robot.arm.setTarget(RobotParams.ARM_SCORE_POS, false, 1.0);
                    sm.waitForSingleEvent(
                        event,
                        visionType == VisionType.CONE_AND_POLE_VISION ? State.LOOK_FOR_POLE : State.SCORE);
                    break;

                case LOOK_FOR_POLE:
                    // Call vision to detect the junction pole
                    TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> poleInfo =
                        robot.vision.getBestDetectedPoleInfo();
                    if (poleInfo != null)
                    {
                        poleAngle = poleInfo.distanceFromImageCenter.x * RobotParams.ELEVATORCAM_ANGLE_PER_PIXEL;
                    }
                    else
                    {
                        poleAngle = null;
                    }

                    if (poleAngle != null)
                    {
                        sm.setState(State.ALIGN_TO_POLE);
                    }
                    //if we don't see the target give it another second to keep looking(we haven't set expireTime yet
                    else if (visionExpireTime == null)
                    {
                        visionExpireTime = TrcUtil.getCurrentTime() + 0.5;
                    }
                    else if (TrcUtil.getCurrentTime() >= visionExpireTime)
                    {
                        //times up, reset expireTime, assume it's aligned and score.
                        visionExpireTime = null;
                        sm.setState(State.SCORE);
                    }
                    break;

                case ALIGN_TO_POLE:
                    // Call vision to detect the junction pole
                    robot.turret.setTarget(poleAngle, 1.0, event, null);
                    sm.waitForSingleEvent(event, State.SCORE);
                    break;

                case SCORE: //7. spin intake backwards
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP, event, null, 0.5);
                    sm.waitForSingleEvent(event, State.DONE);
                    break;

                default:
                case DONE:
                    cancel();
                    break;
            }

            if (msgTracer != null)
            {
                msgTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }
    }   //cycleTask

}   //class TaskCyclingCones
