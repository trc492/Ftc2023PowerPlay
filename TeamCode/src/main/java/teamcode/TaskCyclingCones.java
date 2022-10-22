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

import org.checkerframework.checker.units.qual.C;

import TrcCommonLib.trclib.TrcDbgTrace;
import TrcCommonLib.trclib.TrcEvent;
import TrcCommonLib.trclib.TrcNotifier;
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
        ALIGN_VISION//uses vision to horizontally align to target
    }

    /*
     * 1. if using vision, horizontally align to the cone and calculate distance
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
        ALIGN_TO_CONE,
        DETERMINE_DISTANCE_TO_CONE,
        PREPARE_PICKUP,
        START_SPIN,
        PICKUP_CONE,
        RAISE_ELEVATOR,
        PREPARE_SCORE,
        ALIGN_ARM,
        SCORE,
        RESET,
        DONE
    }

    private final Robot robot;
    private FtcAuto.AutoChoices autoChoices;
    private final TrcStateMachine<State> sm;
    private final TrcTaskMgr.TaskObject cycleTaskObj;
    private TrcDbgTrace msgTracer = null;
    private CycleType cycleType = CycleType.AUTO_FULL_CYCLE;
    private VisionType visionType = VisionType.ALIGN_VISION;
    private TrcEvent onFinishEvent = null;
    private TrcNotifier.Receiver onFinishCallback = null;
    private final TrcEvent driveEvent;
    private final TrcEvent elevatorEvent;
    private final TrcEvent armEvent;
    private final TrcEvent turretEvent;
    private final TrcEvent intakeEvent;
    private int conesRemaining;

    //location of the cone or pole relative to the robot
    private TrcPose2D targetLocation;

    public TaskCyclingCones(Robot robot)
    {
        this.robot = robot;
        driveEvent = new TrcEvent(moduleName + ".drive");
        elevatorEvent = new TrcEvent(moduleName + ".elevator");
        armEvent = new TrcEvent(moduleName + ".arm");
        turretEvent = new TrcEvent(moduleName + ".turret");
        intakeEvent = new TrcEvent(moduleName + ".intake");
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
    public void doFullAutoCycle(VisionType visionType, int conesRemaining, TrcEvent event){
        startCycling(CycleType.AUTO_FULL_CYCLE, visionType, conesRemaining, event, null);
    }
    //teleop call to do pickupOnly
    public void doPickup(VisionType visionType, int conesRemaining, TrcEvent event){
        startCycling(CycleType.PICKUP_ONLY, visionType, conesRemaining, event, null);
    }
    //teleop call to do scoring, only the cone it currently has
    public void scoreCone(VisionType visionType, TrcEvent event){
        startCycling(CycleType.SCORING_ONLY, visionType, 1, event, null);
    }
    //prepare for cycling, start sm
    public void startCycling(CycleType cycleType, VisionType visionType, int conesRemaining, TrcEvent event, TrcNotifier.Receiver callback)
    {
        this.cycleType = cycleType;
        this.visionType = visionType;
        this.conesRemaining = conesRemaining;
        this.onFinishEvent = event;
        this.onFinishCallback = callback;
        cycleTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        sm.start(State.PREPARE_PICKUP);
        if(cycleType == CycleType.SCORING_ONLY){
            //sm.start(cycleType == CycleType.SCORING_ONLY? State.ALIGN_TO_POLE: State.ALIGN_TO_CONE);
        }
    }   //startCycling

    public void setAutoChoices(FtcAuto.AutoChoices autoChoices)
    {
        this.autoChoices = autoChoices;
    }

    private void cycleTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        State state = sm.checkReadyAndGetState();

        if (state != null)
        {
            boolean traceState = true;
            double matchTime = TrcUtil.getModeElapsedTime();

            switch (state)
            {
                case ALIGN_TO_CONE:
                    //don't use vision for horizontally align if its AUTO_FULL_CYCLE
                        //because location of cone stack is set
                    if(cycleType != CycleType.AUTO_FULL_CYCLE && visionType != VisionType.NO_VISION){
                        //use vision to find x, y, location of cone relative to robot
                        TrcVisionTargetInfo<?> targetInfo = robot.vision.getBestDetectedTargetInfo(null);
                        //Todo: not sure if this is correct
                        //relative location from camera to cone
                        targetLocation = new TrcPose2D(targetInfo.distanceFromCamera.x, targetInfo.distanceFromCamera.y, targetInfo.horizontalAngle);
                    }
                    sm.setState(State.PREPARE_PICKUP);
                    break;
                case PREPARE_PICKUP: //1. drive to the cone(using targetLocation) or auto conestack(set location)
                    if(targetLocation != null){
                        robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), true,
                                targetLocation);
                    }
                    else if(cycleType == CycleType.AUTO_FULL_CYCLE){
                        //drive to position where robot can pick up from the cone stack
                        //Todo: find position
                        robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.getAutoTargetPoint(-2.5, -0.3, -104, autoChoices));
                    }
                    //set elevator to height so that intake is several inches above the cone stack
                    //todo: tune elevator height
                    double elevatorHeight = RobotParams.ELEVATOR_PRESET_LEVELS[conesRemaining] + RobotParams.CONE_GRAB_HEIGHT;
                    robot.elevator.setTarget(elevatorHeight, true, 1.0, elevatorEvent);
                    robot.arm.setTarget(RobotParams.ARM_EXTENDED, false, 1.0, armEvent);
                    sm.addEvent(driveEvent);
                    sm.addEvent(armEvent);
                    sm.addEvent(elevatorEvent);
                    sm.waitForEvents(State.START_SPIN, 0.0, true);
                    break;

                case START_SPIN: //2. spin turret to the front and spin intake wheels
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, 1.0, turretEvent, null);
                    robot.intake.setPower(RobotParams.INTAKE_POWER_PICKUP);
                    sm.waitForSingleEvent(turretEvent, State.PICKUP_CONE);
                    break;

                case PICKUP_CONE: //3. lower the elevator to the correct height
                    robot.elevator.setPresetPosition(conesRemaining, elevatorEvent, null);
                    sm.waitForSingleEvent(elevatorEvent, State.RAISE_ELEVATOR);
                    break;

                case RAISE_ELEVATOR: //4. raise the elevator up higher than the pole
                    robot.intake.setPower(0);
                    robot.elevator.setTarget(RobotParams.HIGH_JUNCTION_HEIGHT, true, 1.0, elevatorEvent);
                    sm.waitForSingleEvent(elevatorEvent, State.PREPARE_SCORE);
                    break;

                case PREPARE_SCORE: //5. turn turret to the back while driving backwards to the pole
                    robot.robotDrive.purePursuitDrive.start(
                            driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(-1.2, -0.2, -104.0, autoChoices));
                    robot.turret.setTarget(RobotParams.TURRET_BACK);
                    sm.waitForSingleEvent(driveEvent, State.ALIGN_ARM);
                    break;

                case ALIGN_ARM: //6. set arm to 90 (to reset arm pos after ensuring turret will not bonk, might not be needed)
                    robot.arm.setTarget(RobotParams.ARM_EXTENDED, false, 1.0, armEvent);
                    sm.waitForSingleEvent(armEvent, State.SCORE);
                    break;

                case SCORE: //7. spin intake backwards
                    robot.intake.autoAssist(RobotParams.INTAKE_POWER_DUMP, intakeEvent, null, 1.0);
                    sm.waitForSingleEvent(intakeEvent, State.RESET);
                    break;

                case RESET: //8. turn turret 180 degrees and lower elevator while driving back to start position
                    robot.robotDrive.purePursuitDrive.start(
                            driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(-1.243, -0.061, -104.0, autoChoices));
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, 1.0, turretEvent, null);
                    robot.elevator.setTarget(RobotParams.ELEVATOR_MIN_POS, true, 1.0, elevatorEvent);
                    sm.addEvent(driveEvent);
                    sm.addEvent(turretEvent);
                    sm.addEvent(elevatorEvent);
                    sm.waitForEvents(State.DONE, 0.0, true);
                    break;

                default:
                case DONE:
                    cancel();
                    break;
            }

            if (msgTracer != null && traceState)
            {
                msgTracer.traceStateInfo(
                    sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                    robot.robotDrive.purePursuitDrive, null);
            }
        }
    }   //cycleTask

}   //class TaskCyclingCones
