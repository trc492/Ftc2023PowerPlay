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
import TrcCommonLib.trclib.TrcOpenCvPipeline;
import TrcCommonLib.trclib.TrcPose2D;
import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;
import TrcCommonLib.trclib.TrcVisionTargetInfo;
import TrcFtcLib.ftclib.FtcEocvDetector;

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
        CHECK_VISION,
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
    private Double visionExpireTime = null;
    boolean coneIsPreload = true;
    private TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> poleInfo;

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

    //doFullAutoCycle pickups and scores 1 cone
    /*Preconditions(AUTO_FULL_CYCLE)
         * robot facing the cone stack with high and mid junction on either side turret position 90 degrees
         * turret position 90 degrees means intake is right on top of the high pole or mid pole (because we just scored a cone)
         * elevator is height of the high pole
         * arm position extended, intake stopped
    */
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
        cycleTaskObj.registerTask(TrcTaskMgr.TaskType.SLOW_POSTPERIODIC_TASK);
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
            double matchTime = TrcUtil.getModeElapsedTime();

            switch (state) {
                case PREPARE_PICKUP: //1. drive forward to the cone and prepare turret to pick it up
                    //drive to the cone stack
                    robot.robotDrive.purePursuitDrive.start(
                            driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                            robot.robotDrive.getAutoTargetPoint(RobotParams.CONE_STACK_RED_LEFT.x - RobotParams.TURRET_PICKUP_OFFSET,
                                    RobotParams.CONE_STACK_RED_LEFT.y, -90.0, autoChoices));

                    //set elevator to height so that intake is several inches above the cone stack
                    //todo: tune elevator height
                    robot.turret.setTarget(RobotParams.TURRET_FRONT, 1.0, turretEvent, null);
                    sm.waitForSingleEvent(driveEvent, State.PICKUP_CONE);
                    break;

                case ALIGN_TO_CONE:
                    //if using vision turn the robot to face the cone
                    if (visionType != VisionType.NO_VISION) {
                        //use vision to find x, y, location of cone relative to robot
                        //set pipeline to look for red or blue cones
                        if(autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE){
                            robot.vision.frontEocvVision.setDetectObjectType(EocvVision.ObjectType.RED_CONE);
                        }
                        else{
                            robot.vision.frontEocvVision.setDetectObjectType(EocvVision.ObjectType.BLUE_CONE);
                        }
                        TrcVisionTargetInfo<?> targetInfo = robot.vision.getBestDetectedTargetInfo(null);
                        //Todo: not sure if this is correct
                        //relative location from camera to cone
                        targetLocation = new TrcPose2D(targetInfo.distanceFromCamera.x, targetInfo.distanceFromCamera.y, targetInfo.horizontalAngle);
                    }
                    //if we found the target, strafe so that the cone is centered with the robot(only for auto bc angle should be pretty accurate)
                    if (targetLocation != null && cycleType == CycleType.AUTO_FULL_CYCLE) {
                        robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), true,
                                robot.robotDrive.getAutoTargetPoint(targetLocation.x, 0, 0, autoChoices));

                        sm.waitForSingleEvent(driveEvent, State.PREPARE_PICKUP);
                    }
                    //if we don't see the target give it another second to keep looking(we haven't set expireTime yet
                    else if(visionType == VisionType.ALIGN_VISION && visionExpireTime == null){
                        visionExpireTime = TrcUtil.getCurrentTime() + 1;
                    }
                    else if(visionType == VisionType.ALIGN_VISION && TrcUtil.getCurrentTime() == visionExpireTime){
                        //set vision expire time equal to null
                        visionExpireTime = null;
                        sm.setState(State.PREPARE_PICKUP);
                    }
                    break;

                case PICKUP_CONE: //2. lower elevator to the cone, spin intake wheels
                    robot.elevator.setPresetPosition(conesRemaining, elevatorEvent, null);
                    robot.intake.setPower(RobotParams.INTAKE_POWER_DUMP);//autoAssist(RobotParams.INTAKE_POWER_PICKUP);
                    sm.waitForSingleEvent(elevatorEvent, State.RAISE_ELEVATOR);
                    break;


                case RAISE_ELEVATOR: //3 raise the elevator up higher than the pole
                    robot.intake.setPower(0);
                    robot.elevator.setTarget(RobotParams.HIGH_JUNCTION_HEIGHT, true, 1.0, elevatorEvent);
                    sm.waitForSingleEvent(elevatorEvent, State.PREPARE_SCORE);
                    break;

                case PREPARE_SCORE: //5. turn turret to the right side while driving backwards until intake is right above the pole
                    if(cycleType == CycleType.AUTO_FULL_CYCLE){
                        robot.robotDrive.purePursuitDrive.start(
                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
                                robot.robotDrive.getAutoTargetPoint(-1, -0.5, -90, autoChoices));
                    }
                    //alternatively, use vision to drive until the pole is centered with the camera
                    //drive straight backwards until the middle of the robot is aligned with the pole
                    robot.turret.setTarget(RobotParams.TURRET_RIGHT);
                    sm.waitForSingleEvent(driveEvent, State.CHECK_VISION);
                    break;

                case CHECK_VISION:
                    poleInfo = robot.vision.getDetectedPoleInfo();
                    if (poleInfo != null)
                        sm.setState(State.SCORE);
                    break;

                case SCORE: //7. spin intake backwards
                    //TODO: Use target info to align robot
                    //targetInfo[0].horizontalAngle
                    robot.intake.setPower(RobotParams.INTAKE_POWER_DUMP, 1.0, intakeEvent);
                    sm.waitForSingleEvent(intakeEvent, State.DONE);
                    break;

                //don't need this for now
//                case RESET: //8. turn turret to the front and lower elevator. may need to drive forward a bit for auto
//                    if(cycleType == CycleType.AUTO_FULL_CYCLE){
//                        robot.robotDrive.purePursuitDrive.start(
//                                driveEvent, robot.robotDrive.driveBase.getFieldPosition(), false,
//                                robot.robotDrive.getAutoTargetPoint(-1.5, -0.5, -90.0, autoChoices));
//                    }
//                    robot.turret.setTarget(RobotParams.TURRET_FRONT, 1.0, turretEvent, null);
//                    robot.elevator.setTarget(RobotParams.ELEVATOR_MIN_POS, true, 1.0, elevatorEvent);
//                    robot.arm.setTarget(RobotParams.ARM_EXTENDED);
//                    sm.addEvent(driveEvent);
//                    sm.addEvent(turretEvent);
//                    sm.addEvent(elevatorEvent);
//                    sm.waitForEvents(State.DONE, 0.0, true);
//                    break;

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
