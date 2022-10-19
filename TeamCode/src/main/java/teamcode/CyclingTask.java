package teamcode;

import TrcCommonLib.trclib.TrcRobot;
import TrcCommonLib.trclib.TrcStateMachine;
import TrcCommonLib.trclib.TrcTaskMgr;
import TrcCommonLib.trclib.TrcUtil;

public class CyclingTask {
    private static final String moduleName = "cycle task";
    Robot robot;
    Vision visionType;
    Type cycleType;
    private final TrcTaskMgr.TaskObject cycleTaskObj;
    private final TrcStateMachine<State> sm;

    //Vision Types:
    public enum Vision {
        NO_VISION,
        FULL_VISION,//uses vision to horizontally align to target and determine distance to target
        ALIGN_VISION_ONLY//only uses vision to horizontally align to target
    }

    public enum Type {
        FULL_CYCLE, //does a full cycle(pickup and scoring), used for auto and maybe teleop
        SCORING_ONLY,//assumes robot already has a cone, scores it onto the pole
        PICKUP_ONLY//picks up a cone from the stack
    }

    public enum State {
        //Pickup states
        ALIGN_TO_CONE,        //horizontally align to the cone
        FIND_DIST_TO_CONE,    //Determine distance to cone(if doing FULL_VISION)
        DRIVE_TO_CONE,        //Drive forward if we determined distance to cone
        PREPARE_PICKUP,
        GRAB_CONE,
        TURN_AROUND,//if doing FULL_CYCLE, may need to rotate turret to switch from picking up cones to scoring them
        //Scoring states
        ALIGN_TO_POLE,
        FIND_DIST_TO_POLE,
        DRIVE_TO_POLE,
        PREPARE_SCORE,
        SCORE_CONE
    }

    public CyclingTask(Robot robot) {
        sm = new TrcStateMachine<>(moduleName);
        cycleTaskObj = TrcTaskMgr.createTask(moduleName, this::cycleTask);
    }
    //call these methods from outside to run autoassists
    public void doFullCycle(Vision visionType){

        prepareToCycle(Type.FULL_CYCLE, visionType);
    }
    public void autoScoringOnly(Vision visionType){
        prepareToCycle(Type.SCORING_ONLY, visionType);
    }
    public void autoPickupOnly(Vision visionType){
        prepareToCycle(Type.PICKUP_ONLY, visionType);
    }
    //prepare for cycling, start sm
    public void prepareToCycle(Type cycleType, Vision visionType){
        this.visionType = visionType;
        this.cycleType = cycleType;
        cycleTaskObj.registerTask(TrcTaskMgr.TaskType.FAST_POSTPERIODIC_TASK);
        State startingState = State.ALIGN_TO_CONE;
        if(cycleType == Type.SCORING_ONLY){
            startingState = State.ALIGN_TO_POLE;
        }
        sm.setState(startingState);

    }


    private void cycleTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode) {
        final String funcName = "autoShootTask";
        State state = sm.checkReadyAndGetState();

        if (state != null) {
            double matchTime = TrcUtil.getModeElapsedTime();
            boolean traceState = true;

            switch (state) {
                //Todo for 10/19: write out each of the steps in enum "State" in a similar format to CmdAutoHigh, ignore vision steps(like aligning or determining distance for now
            }

        }
    }
}


