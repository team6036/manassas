package frc.robot.auto;

import java.util.ArrayList;

import frc.robot.common.SwerveController;

public class AutoSequence {

    public SwerveController swerve;
    public ArrayList<AbstractAction> actions;
    public int actionIndex;
    public AbstractAction action;
    public boolean done;
    
    public AutoSequence(SwerveController swerve, AbstractAction... actions_arr){
        this.swerve = swerve;

        //convert array into arraylist
        actions = new ArrayList<AbstractAction>();
        for(AbstractAction action_new : actions_arr){
            actions.add(action_new);
        }

        actionIndex = 0;
    }

    public void addAction(AbstractAction action_new){
        actions.add(action_new);
    }

    public void runSequence(){
        
        if(actionIndex > actions.size() - 1){
            swerve.stop();
            done = true;
            return;
        }

        action = actions.get(actionIndex);
        if(!action.done){
            action.runAction(swerve);
        }else{
            actionIndex++;
        }
    }

}
