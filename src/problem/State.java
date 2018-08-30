package problem;

import java.util.ArrayList;
import java.util.List;

public class State {

    private List<RobotConfig> robotPath;
    private List<List<MovingBox>> movingBoxPath;
    private List<List<MovingObstacle>> movingObstaclePath;



    public State(){
        robotPath = new ArrayList<>();
        movingBoxPath = new ArrayList<>();
        movingObstaclePath = new ArrayList<>();
    }

    public void addToPath(RobotConfig robo,
                 List<MovingBox> movingBoxes,
                 List<MovingObstacle> movingObstacles){

        this.robotPath.add(robo);
        this.movingBoxPath.add(movingBoxes);
        this.movingObstaclePath.add(movingObstacles);
    }

    public int returnPathLength(){return robotPath.size();}
    public List<RobotConfig> returnRoboPath(){return robotPath;}
    public List<List<MovingBox>> returnMovingBoxPath(){return movingBoxPath;}
    public List<List<MovingObstacle>> returnMovingObstaclePath(){return movingObstaclePath;}
}
