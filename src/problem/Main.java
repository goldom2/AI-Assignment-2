package problem;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class Main {

    public static void main(String[] args) throws IOException {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input1.txt");
//            ps.loadSolution("solution1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }
        System.out.println("Finished loading!");

        List<Box> tmpMb = ps.getMovingBoxes();
        List<Box> tmpMo = ps.getMovingObstacles();

        List<MovingBox> mb = new ArrayList<>();
        List<MovingObstacle> mo = new ArrayList<>();

        for(Box tt : tmpMb){
            mb.add((MovingBox) tt);
        }
        for(Box tt : tmpMo){
            mo.add((MovingObstacle) tt);
        }

        Sampler ss = new Sampler(ps.getInitialRobotConfig(),
                mb, ps.getStaticObstacles(),mo);
        ss.stepObjectiveSampling();
    }
}