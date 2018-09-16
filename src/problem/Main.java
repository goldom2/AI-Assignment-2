package problem;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Main {

    public static void main(String[] args) {
        long startTime = System.nanoTime();
//        if (args.length != 2) {
//            System.out.println("Usage: java ProgramName inputFileName outputFileName");
//            System.exit(1);
//        }
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("spaces.txt");
//            ps.loadSolution("solution1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred: Could not load + " + args[0]);
            System.exit(2);
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
        ss.stepObjectiveSampling("solution1.txt");
        long endTime = System.nanoTime();

        System.out.println("Time for execution: " + (endTime - startTime)/1000000);
    }
}