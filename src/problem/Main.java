package problem;

import java.io.IOException;

public class Main {

    public static void main(String[] args) {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem("input2.txt");
            ps.loadSolution("solution1.txt");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
        }

        System.out.println("Finished loading!");
    }
}