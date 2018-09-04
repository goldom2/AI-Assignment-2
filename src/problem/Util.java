//package problem;
//
//import java.io.BufferedWriter;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.util.List;
//
//public class Util {
//
//    public static void writeToOutput(State state) throws IOException {
//
//        int length = state.returnPathLength();
//        int currentLine = 0;
//
//        //assume that all lists are the same size
//        BufferedWriter writer = new BufferedWriter(new FileWriter("solution1.txt"));
//
//        writer.write(Integer.toString(length));
//        writer.newLine();
//        while(currentLine < length) {
//
//            writer.write(Double.toString(state.returnRoboPath().get(currentLine).getPos().getX()) + " "
//                    + Double.toString(state.returnRoboPath().get(currentLine).getPos().getY()) + " "
//                    + Double.toString(state.returnRoboPath().get(currentLine).getOrientation()) + " ");
//
//            for(List<MovingBox> mbPath : state.returnMovingBoxPath()){
//                writer.write(Double.toString(mbPath.get(currentLine).getPos().getX()) + " "
//                        + Double.toString(mbPath.get(currentLine).getPos().getY()) + " ");
//            }
//
//            for(List<MovingObstacle> moPath : state.returnMovingObstaclePath()){
//                writer.write(Double.toString(moPath.get(currentLine).getPos().getX()) + " "
//                        + Double.toString(moPath.get(currentLine).getPos().getY()));
//            }
//
//            writer.newLine();
//            currentLine++;
//        }
//        writer.close();
//    }
//}
