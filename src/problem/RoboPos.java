package problem;

public class RoboPos {
    RobotConfig cur;
    double weight;
    double heuristic;
    RobotConfig prev;

    public RoboPos(RobotConfig cur, double weight, double heuristic, RobotConfig prev) {
        this.cur = cur;
        this.weight = weight;
        this.heuristic = heuristic;
        this.prev = prev;
    }

    public RobotConfig getRobo() {
        return cur;
    }

    public RobotConfig getPrev() {
        return prev;
    }

    public Double getHeuristic() {
        return heuristic;
    }

    public double getWeight() {
        return weight;
    }

    @Override
    public boolean equals(Object o) {
        if (o instanceof RoboPos) {
            RoboPos other = (RoboPos) o;
            return this.getRobo().getPos().equals(other.getRobo().getPos());
//            return this.getBox().equals(other.getBox())
//                    && this.getWeight() == other.getWeight()
//                    && (this.getPrev() == null
//                    ? other.getPrev() == null : this.getPrev().equals(other.getPrev()));
//            }
        }
        return false;
    }
}
