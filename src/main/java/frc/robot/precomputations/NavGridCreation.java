package frc.robot.precomputations;

public class NavGridCreation {
    private static class Vec2 {
        public double x;
        public double y;

        public Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public Vec2 add(Vec2 v) {
            this.x += v.x;
            this.y += v.y;

            return this;
        }

        public Vec2 sub(Vec2 v) {
            this.x -= v.x;
            this.y -= v.y;

            return this;
        }

        public double length() {
            return Math.hypot(x, y);
        }

        public Vec2 normalize() {
            double l = length();

            if(l == 0) {
                this.x = this.y = 0;
            } else {
                l = 1 / l;
                this.x *= l;
                this.y *= l;
            }

            return this;
        }

        public double cross(Vec2 a) {
            return this.x * a.y - this.y * a.x;
        }

        public double dot(Vec2 a) {
            return this.x * a.x + this.y * a.y;
        }
    }
        
    private static final double NODE_SIZE = 0.1;

    private static final Vec2 LEFT_TRUSS_BLUE = new Vec2(5.61, 5.37);
    private static final Vec2 CENTER_TRUSS_BLUE = new Vec2(3.39, 4.10);
    private static final Vec2 RIGHT_TRUSS_BLUE = new Vec2(5.61, 2.85);

    private static final Vec2 LEFT_TRUSS_RED = new Vec2(10.96, 5.34);
    private static final Vec2 CENTER_TRUSS_RED = new Vec2(13.20, 4.10);
    private static final Vec2 RIGHT_TRUSS_RED = new Vec2(10.96, 2.82);

    private static final Vec2 LOWER_LEFT_SUB_BLUE = new Vec2(0.00, 6.49);
    private static final Vec2 UPPER_LEFT_SUB_BLUE = new Vec2(0.90, 6.04);
    private static final Vec2 LOWER_RIGHT_SUB_BLUE = new Vec2(0.00, 4.59);
    private static final Vec2 UPPER_RIGHT_SUB_BLUE = new Vec2(0.90, 5.00);

    private static final Vec2 LOWER_LEFT_SUB_RED = new Vec2(15.66, 6.49);
    private static final Vec2 UPPER_LEFT_SUB_RED = new Vec2(15.66, 6.04);
    private static final Vec2 LOWER_RIGHT_SUB_RED = new Vec2(16.49, 4.59);
    private static final Vec2 UPPER_RIGHT_SUB_RED = new Vec2(15.66, 5.00);
    public static void main(String[] args) {

    }
}
