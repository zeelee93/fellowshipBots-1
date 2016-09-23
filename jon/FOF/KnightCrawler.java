package leczner.jon.FOF;

import robocode.*;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Created by jonathanleczner on 9/23/16.
 */
public class KnightCrawler extends TeamRobot {
    final Map<String, RobotData> enemyMap;

    private boolean peek; // Don't turn if there's a robot there
    private int direction = 1; // pos - forward, neg - back
    private double moveAmount; // How much to move
    private boolean horizontal = false;
    private boolean vertical = false;
    private double wallBuffer; // padding for movement
    private long lastDirectionShift;

    private int scanDir = 1; // pos - right, neg - left

    RobotData oldestScanned;
    RobotData target;

    public KnightCrawler() {
        enemyMap = new LinkedHashMap<String, RobotData>(5, 2, true);
    }

    /**
     * run: Move around the walls
     */
    public void run() {
        init();

        while (true) {
            handleRadar();
            doMovement();
            letEmHaveIt();
            scan();
        }
    }

    private void init() {
        // Set colors
        Color midnight = new Color(25, 25, 112);
        Color neonGreen = new Color(57, 255, 20);
        setBodyColor(midnight);
        setGunColor(midnight);
        setRadarColor(midnight);
        setScanColor(Color.GREEN);
        setBulletColor(neonGreen);

        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        peek = false;
        wallBuffer = this.getWidth(); // to keep from hitting walls
        // Initialize moveAmount to the maximum possible for this battlefield.
        moveAmount = Math.max(getBattleFieldWidth() - wallBuffer, getBattleFieldHeight() - wallBuffer);
    }

    private void handleRadar() {
        setTurnRadarRightRadians(scanDir * Double.POSITIVE_INFINITY);
    }

    private void letEmHaveIt() {
        // Update our target robot to fire at
        updateTarget();
        // Update the gun direction
        updateGunDirection();
        // Fires the gun, when it is ready
        fireGunWhenReady();
    }

    private void doMovement() {
        boolean horizontal = false;
        boolean vertical = false;

        int newDirection = direction;
        double newHeading = getHeadingRadians();

        if (target != null) {
            // Check if our robot is at the upper or lower border and hence should move horizontally
            if (getY() < wallBuffer || getY() > getBattleFieldHeight() - wallBuffer) {
                horizontal = true;
            }
            // Check if our robot is at the left or right border and hence should move vertically
            if (getX() < wallBuffer || getX() > getBattleFieldWidth() - wallBuffer) {
                vertical = true;
            }
            // If we are in one of the corners of the battlefield, we could move both horizontally
            // or vertically.
            // In this situation, we need to choose one of the two directions.
            if (horizontal && vertical) {
                // If the horizontal distance to our target is lesser than the vertical distance,
                // then we choose to move vertically, and hence we clear the horizontal flag.
                if (Math.abs(target.targetX - getX()) <= Math.abs(target.targetY - getY())) {
                    horizontal = false; // Do not move horizontally => move vertically
                }
            }
            // Adjust the heading of our robot with 90 degrees, if it must move horizontally.
            // Otherwise the calculated heading is towards moving vertically.
            if (horizontal) {
                newHeading -= Math.PI / 2;
            }
            // Set the robot to turn left the amount of radians we have just calculated
            setTurnLeftRadians(Utils.normalRelativeAngle(newHeading));

            // Check if our robot has finished turning, i.e. has less than 1 degrees left to turn
            if (Math.abs(getTurnRemaining()) < 1 || Math.abs(getVelocity()) < 0.01) {
                // If we should move horizontally, the set the robot to move ahead with the
                // horizontal distance to the target robot. Otherwise, use the vertical distance.
                double delta; // delta is the delta distance to move
                if (horizontal) {
                    delta = target.targetX - getX();
                } else {
                    delta = target.targetY - getY();
                }
                setAhead(delta);

                // Set the new direction of our robot to 1 (meaning move forward) if the delta
                // distance is positive; otherwise it is set to -1 (meaning move backward).
                newDirection = delta > 0 ? 1 : -1;

                // Check if more than 10 turns have past since we changed the direction the last
                // time
                if (getTime() - lastDirectionShift > 10) {
                    // If so, set the new direction to be the reverse direction if the velocity < 1
                    if (Math.abs(getVelocity()) < 1) {
                        newDirection = direction * -1;
                    }
                    // Check if the direction really changed
                    if (newDirection != direction) {
                        // If the new direction != current direction, then set the current direction
                        // to be the new direction and save the current time so we know when we
                        // changed the direction the last time.
                        direction = newDirection;
                        lastDirectionShift = getTime();
                    }
                }
            }
        }
        // Set ahead 100 units forward or backward depending on the direction
        setAhead(100 * direction);
    }

    @Override
    public void onHitRobot(HitRobotEvent e) {
        // If he's in front of us, set back up a bit.
        if (e.getBearing() > -90 && e.getBearing() < 90) {
            back(100);
        } // else he's in back of us, so set ahead a bit.
        else {
            ahead(100);
        }
    }

    @Override
    public void onHitWall(HitWallEvent e) {
        System.out.println("hit wall, turning right");
        turnRight(e.getBearing() + 90);
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
//        letEmHaveIt(e);
        // Note that scan is called automatically when the robot is moving.
        // By calling it manually here, we make sure we generate another scan event if there's a robot on the next
        // wall, so that we do not start moving up it until it's gone.
        if (peek) {
            scan();
        }
    }

    @Override
    public void onRobotDeath(RobotDeathEvent robotDeathEvent) {
        // Gets the name of the robot that died
        final String deadRobotName = robotDeathEvent.getName();

        // Remove the robot data for the robot that died from the enemy map
        enemyMap.remove(deadRobotName);

        // Remove the data entry for the oldest scanned robot, if we have such an entry
        if (oldestScanned != null && oldestScanned.name.equals(deadRobotName)) {
            oldestScanned = null;
        }
        if (target != null && target.name.equals(deadRobotName)) {
            target = null;
        }
    }

    // ------------------ DEBUG -------------------------------------------------------------
    /**
     * Method that paints a filled circle at the specified coordinate (x,y) and given color. The
     * circle will have a radius of 20 pixels (meaning that the diameter will be 40 pixels).
     *
     * @param gfx
     *            is the graphics context to draw within.
     * @param x
     *            is the x coordinate for the center of the circle.
     * @param y
     *            is the y coordinate for the center of the circle.
     * @param color
     *            is the color of the filled circle.
     */
    private void fillCircle(Graphics2D gfx, double x, double y, Color color) {
        // Set the pen color
        gfx.setColor(color);
        // Paint a filled circle (oval) that has a radius of 20 pixels with a center at the input
        // coordinates.
        gfx.fillOval((int) x - 20, (int) y - 20, 40, 40);
    }

    @Override
    public void onPaint(Graphics2D g) {
        // Set the line width to 2 pixels
        g.setStroke(new BasicStroke(2f));

        // Prepare colors for painting the scanned coordinate and target coordinate
        Color color1 = new Color(0x00, 0xFF, 0x00, 0x40); // Green with 25% alpha blending
        Color color2 = new Color(0xFF, 0xFF, 0x00, 0x40); // Yellow with 25% alhpa blending

        // Paint a two circles for each robot in the enemy map. One circle where the robot was
        // scanned the last time, and another circle where our robot must point the gun in order to
        // hit it (target coordinate). In addition, a line is drawn between these circles.
        for (RobotData robot : enemyMap.values()) {
            // Paint the two circles and a line
            fillCircle(g, robot.scannedX, robot.scannedY, color1); // scanned coordinate
            fillCircle(g, robot.targetX, robot.targetY, color2); // target coordinate
            g.setColor(color1);
            g.drawLine((int) robot.scannedX, (int) robot.scannedY, (int) robot.targetX, (int) robot.targetY);
        }

        // Paint a two circles for the target robot. One circle where the robot was
        // scanned the last time, and another circle where our robot must point the gun in order to
        // hit it (target coordinate). In addition, a line is drawn between these circles.
        if (target != null) {
            // Prepare colors for painting the scanned coordinate and target coordinate
            color1 = new Color(0xFF, 0x7F, 0x00, 0x40); // Orange with 25% alpha blending
            color2 = new Color(0xFF, 0x00, 0x00, 0x80); // Red with 50% alpha blending

            // Paint the two circles and a line
            fillCircle(g, target.scannedX, target.scannedY, color1); // scanned coordinate
            fillCircle(g, target.targetX, target.targetY, color2); // target coordinate
            g.setColor(color1);
            g.drawLine((int) target.scannedX, (int) target.scannedY, (int) target.targetX, (int) target.targetY);
        }
    }

    // ------------------ HELPERS -------------------------------------------------------------
    /**
     * Method that returns a value that is guaranteed to be within a value range defined by a
     * minimum and maximum value based on an input value.<br>
     * If the input value is lesser than the minimum value, the returned value will be set to the
     * minimum value.<br>
     * If the input value is greater than the maximum value, the returned value will be set to the
     * maximum value.<br>
     * Otherwise the returned value will be equal to the input value.
     *
     * @param value
     *            is the input value to limit.
     * @param min
     *            is the allowed minimum value.
     * @param max
     *            is the allowed maximum value.
     * @return the limited input value that is guaranteed to be within the specified minimum and
     *         maximum range.
     */
    private double limit(double value, double min, double max) {
        return Math.min(max, Math.max(min, value));
    }

    /**
     * Methods that returns the distance to a coordinate (x,y) from our robot.
     *
     * @param x
     *            is the x coordinate.
     * @param y
     *            is the y coordinate.
     * @return the distance to the coordinate (x,y).
     */
    private double distanceTo(double x, double y) {
        return Math.hypot(x - getX(), y - getY());
    }

    /**
     * Method that returns the angle to a coordinate (x,y) from our robot.
     *
     * @param x
     *            is the x coordinate.
     * @param y
     *            is the y coordinate.
     * @return the angle to the coordinate (x,y).
     */
    private double angleTo(double x, double y) {
        return Math.atan2(x - getX(), y - getY());
    }

    /**
     * Method that returns the bearing to a coordinate (x,y) from the position and heading of our
     * robot. The bearing is the delta angle between the heading of our robot and the angle of the
     * specified coordinate.
     *
     * @param x
     *            is the x coordinate.
     * @param y
     *            is the y coordinate.
     * @return the angle to the coordinate (x,y).
     */
    private double bearingTo(double heading, double x, double y) {
        return Utils.normalRelativeAngle(angleTo(x, y) - heading);
    }

    /**
     * Find closest for now, maybe weakest later
     */
    private void updateTarget() {
        target = null;
        // Create a list over possible target robots that is a copy of robot data from the enemy map
        ArrayList<RobotData> targets = new ArrayList<RobotData>(enemyMap.values());

        // Set the target robot to be the one among all possible target robots that is closest to
        // our robot.
        double minDist = Double.POSITIVE_INFINITY;
        for (RobotData robot : targets) {
            double dist = distanceTo(robot.targetX, robot.targetY);
            if (dist < minDist) {
                minDist = dist;
                target = robot;
            }
        }

        // If we still haven't got a target robot, then take the first one from our list of target
        // robots if the list is not empty.
        if (target == null && targets.size() > 0) {
            target = targets.get(0);
        }
    }

    /**
     * To target
     */
    private void updateGunDirection() {
        // Only update the gun direction, if we have a current target
        if (target != null) {
            // Calculate the bearing between the gun and the target, which can be positive or
            // negative
            double targetBearing = bearingTo(getGunHeadingRadians(), target.targetX, target.targetY);
            // Set the gun to turn right the amount of radians defined by the bearing to the target
            setTurnGunRightRadians(targetBearing); // positive => turn right, negative => turn left
        }
    }

    private void fireGunWhenReady() {
        // We only fire the fun, when we have a target robot
        if (target != null) {
            // Only fire when the angle of the gun is pointing at our (virtual) target robot

            // Calculate the distance between between our robot and the target robot
            double dist = distanceTo(target.targetX, target.targetY);
            // Angle that "covers" the the target robot from its center to its edge
            double angle = Math.atan(wallBuffer / dist);

            // Check if the remaining angle (turn) to move the gun is less than our calculated cover
            // angle
            if (Math.abs(getGunTurnRemaining()) < angle) {
                // If so, our gun should be pointing at our target so we can hit it => fire!!
                setFire(3);
            }
        }
    }

    /**
     * This class is used for storing data about a robot that has been scanned.<br>
     * The data is mainly a snapshot of specific scanned data like the scanned position (x,y),
     * velocity and heading, put also the calculated predicted target position of the robot when our
     * robot needs to fire at the scanned robot.<br>
     * Note that this class calculates the position (x,y) of the scanned robot as our robot moves,
     * and hence data like the angle and distance to the scanned robot will change over time. by
     * using the position, it is easy to calculate a new angle and distance to the robot.
     */
    class RobotData {
        final String name; // name of the scanned robot
        double scannedX; // x coordinate of the scanned robot based on the last update
        double scannedY; // y coordinate of the scanned robot based on the last update
        double scannedVelocity; // velocity of the scanned robot from the last update
        double scannedHeading; // heading of the scanned robot from the last update
        double targetX; // predicated x coordinate to aim our gun at, when firing at the robot
        double targetY; // predicated y coordinate to aim our gun at, when firing at the robot

        /**
         * Creates a new robot data entry based on new scan data for a scanned robot.
         *
         * @param event
         *            is a ScannedRobotEvent event containing data about a scanned robot.
         */
        RobotData(ScannedRobotEvent event) {
            // Store the name of the scanned robot
            name = event.getName();
            // Updates all scanned facts like position, velocity, and heading
            update(event);
            // Initialize the coordinates (x,y) to fire at to the updated scanned position
            targetX = scannedX;
            targetY = scannedY;
        }

        /**
         * Updates the scanned data based on new scan data for a scanned robot.
         *
         * @param event
         *            is a ScannedRobotEvent event containing data about a scanned robot.
         */
        void update(ScannedRobotEvent event) {
            // Get the position of the scanned robot based on the ScannedRobotEvent
            Point2D.Double pos = getPosition(event);
            // Store the scanned position (x,y)
            scannedX = pos.x;
            scannedY = pos.y;
            // Store the scanned velocity and heading
            scannedVelocity = event.getVelocity();
            scannedHeading = event.getHeadingRadians();
        }

        /**
         * Returns the position of the scanned robot based on new scan data for a scanned robot.
         *
         * @param event
         *            is a ScannedRobotEvent event containing data about a scanned robot.
         * @return the position (x,y) of the scanned robot.
         */
        Point2D.Double getPosition(ScannedRobotEvent event) {
            // Gets the distance to the scanned robot
            double distance = event.getDistance();
            // Calculate the angle to the scanned robot (our robot heading + bearing to scanned
            // robot)
            double angle = getHeadingRadians() + event.getBearingRadians();

            // Calculate the coordinates (x,y) of the scanned robot
            double x = getX() + Math.sin(angle) * distance;
            double y = getY() + Math.cos(angle) * distance;

            // Return the position as a point (x,y)
            return new Point2D.Double(x, y);
        }
    }
}

