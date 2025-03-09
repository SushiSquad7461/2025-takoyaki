package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ReefPositions {
    
    public enum ReefScorePosition {
        FRONT, 
        FRONTLEFT, 
        FRONTRIGHT,
        BACK,
        BACKLEFT, 
        BACKRIGHT
    }
    
    public record ReefPositionsMap(Map<Pose2d, ReefScorePosition> locations, List<Pose2d> poses) {}
    private static ReefPositionsMap scorePositions;

    public static void initialize() {
        if (scorePositions == null) { //adding null check so it isnt initialized every time the alignment command is called
            initializeScorePositions();
        }
    }
    
    public static ReefPositionsMap getScorePositions() {
        if (scorePositions == null) {
            initializeScorePositions();
        }
        return scorePositions;
    }
    
    private static void initializeScorePositions() {
        Distance distanceAway = Inches.of(16.5);
        ArrayList<Pose2d> scorePositionsList = new ArrayList<>();
        Map<Pose2d, ReefScorePosition> locations = new HashMap<>();
        
        // Tag 18 (FRONT)
        Pose2d blueTag18 = new Pose2d(3.6576, 4.0259, Rotation2d.fromDegrees(0));
        final var blueTag18Robot = new Pose2d(
            blueTag18.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(180)), 
            blueTag18.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(180)), 
            blueTag18.getRotation()
        );
        scorePositionsList.add(blueTag18Robot);
        locations.put(blueTag18Robot, ReefScorePosition.FRONT);
        
        // Tag 19 (FRONTLEFT)
        Pose2d blueTag19 = new Pose2d(4.0739, 4.7455, Rotation2d.fromDegrees(-60));
        final var blueTag19Robot = new Pose2d(
            blueTag19.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(120)), 
            blueTag19.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(120)), 
            blueTag19.getRotation()
        );
        scorePositionsList.add(blueTag19Robot);
        locations.put(blueTag19Robot, ReefScorePosition.FRONTLEFT);
        
        // Tag 20 (BACKLEFT)
        Pose2d blueTag20 = new Pose2d(4.9047, 4.7455, Rotation2d.fromDegrees(-120));
        final var blueTag20Robot = new Pose2d(
                blueTag20.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(60)), 
                blueTag20.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(60)), 
                blueTag20.getRotation()
            );
        scorePositionsList.add(blueTag20Robot);
        locations.put(blueTag20Robot, ReefScorePosition.BACKLEFT);
        
        // Tag 21 (BACK)
        Pose2d blueTag21 = new Pose2d(5.3210, 4.0259, Rotation2d.fromDegrees(180));
        final var blueTag21Robot = new Pose2d(
                blueTag21.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(0)), 
                blueTag21.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(0)), 
                blueTag21.getRotation()
            );
        scorePositionsList.add(blueTag21Robot);
        locations.put(blueTag21Robot, ReefScorePosition.BACK);
        
        // Tag 22 (BACKRIGHT)
        Pose2d blueTag22 = new Pose2d(4.9047, 3.3063, Rotation2d.fromDegrees(120));
        final var blueTag22Robot = new Pose2d(
                blueTag22.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-60)), 
                blueTag22.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-60)), 
                blueTag22.getRotation()
            );
        scorePositionsList.add(blueTag22Robot);
        locations.put(blueTag22Robot, ReefScorePosition.BACKRIGHT);
        
        // Tag 17 (FRONTRIGHT)
        Pose2d blueTag17 = new Pose2d(4.0739, 3.3063, Rotation2d.fromDegrees(60));
        final var blueTag17Robot = new Pose2d(
                blueTag17.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-120)), 
                blueTag17.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-120)), 
                blueTag17.getRotation()
            );
        scorePositionsList.add(blueTag17Robot);
        locations.put(blueTag17Robot, ReefScorePosition.FRONTRIGHT);
        
        // Red Alliance
        // Tag 7 (FRONT)
        Pose2d redTag7 = new Pose2d(13.8905, 4.0259, Rotation2d.fromDegrees(180));
        final var redTag7Robot = new Pose2d(
                redTag7.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(0)), 
                redTag7.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(0)), 
                redTag7.getRotation()
            );
        scorePositionsList.add(redTag7Robot);
        locations.put(redTag7Robot, ReefScorePosition.FRONT);
        
        // Tag 6 (FRONTLEFT)
        Pose2d redTag6 = new Pose2d(13.4744, 3.3063, Rotation2d.fromDegrees(120));
        final var redTag6Robot = new Pose2d(
                redTag6.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-60)), 
                redTag6.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-60)), 
                redTag6.getRotation()
            );
        scorePositionsList.add(redTag6Robot);
        locations.put(redTag6Robot, ReefScorePosition.FRONTLEFT);
        
        // Tag 11 (BACKLEFT)
        Pose2d redTag11 = new Pose2d(12.6434, 3.3063, Rotation2d.fromDegrees(60));
        final var redTag11Robot = new Pose2d(
                redTag11.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(-120)), 
                redTag11.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(-120)), 
                redTag11.getRotation()
            );
        scorePositionsList.add(redTag11Robot);
        locations.put(redTag11Robot, ReefScorePosition.BACKLEFT);
        
        // Tag 10 (BACK)
        Pose2d redTag10 = new Pose2d(12.2273, 4.0259, Rotation2d.fromDegrees(0));
        final var redTag10Robot = new Pose2d(
                redTag10.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(180)), 
                redTag10.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(180)), 
                redTag10.getRotation()
            );
        scorePositionsList.add(redTag10Robot);
        locations.put(redTag10Robot, ReefScorePosition.BACK);
        
        // Tag 9 (BACKRIGHT)
        Pose2d redTag9 = new Pose2d(12.6434, 4.7455, Rotation2d.fromDegrees(-60));
        final var redTag9Robot = new Pose2d(
                redTag9.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(120)), 
                redTag9.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(120)), 
                redTag9.getRotation()
            );
        scorePositionsList.add(redTag9Robot);
        locations.put(redTag9Robot, ReefScorePosition.BACKRIGHT);
        
        // Tag 8 (FRONTRIGHT)
        Pose2d redTag8 = new Pose2d(13.4744, 4.7455, Rotation2d.fromDegrees(-120));
        final var redTag8Robot = new Pose2d(
                redTag8.getX() + distanceAway.in(Meters) * Math.cos(Math.toRadians(60)), 
                redTag8.getY() + distanceAway.in(Meters) * Math.sin(Math.toRadians(60)), 
                redTag8.getRotation()
            );
        scorePositionsList.add(redTag8Robot);
        locations.put(redTag8Robot, ReefScorePosition.FRONTRIGHT);
    
        scorePositions = new ReefPositionsMap(locations, scorePositionsList);
    }
}