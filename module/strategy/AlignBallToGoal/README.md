# AlignBallToGoal

## Description

Strafes in a circle around the ball if the ball exists.

## Usage

Add this module to align the ball to the goal.

## Consumes

- `message::strategy::AlignBallToGoal` Task requesting to rotate around the ball.
- `message::Localisation::FilteredBall` Information on where the ball is.
- `message::Localisation::Field` Information regarding the field.

## Emits

- `message::planning::TurnAroundBall` Task requesting to rotate around the ball.

## Dependencies
