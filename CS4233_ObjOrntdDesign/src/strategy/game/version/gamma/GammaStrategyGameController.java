/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/
package strategy.game.version.gamma;
import static strategy.common.PlayerColor.BLUE;
import static strategy.common.PlayerColor.RED;
import static strategy.game.common.PieceType.FLAG;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

import org.hamcrest.Factory;

import strategy.game.StrategyGameController;
import strategy.game.StrategyGameFactory;
import strategy.game.common.Coordinate;
import strategy.game.common.Location;
import strategy.game.common.Location2D;
import strategy.game.common.MoveResult;
import strategy.game.common.MoveResultStatus;
import strategy.game.common.Piece;
import strategy.game.common.PieceLocationDescriptor;
import strategy.game.common.PieceType;
import strategy.game.version.shared.CommonStrategyGameController;
import strategy.common.*;
/**
 * Subclass for gamma strategy gametype's rules
 * @author Ransom and Chrissy
 * @version 1.0
 */
public class GammaStrategyGameController extends CommonStrategyGameController {
	private final Collection<PieceLocationDescriptor> chokeConfig = factory.getChokePointConfigurations();
	
	// Used for checking repetition rule
	private Location lastRedMoveTo = null;
	private Location lastRedMoveFrom = null;
	private Location lastLastRedMoveTo = null;
	private Location lastLastRedMoveFrom = null;
	private Location lastBlueMoveTo = null;
	private Location lastBlueMoveFrom = null;
	private Location lastLastBlueMoveTo = null;
	private Location lastLastBlueMoveFrom = null;
	
	public GammaStrategyGameController()
	{
		gameStarted = false;
		gameOver = false;
		isRedTurn = true;
	}
	
	public void startGame() throws StrategyException{
		if (!gameStarted){
			gameStarted = true;
			isRedTurn = true;
			gameOver = false;
			
			lastRedMoveTo = null;
			lastRedMoveFrom = null;
			lastLastRedMoveTo = null;
			lastLastRedMoveFrom = null;
			lastBlueMoveTo = null;
			lastBlueMoveFrom = null;
			lastLastBlueMoveTo = null;
			lastLastBlueMoveFrom = null;
			
			}
		else throw new StrategyException("You cannot restart the game.");
	}
	
	public MoveResult move(PieceType piece, Location from, Location to) throws StrategyException {
		MoveResult currentMoveResult = null;
		
		if (gameOver) {
			throw new StrategyException("The game is over, you cannot make a move!");
		}
		
		if (!gameStarted) {
			throw new StrategyException("You must start the game!");
		}

		//the flag can't move
		if (piece == FLAG) {
			throw new StrategyException("You can't move your flag!");
		}
		
		//checks if the destination and origin locations of a move are the same
		if ((from.getCoordinate(Coordinate.X_COORDINATE)) == (to.getCoordinate(Coordinate.X_COORDINATE))
				&& (from.getCoordinate(Coordinate.Y_COORDINATE)) == (to.getCoordinate(Coordinate.Y_COORDINATE))) {
			throw new StrategyException("You cannot move a piece to the same location that it is occupying.");
		}
		//checks if move is valid (one square forward, backward, or side-to-side)
		if (!((((from.getCoordinate(Coordinate.X_COORDINATE) + 1) == to.getCoordinate(Coordinate.X_COORDINATE)) 
				|| ((from.getCoordinate(Coordinate.X_COORDINATE) - 1) == to.getCoordinate(Coordinate.X_COORDINATE)))
				|| (((from.getCoordinate(Coordinate.Y_COORDINATE) + 1) == to.getCoordinate(Coordinate.Y_COORDINATE)) 
						|| ((from.getCoordinate(Coordinate.Y_COORDINATE) - 1) == to.getCoordinate(Coordinate.Y_COORDINATE))))) {
			throw new StrategyException("You can't move more than one square!");
		}
		//checking to see if a destination location is off the board
		if (	   (to.getCoordinate(Coordinate.X_COORDINATE) < 0)
				|| (to.getCoordinate(Coordinate.Y_COORDINATE) < 0) 
				|| (to.getCoordinate(Coordinate.X_COORDINATE) > 9) 
				|| (to.getCoordinate(Coordinate.Y_COORDINATE) > 9)) {
			throw new StrategyException("You can't move a piece off of the board!");
		}
		//checks if a move is diagonal
		if (((((from.getCoordinate(Coordinate.X_COORDINATE) + 1) == to.getCoordinate(Coordinate.X_COORDINATE)) 
				|| ((from.getCoordinate(Coordinate.X_COORDINATE) - 1) == to.getCoordinate(Coordinate.X_COORDINATE)))
				&& (((from.getCoordinate(Coordinate.Y_COORDINATE) + 1) == to.getCoordinate(Coordinate.Y_COORDINATE)) 
						|| ((from.getCoordinate(Coordinate.Y_COORDINATE) - 1) == to.getCoordinate(Coordinate.Y_COORDINATE))))) {
			throw new StrategyException("You can't move diagonally!");
		}
		//checks if the move origin is empty
		if(getPieceAt(from) == null) {
			throw new StrategyException("You are moving a nonexistent piece");
		}
		//checks if blue is moving out of its turn
		if (isRedTurn) {
			if (getPieceAt(from).getOwner() == BLUE) {
				throw new StrategyException("It isn't blue's turn.");
			}
		}
		
		//checks if red is moving out of its turn
		if (!isRedTurn) {
			if (getPieceAt(from).getOwner() == RED) {
				throw new StrategyException("It isn't red's turn.");
			}
		}
		
		// This does returns a null pointer exception
		if (piece != getPieceAt(from).getType()) {
			throw new StrategyException("The given piece type does not match the piece in that location");
		}
		
		
		// Save last red and blue moves
		// Check for violation of the move repetition rule
		if (getPieceAt(from) != null){
			if (getPieceAt(from).getOwner() == RED){
				if ((lastLastRedMoveTo == to) && (lastLastRedMoveFrom == from)){
					throw new StrategyException("Red cannot move back to this location, violation of the move repetition rule.");
				}
				lastLastRedMoveTo = lastRedMoveTo;//new Location2D(lastRedMoveTo.getCoordinate(Coordinate.X_COORDINATE), lastRedMoveTo.getCoordinate(Coordinate.Y_COORDINATE));
				lastLastRedMoveFrom =  lastRedMoveFrom;//new Location2D(lastRedMoveFrom.getCoordinate(Coordinate.X_COORDINATE), lastRedMoveFrom.getCoordinate(Coordinate.Y_COORDINATE));
				lastRedMoveTo =  to; //new Location2D(to.getCoordinate(Coordinate.X_COORDINATE), to.getCoordinate(Coordinate.Y_COORDINATE));
				lastRedMoveFrom = from; //new Location2D(from.getCoordinate(Coordinate.X_COORDINATE), from.getCoordinate(Coordinate.Y_COORDINATE));
			}
			if (getPieceAt(from).getOwner() == BLUE){
				if ((lastLastBlueMoveTo == to) && (lastLastBlueMoveFrom == from)){
					throw new StrategyException("Blue cannot move back to this location, violation of the move repetition rule.");
				}
				lastLastBlueMoveTo = lastBlueMoveTo;//new Location2D(lastRedMoveTo.getCoordinate(Coordinate.X_COORDINATE), lastRedMoveTo.getCoordinate(Coordinate.Y_COORDINATE));
				lastLastBlueMoveFrom =  lastBlueMoveFrom;//new Location2D(lastRedMoveFrom.getCoordinate(Coordinate.X_COORDINATE), lastRedMoveFrom.getCoordinate(Coordinate.Y_COORDINATE));
				lastBlueMoveTo =  to; //new Location2D(to.getCoordinate(Coordinate.X_COORDINATE), to.getCoordinate(Coordinate.Y_COORDINATE));
				lastBlueMoveFrom = from; //new Location2D(from.getCoordinate(Coordinate.X_COORDINATE), from.getCoordinate(Coordinate.Y_COORDINATE));
			}
		}
		
		// If there is nothing in the destination location then update moving piece's location
		if(getPieceAt(to) == null) {
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(from), to));
			updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));
		}
		
		//moving into a self-occupied location
		else if (getPieceAt(to).getOwner() == getPieceAt(from).getOwner()) {
			throw new StrategyException("You are trying to move to a location occupied by your own piece.");
		}
		
		// Check if you are moving a choke point
		if (getPieceAt(from) != null) {
			if (getPieceAt(from).getType() == PieceType.CHOKE_POINT) {
			throw new StrategyException("You cannot move a choke point");
			}
		}
		// Check if you are moving to a choke point
		if (getPieceAt(to) != null) {
			if (getPieceAt(to).getType() == PieceType.CHOKE_POINT) {
				throw new StrategyException("You cannot move onto a choke point");
			}
		}
		
		
		
		 //switching the current turn
		if (isRedTurn) {
			isRedTurn = false;
		}
		else isRedTurn = true;
		
		if (currentMoveResult == null) {
			currentMoveResult = battle(piece, from, to);
		}

		return currentMoveResult;
	}
	
	public Piece getPieceAt(Location location) throws StrategyException
	{
		Piece result = null;
		checkLocationCoordinates(location, location.getCoordinate(Coordinate.X_COORDINATE), location.getCoordinate(Coordinate.Y_COORDINATE));
		
		// Create one collection for all pieces so that you only have to iterate through one Collection
		final Collection<PieceLocationDescriptor> allPieces = new ArrayList<PieceLocationDescriptor>();
		allPieces.addAll(factory.getInitialRedConfigurations());
		allPieces.addAll(factory.getInitialBlueConfigurations());
		allPieces.addAll(chokeConfig);
		final Iterator<PieceLocationDescriptor> allPiecesIterator = allPieces.iterator();
		while(allPiecesIterator.hasNext()) {
			PieceLocationDescriptor currentPiece = allPiecesIterator.next();
			int configXCoord = currentPiece.getLocation().getCoordinate(Coordinate.X_COORDINATE);
			int configYCoord = currentPiece.getLocation().getCoordinate(Coordinate.Y_COORDINATE);
			int actualXCoord = location.getCoordinate(Coordinate.X_COORDINATE);
			int actualYCoord = location.getCoordinate(Coordinate.Y_COORDINATE);
			// Test the location of the piece against the location given
			if ((configXCoord == actualXCoord) && (configYCoord == actualYCoord)) {
				result = currentPiece.getPiece();
				return result;
			}
		}
		return null;
	}
	
}
