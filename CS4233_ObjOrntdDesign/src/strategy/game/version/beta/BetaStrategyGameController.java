/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

package strategy.game.version.beta;

import static strategy.common.PlayerColor.BLUE;
import static strategy.common.PlayerColor.RED;
import static strategy.game.common.PieceType.CAPTAIN;
import static strategy.game.common.PieceType.COLONEL;
import static strategy.game.common.PieceType.FLAG;
import static strategy.game.common.PieceType.LIEUTENANT;
import static strategy.game.common.PieceType.MARSHAL;
import static strategy.game.common.PieceType.SERGEANT;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

import strategy.common.*;
import strategy.game.*;
import strategy.game.common.*;
import strategy.game.version.shared.*;

/**
 * The BetaStrategyGameController implements the game core for
 * the Beta Strategy version.
 * @author cguertin, ranmow
 * @version Sep 11, 2013
 */
public class BetaStrategyGameController extends CommonStrategyGameController
{
	private int moveCounter = 0;
	
	public BetaStrategyGameController()
	{
		gameStarted = false;
		gameOver = false;
		isRedTurn = true;
		moveCounter = 0;
	}
	
	public void startGame() throws StrategyException{
		if (!gameStarted){
			gameStarted = true;
			isRedTurn = true;
			gameOver = false;
			}
		else throw new StrategyException("You cannot restart the game.");
	}
	

	/*
	 * @see strategy.game.StrategyGameController#move(strategy.game.common.PieceType, strategy.game.common.Location, strategy.game.common.Location)
	 */
	public MoveResult move(PieceType piece, Location from, Location to)
			throws StrategyException
	{
		MoveResult currentMoveResult = null;
		moveCounter++;
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
				|| (to.getCoordinate(Coordinate.X_COORDINATE) > 5) 
				|| (to.getCoordinate(Coordinate.Y_COORDINATE) > 5)) {
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
		
		// If there is nothing in the destination location then update moving piece's location
		if(getPieceAt(to) == null) {
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(from), to));
			updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));
		}
		
		//moving into a self-occupied location
		else if (getPieceAt(to).getOwner() == getPieceAt(from).getOwner()) {
				throw new StrategyException("You are trying to move to a location occupied by your own piece.");
			}
		 //switching the current turn
		if (isRedTurn) {
			isRedTurn = false;
		}
		else isRedTurn = true;
		
		
		if (currentMoveResult == null) {
			currentMoveResult = battle(piece, from, to);
		}
		if (moveCounter == 12 && 
				((currentMoveResult.getStatus() != MoveResultStatus.BLUE_WINS)
						&& (currentMoveResult.getStatus() != MoveResultStatus.RED_WINS))) { 
			//checking if we're over 12 moves because a player might win with their 12th move. So the 13th move
			//will actually return the draw (and the move won't take effect)
			gameOver = true;
			return new MoveResult(MoveResultStatus.DRAW, null);
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
