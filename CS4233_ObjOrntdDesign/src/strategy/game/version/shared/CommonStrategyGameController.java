/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/
/**
 * Written for CS4233
 * @version 1.0
 *@author Ransom Mowris and Chrissy Guertin
 */
package strategy.game.version.shared;

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

/**
 * Abstract super class that contains shared methods for gamecontrollers
 * @author Ransom and Chrissy
 * @version 1.0
 */
public abstract class CommonStrategyGameController implements StrategyGameController {
	public final static StrategyGameFactory factory = StrategyGameFactory.getInstance();
	public static boolean gameStarted = false;
	public static boolean gameOver = false;
	private final Collection<PieceLocationDescriptor> redConfig = factory.getInitialRedConfigurations();
	private final Collection<PieceLocationDescriptor> blueConfig = factory.getInitialBlueConfigurations();
	public static boolean isRedTurn = true;
	
	/**
	 * commonStrategyGameController is the factory for commonStrategyGameController
	 */
//	public void commonStrategyGameController() {
//	}
	@Override
	abstract public void startGame() throws StrategyException;
	
	/**
	 * This method executes a move in the game. It is called for every move that must be
	 * made.
	 * 
	 * @param piece
	 *            the piece type that is being moved
	 * @param from
	 *            the location where the piece begins. 
	 * @param to
	 *            the location where the piece is after the move has been made.
	 * @return the result of the move
	 * @throws StrategyException
	 *             if there are any problems in making the move (such as specifying a
	 *             location that does not have the specified piece, or the color of
	 *             the piece is not the color of the player who is moving).
	 */
	@Override
	abstract public MoveResult move(PieceType piece, Location from, Location to) throws StrategyException;
	
	/* (non-Javadoc)
	 * @see strategy.game.StrategyGameController#getPieceAt(strategy.game.common.Location)
	 */
	@Override
	abstract public Piece getPieceAt(Location location) throws StrategyException;
	
	/**
	 * Check a location for validity. Throws an exception if the coordinates
	 * are not equal to the expected value.
	 * @param location the location to check
	 * @param x the expected x-coordinate
	 * @param y the expected y-coordinate
	 * @throws StrategyException if the location's coordinates do not match
	 * 		the expected values.
	 */
	@Override
	public void checkLocationCoordinates(Location location, int x, int y) 
			throws StrategyException
	{
		final int locationX = location.getCoordinate(Coordinate.X_COORDINATE);
		final int locationY = location.getCoordinate(Coordinate.Y_COORDINATE);
		if (x != locationX || y != locationY) {
			throw new StrategyException(
					"Expected (" + x + ',' + y + ") and received ("
							+ locationX + ',' + locationY + ')');
		}
	}
	@Override
	public MoveResult battle(PieceType piece, Location from, Location to) throws StrategyException {
	 	MoveResult currentMoveResult = null;
		 //if the attacked piece is a flag, this ends the game and checks who won
		if(getPieceAt(to).getRank() == 1) { 
			 gameOver = true;
			 if (getPieceAt(from).getOwner() == PlayerColor.BLUE) {
				 currentMoveResult = new MoveResult(MoveResultStatus.BLUE_WINS, new PieceLocationDescriptor(getPieceAt(from), to));

			 }
			 else {
				 currentMoveResult = new MoveResult(MoveResultStatus.RED_WINS, new PieceLocationDescriptor(getPieceAt(from), to));
				 }
			 updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));
			 return currentMoveResult;
		 }
		//this is handling bombs. Destroys the attacking piece if it's not a miner, otherwise it destroys the bomb
		if(getPieceAt(to).getRank() == 2) { 
			if (getPieceAt(from).getRank() == 5) { //if the attacking piece is a miner...
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(from), to));
			 if (getPieceAt(from).getOwner() == PlayerColor.BLUE) {
				 redConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
			 }
			 else {
				 blueConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
				 }
			 updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));
			 
			}
			else { //if it isn't a miner...
				currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(to), to));
				if (getPieceAt(from).getOwner() == BLUE){ // If blue is defending
					blueConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from)); // Remove red attacker
				}
				else {
					redConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from)); // If red is defending, remove blue attacker
				}
			}
			return currentMoveResult;
		 }
		
		//spy handling
		else if ( (getPieceAt(to).getRank() == 12) && (getPieceAt(from).getRank() == 3)) {
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(from), to));
			if (getPieceAt(from).getOwner() == PlayerColor.BLUE) {
				redConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
			 }
			else {
				blueConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
				 }
			updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));

			return currentMoveResult;
		}
		 
		//defender wins, then moves into the attacker's space
		 else if(getPieceAt(to).getRank() > getPieceAt(from).getRank()) {
			if (getPieceAt(to).getOwner() == BLUE){ // If blue is defending
				redConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from)); // Remove red attacker
			}
			else {
				blueConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from)); // If red is defending, remove blue attacker
			}
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(to), from));	
			updatePieceLocation(from, new PieceLocationDescriptor(getPieceAt(to), to));
		}
		
		// if defender has lower rank than attacker, defender is destroyed
		// attacker moves into its place
		else if(getPieceAt(to).getRank() < getPieceAt(from).getRank()) {
			if (getPieceAt(to).getOwner() == RED){ // If blue is attacking
				redConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to)); // Remove red defender
			}
			else {
				blueConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to)); // If red is attacking, remove blue defender
			}
			currentMoveResult = new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(getPieceAt(from), to));
			updatePieceLocation(to, new PieceLocationDescriptor(getPieceAt(from), from));

		}
		
		// Draw, both pieces are removed
		else if(getPieceAt(to).getRank() == getPieceAt(from).getRank()) {
			if (getPieceAt(from).getOwner() == BLUE) { // If blue is attacking
				blueConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from));
				redConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
			}
			else {
				blueConfig.remove(new PieceLocationDescriptor(getPieceAt(to), to));
				redConfig.remove(new PieceLocationDescriptor(getPieceAt(from), from));
			}
			currentMoveResult = new MoveResult(MoveResultStatus.OK, null);
		}
//		//if blue only has one piece left (flag), then red wins
//		 if (blueConfig.size() == 1) {
//			 return new MoveResult(MoveResultStatus.RED_WINS, null);
//		 }
//		 //if red only has one piece left (flag), then blue wins
//		 if (redConfig.size() == 1) {
//			 return new MoveResult(MoveResultStatus.BLUE_WINS, null);
//		 }
		 return currentMoveResult;
	}

	/**
	 * @param to the location that the piece is moving to
	 * @param movingPiece the piece that is being updated
	 */
	public void updatePieceLocation(Location to, PieceLocationDescriptor movingPiece)
	{
		//Piece result = null;
		final Piece piece = movingPiece.getPiece();
		final PlayerColor color = piece.getOwner();
		final Iterator<PieceLocationDescriptor> iterator;
		if (color == PlayerColor.RED) {
			redConfig.remove(movingPiece);
			redConfig.add(new PieceLocationDescriptor(piece, to));
		}
		else {
			blueConfig.remove(movingPiece);
			blueConfig.add(new PieceLocationDescriptor(piece, to));
		}
	}
}
