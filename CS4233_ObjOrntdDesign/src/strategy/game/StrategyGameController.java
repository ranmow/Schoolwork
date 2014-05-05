/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

package strategy.game;

import strategy.common.*;
import strategy.game.common.*;

/**
 * The StrategyGameController is the interface for any version of a Strategy game
 * controller. The purpose of this interface is to control the flow of a
 * game between players and to ensure that the rules are followed. Any version
 * of a Strategy game must implement this interface.
 * 
 * This interface defines all of the behavior that an outside client (such as an
 * AI player) can use.
 * 
 * @author RMowris
 * @version Aug 28, 2013
 */
public interface StrategyGameController
{
	/**
	 * Before a game can be played, this method must be invoked.
	 * @throws StrategyException if the game is not in a state where it can be (re)started
	 */
	void startGame() throws StrategyException;
	
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
	MoveResult move(PieceType piece, Location from, Location to) throws StrategyException;
	
	/**
	 * This method returns the piece on the game board that is located at the
	 * specified location.
	 * @param location the location of the desired piece
	 * @return the piece located at the specified location or null if there is none
	 * @throws StrategyException 
	 */
	Piece getPieceAt(Location location) throws StrategyException;
	/**
	 * battle handles conflicts between pieces
	 * @param piece the piece being moved
	 * @param from the location of piece
	 * @param to the destination of piece's move
	 * @return the result of the battle
	 * @throws StrategyException if there is a problem
	 */
	MoveResult battle(PieceType piece, Location from, Location to) throws StrategyException;

	/**
	 * @param to the location that the piece is moving to
	 * @param movingPiece the piece that is being updated
	 */
	void updatePieceLocation(Location to, PieceLocationDescriptor movingPiece);
	/**
	 * checkLocationCoordinates ensures that a location is valid
	 * @param location is the given location
	 * @param x is location's x coord
	 * @param y is location's y coord
	 * @throws StrategyException if the location is not valid
	 */
	void checkLocationCoordinates(Location location, int x, int y) 
			throws StrategyException;
}
