/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

package strategy.game.version.alpha;

import strategy.common.*;
import strategy.game.*;
import strategy.game.common.*;
import strategy.game.version.shared.*;

/**
 * The AlphaStrategyGameController implements the game core for
 * the Alpha Strategy version.
 * @author cguertin, ranmow
 * @version Sep 6, 2013
 */
public class AlphaStrategyGameController extends CommonStrategyGameController
{
	private final Location redMarshalLocation = new Location2D(0, 0);
	private final Location redFlagLocation = new Location2D(1, 0);
	private final Location blueFlagLocation = new Location2D(0, 1);
	private final Location blueMarshalLocation = new Location2D(1, 1);
	private final Piece redMarshal = new Piece(PieceType.MARSHAL, PlayerColor.RED);
	private final Piece redFlag = new Piece(PieceType.FLAG, PlayerColor.RED);
	private final Piece blueFlag = new Piece(PieceType.FLAG, PlayerColor.BLUE);
	private final Piece blueMarshal = new Piece(PieceType.MARSHAL, PlayerColor.BLUE);
	
	public AlphaStrategyGameController()
	{
		gameStarted = false;
		gameOver = false;
	}
	
	/*
	 * @see strategy.game.StrategyGameController#startGame()
	 */
	public void startGame()
	{
		gameStarted = true;
		gameOver = false;
	}

	
	public Piece getPieceAt(Location location)
	{
		Piece result = null;
		if (location.equals(redMarshalLocation) && gameStarted && !gameOver) {
			result = redMarshal;
		} else if (location.equals(redFlagLocation)) {
			result = redFlag;
		} else if (location.equals(blueFlagLocation)) {
			result = gameOver ? redMarshal : blueFlag;
		} else if (location.equals(blueMarshalLocation)) {
			result = blueMarshal;
		}
		return result;
	}
	
	/*
	 * @see strategy.game.StrategyGameController#move(strategy.game.common.PieceType, strategy.game.common.Location, strategy.game.common.Location)
	 */
	public MoveResult move(PieceType piece, Location from, Location to)
			throws StrategyException
	{
		if (gameOver) {
			throw new StrategyException("The game is over, you cannot make a move");
		}
		if (!gameStarted) {
			throw new StrategyException("You must start the game!");
		}
		if (piece != PieceType.MARSHAL) {
			throw new StrategyException("Invalid piece, expected Marshal and got " +
					piece);
		}
		checkLocationCoordinates(from, 0, 0);
		checkLocationCoordinates(to, 0, 1);
		gameOver = true;
		final PieceLocationDescriptor redMarshal =
				new PieceLocationDescriptor(
					new Piece(PieceType.MARSHAL, PlayerColor.RED),
					new Location2D(0, 1));
		return new MoveResult(MoveResultStatus.RED_WINS, redMarshal);
	}

}
