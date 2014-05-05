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

import static org.junit.Assert.*;
import static strategy.common.PlayerColor.*;
import static strategy.game.common.PieceType.*;
import static strategy.game.common.MoveResultStatus.*;

import java.util.ArrayList;

import org.junit.*;

import strategy.common.*;
import strategy.game.*;
import strategy.game.common.*;

/**
 * Test suite for GammaStrategyMaster.
 * @author cguertin, ranmow
 * @version Sep 12, 2013
 */
public class GammaStrategyStudentTest
{
	private ArrayList<PieceLocationDescriptor> redConfiguration;
	private ArrayList<PieceLocationDescriptor> blueConfiguration;
	private final static StrategyGameFactory factory = StrategyGameFactory.getInstance();
	private StrategyGameController game;	// used for many tests
	
	// Locations used in the test
	private Location
		loc00 = new Location2D(0, 0),
		loc01 = new Location2D(0, 1),
		loc02 = new Location2D(0, 2),
		loc03 = new Location2D(0, 3),
		loc04 = new Location2D(0, 4),
		loc10 = new Location2D(1, 0),
		loc11 = new Location2D(1, 1),
		loc12 = new Location2D(1, 2),
		loc13 = new Location2D(1, 3),
		loc14 = new Location2D(1, 4),
		loc15 = new Location2D(1, 5),
		loc21 = new Location2D(2, 1),
		loc22 = new Location2D(2, 2),
		loc23 = new Location2D(2, 3),
		loc24 = new Location2D(2, 4),
		loc31 = new Location2D(3, 1),
		loc41 = new Location2D(4, 1),
		loc42 = new Location2D(4, 2),
		loc43 = new Location2D(4, 3),
		loc51 = new Location2D(5, 1),
		loc52 = new Location2D(5, 2),
		loc53 = new Location2D(5, 3),
		loc54 = new Location2D(5, 4),
		badLoc = new Location2D(-1, 6)
		;
	
	/*
	 * The board with the initial configuration looks like this:
	 *   |  0  |  1  |  2  |  3  |  4  |  5  |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 5 | MAR | COL | COL | CPT | CPT | LT  |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 4 | LT  | LT  | SGT | SGT | SGT |  F  |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 3 |     |     |     |     |     |     |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 2 |     |     |     |     |     |     |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 1 |  F  | LT  | LT  | SGT | SGT | SGT |
	 * - +-----+-----+-----+-----+-----+-----+
	 * 0 | MAR | COL | COL | CPT | CPT | LT  |
	 * - +-----+-----+-----+-----+-----+-----+
	 *   |  0  |  1  |  2  |  3  |  4  |  5  |
	 */
	@Before
	public void setup() throws StrategyException
	{
		redConfiguration = new ArrayList<PieceLocationDescriptor>();
		blueConfiguration = new ArrayList<PieceLocationDescriptor>();
		addToConfiguration(FLAG, RED, 0, 1);
		addToConfiguration(MARSHAL, RED, 0, 0);
		addToConfiguration(COLONEL, RED, 1, 0);
		addToConfiguration(COLONEL, RED, 2, 0);
		addToConfiguration(CAPTAIN, RED, 3, 0);
		addToConfiguration(CAPTAIN, RED, 4, 0);
		addToConfiguration(LIEUTENANT, RED, 5, 0);
		addToConfiguration(LIEUTENANT, RED, 1, 1);
		addToConfiguration(LIEUTENANT, RED, 2, 1);
		addToConfiguration(SERGEANT, RED, 3, 1);
		addToConfiguration(SERGEANT, RED, 4, 1);
		addToConfiguration(SERGEANT, RED, 5, 1);
		addToConfiguration(FLAG, BLUE, 5, 4);
		addToConfiguration(MARSHAL, BLUE, 0, 5);
		addToConfiguration(COLONEL, BLUE, 1, 5);
		addToConfiguration(COLONEL, BLUE, 2, 5);
		addToConfiguration(CAPTAIN, BLUE, 3, 5);
		addToConfiguration(CAPTAIN, BLUE, 4, 5);
		addToConfiguration(LIEUTENANT, BLUE, 5, 5);
		addToConfiguration(LIEUTENANT, BLUE, 0, 4);
		addToConfiguration(LIEUTENANT, BLUE, 1, 4);
		addToConfiguration(SERGEANT, BLUE, 2, 4);
		addToConfiguration(SERGEANT, BLUE, 3, 4);
		addToConfiguration(SERGEANT, BLUE, 4, 4);
		game = factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
		game.startGame();
	}
	
	@Test(expected=StrategyException.class)
	public void cannotCreateGammaStrategyWithNullConfigurations() throws StrategyException
	{
		factory.makeGammaStrategyGame(null, null);
	}
	
	@Test
	public void createGammaStrategyController() throws StrategyException
	{
		assertNotNull(factory.makeGammaStrategyGame(redConfiguration, blueConfiguration));
	}
	
	@Test(expected=StrategyException.class)
	public void redConfigurationHasTooFewItem() throws StrategyException
	{
		redConfiguration.remove(0);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void blueConfigurationHasTooManyItems() throws StrategyException
	{
		addToConfiguration(SERGEANT, BLUE, 0, 3);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnInvalidRow() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshall @ (0, 0)
		addToConfiguration(MARSHAL, RED, 0, 3);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnInvalidColumn() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshall @ (0, 0)
		addToConfiguration(MARSHAL, RED, -1, 0);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeBluePieceOnInvalidRow() throws StrategyException
	{
		blueConfiguration.remove(11);	// Sergeant @ (0, 4)
		addToConfiguration(SERGEANT, BLUE, 0, 2);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeBluePieceOnInvalidColumn() throws StrategyException
	{
		blueConfiguration.remove(11);	// Sergeant @ (0, 4)
		addToConfiguration(SERGEANT, BLUE, 6, 4);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void twoPiecesOnSameLocationInStartingConfiguration() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshall @ (0, 0)
		addToConfiguration(MARSHAL, RED, 0, 1); // Same place as RED Flag
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void usePieceNotInVersionInStartingConfiguration() throws StrategyException
	{
		redConfiguration.remove(1); // Marshall @ (0, 0)
		addToConfiguration(BOMB, RED, 0, 0);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void redHasOneColonelAndTwoSergeants() throws StrategyException
	{
		redConfiguration.remove(2); // Colonel @ (1, 0)
		addToConfiguration(SERGEANT, RED, 1, 0);
		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void makeMoveBeforeCallingStartGame() throws StrategyException
	{
		game = factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
		game.move(LIEUTENANT, loc11, loc12);
	}
	
	@Test
	public void redMakesValidFirstMoveStatusIsOK() throws StrategyException
	{
		final MoveResult result = game.move(LIEUTENANT, loc11, loc12);
		assertEquals(OK, result.getStatus());
	}
	
	@Test
	public void redMakesValidFirstMoveAndBoardIsCorrect() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		assertNull(game.getPieceAt(loc11));
		assertEquals(new Piece(LIEUTENANT, RED), game.getPieceAt(loc12));
	}
	
	@Test(expected=StrategyException.class)
	public void redAttemptsMoveFromEmptyLocation() throws StrategyException
	{
		game.move(LIEUTENANT, loc12, loc13);
	}
	
	@Test(expected=StrategyException.class)
	public void redMovesPieceNotOnFromLocation() throws StrategyException
	{
		game.move(LIEUTENANT, loc31, loc12);
	}
	
	@Test
	public void blueMakesValidFirstMoveAndBoardIsCorrect() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		game.move(LIEUTENANT, loc04, loc03);
		assertEquals(new Piece(LIEUTENANT, BLUE), game.getPieceAt(loc03));
	}
	
	@Test(expected=StrategyException.class)
	public void redMovesPieceNotInGame() throws StrategyException
	{
		game.move(SCOUT, loc11, loc12);
	}
	
	@Test(expected=StrategyException.class)
	public void redMovesFromInvalidLocation() throws StrategyException
	{
		game.move(LIEUTENANT, badLoc, loc12);
	}
	
	@Test(expected=StrategyException.class)
	public void blueMovesToInvalidLocation() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		game.move(SERGEANT, loc24, badLoc);
	}
	
	@Test(expected=StrategyException.class)
	public void redMoveOutOfTurn() throws StrategyException
	{
		game.move(SERGEANT, loc41, loc42);
		game.move(SERGEANT, loc42, loc43);
	}
	
	@Test
	public void attemptToMoveAfterGameIsOver() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		/*game.move(SERGEANT, loc24, loc23);
		game.move(LIEUTENANT, loc12, loc11);
		game.move(SERGEANT, loc23, loc24);
		game.move(LIEUTENANT, loc11, loc12);
		game.move(SERGEANT, loc24, loc23);
		game.move(LIEUTENANT, loc12, loc11);
		game.move(SERGEANT, loc23, loc24);
		game.move(LIEUTENANT, loc11, loc12);
		game.move(SERGEANT, loc24, loc23);
		game.move(LIEUTENANT, loc12, loc11);
		game.move(SERGEANT, loc23, loc24);
		game.move(LIEUTENANT, loc11, loc12);*/
	}
	
	@Test(expected=StrategyException.class)
	public void moveWrongColorPiece() throws StrategyException
	{
		game.move(LIEUTENANT, loc04, loc03);
	}
	
	@Test
	public void redWins() throws StrategyException
	{
		game.move(SERGEANT, loc51, loc52);
		game.move(LIEUTENANT, loc04, loc03);
		game.move(SERGEANT, loc52, loc53);
		game.move(LIEUTENANT,  loc03,  loc02);
		final MoveResult moveResult = game.move(SERGEANT, loc53, loc54);
		assertEquals(RED_WINS, moveResult.getStatus());
	}
	
	@Test
	public void blueWins() throws StrategyException
	{
		game.move(SERGEANT, loc51, loc52);
		game.move(LIEUTENANT, loc04, loc03);
		game.move(SERGEANT, loc52, loc53);
		game.move(LIEUTENANT,  loc03,  loc02);
		game.move(SERGEANT, loc53, loc52);
		final MoveResult moveResult = game.move(LIEUTENANT, loc02, loc01);
		assertEquals(BLUE_WINS, moveResult.getStatus());
	}
	
	@Test
	public void redAttackerWinsStrike() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		game.move(LIEUTENANT, loc14, loc13);
		game.move(LIEUTENANT, loc12, loc02);
		game.move(LIEUTENANT, loc13, loc12);
		game.move(COLONEL, loc10, loc11);
		game.move(LIEUTENANT, loc04, loc03);
		final MoveResult moveResult = game.move(COLONEL, loc11, loc12);
		assertEquals(OK, moveResult.getStatus());
		assertEquals(
				new PieceLocationDescriptor(new Piece(COLONEL, RED), loc12),
				moveResult.getBattleWinner());
		assertNull(game.getPieceAt(loc11));
		assertEquals(new Piece(COLONEL, RED), game.getPieceAt(loc12));
	}
	
	@Test
	public void blueAttackerWinsStrike() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // b
		game.move(SERGEANT, loc51, loc52); // r
		game.move(LIEUTENANT, loc13, loc03); // b
		game.move(LIEUTENANT, loc12, loc13); // r
		game.move(COLONEL, loc15, loc14); // b
		game.move(SERGEANT, loc52, loc53); // r
		final MoveResult moveResult = 
				game.move(COLONEL, loc14, loc13); // blue attacks
		assertEquals(OK, moveResult.getStatus());
		assertEquals(
				new PieceLocationDescriptor(new Piece(COLONEL, BLUE), loc13),
				moveResult.getBattleWinner());
		Piece pieceat13 = game.getPieceAt(loc13);
		assertEquals(new Piece(COLONEL, BLUE), pieceat13); // blue defender wins
	}
	
	@Test
	public void redDefenderWinsStrike() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // blue
		game.move(LIEUTENANT, loc12, loc02); // r
		game.move(LIEUTENANT, loc13, loc12); // b
		game.move(COLONEL, loc10, loc11); // r
		final MoveResult moveResult = 
				game.move(LIEUTENANT, loc12, loc11); // blue
		assertEquals(OK, moveResult.getStatus());
		assertEquals(
				new PieceLocationDescriptor(new Piece(COLONEL, RED), loc12),
				moveResult.getBattleWinner());
		assertNull(game.getPieceAt(loc11));
		Piece pieceat12 = game.getPieceAt(loc12);
		assertEquals(new Piece(COLONEL, RED), pieceat12); // red defender wins
	}
	
	@Test
	public void blueDefenderWinsStrike() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // b
		game.move(SERGEANT, loc51, loc52); // r
		game.move(LIEUTENANT, loc13, loc03); // b
		game.move(LIEUTENANT, loc12, loc13); // r
		game.move(COLONEL, loc15, loc14); // b
		final MoveResult moveResult = 
				game.move(LIEUTENANT, loc13, loc14); // r
		assertEquals(OK, moveResult.getStatus());
		assertEquals(
				new PieceLocationDescriptor(new Piece(COLONEL, BLUE), loc13),
				moveResult.getBattleWinner());
		Piece pieceat13 = game.getPieceAt(loc13);
		assertEquals(new Piece(COLONEL, BLUE), pieceat13); // blue defender wins
	}
	
	@Test
	public void blueAttacksResultInDraw() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // b
		game.move(SERGEANT, loc51, loc52); // r
		game.move(LIEUTENANT, loc13, loc12); // blue attacks = draw
	}
	
	@Test
	public void strikeResultsInDraw() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		game.move(LIEUTENANT, loc14, loc13);
		final MoveResult moveResult = game.move(LIEUTENANT, loc12, loc13);
		assertEquals(OK, moveResult.getStatus());
		assertNull(moveResult.getBattleWinner());
		assertNull(game.getPieceAt(loc12));
		assertNull(game.getPieceAt(loc13));
	}
	
	@Test(expected=StrategyException.class)
	public void attemptToStrikeYourOwnPiece() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc21);
	}
	
	@Test
	public void attemptToMoveDiagonally() throws StrategyException
	{
		try {
			game.move(LIEUTENANT, loc11, loc22);
			fail("Exception expected");
		} catch (StrategyException e) {
			assertTrue(true);
		} catch (StrategyRuntimeException e) {
			assertTrue(true);
		}
	}

	@Test(expected=StrategyException.class)
	public void moveToSelfOccupiedLocation() throws StrategyException
	{
		game.move(MARSHAL, loc00, loc00);
	}
	
	@Test(expected=StrategyException.class)
	public void moveOffBoard() throws StrategyException
	{
		game.move(MARSHAL, loc00, new Location2D(-1,0));
	}
	
	@Test
	public void attemptToMoveFurtherThanOneLocation() throws StrategyException
	{
		try {
			game.move(LIEUTENANT, loc11, loc13);
			fail("Exception expected");
		} catch (StrategyException e) {
			assertTrue(true);
		} catch (StrategyRuntimeException e) {
			assertTrue(true);
		}
	}
	
	@Test(expected=StrategyException.class)
	public void attemptToMoveFlag() throws StrategyException
	{
		game.move(FLAG, loc01, loc02);
	}
	
	
	@Test(expected=StrategyException.class)
	public void attemptToRestartGameInProgress() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12);
		game.startGame();
	}
	
	@Test (expected=StrategyException.class)
	public void attemptToRestartCompletedGame() throws StrategyException
	{
		game.move(SERGEANT, loc51, loc52);
		game.move(LIEUTENANT, loc04, loc03);
		game.move(SERGEANT, loc52, loc53);
		game.move(LIEUTENANT,  loc03,  loc02);
		game.move(SERGEANT, loc53, loc54);
		game.startGame();
	}
	
	@Test(expected=StrategyException.class)
	public void makeMoveAfterEndGame() throws StrategyException
	{
		game.move(SERGEANT, loc51, loc52);
		game.move(LIEUTENANT, loc04, loc03);
		game.move(SERGEANT, loc52, loc53);
		game.move(LIEUTENANT,  loc03,  loc02);
		game.move(SERGEANT, loc53, loc54);
		game.move(LIEUTENANT, loc02, loc03);
	}
	
	@Test(expected=StrategyException.class)
	public void moveChokePoint() throws StrategyException
	{
		game.move(CHOKE_POINT, loc22, loc12);
	}
	
	@Test (expected=StrategyException.class)
	public void moveOntoChokePoint() throws StrategyException
	{
		game.move(LIEUTENANT, loc21, loc22);
	}
	
	@Test(expected=StrategyException.class)
	public void redViolateRepetitionRule() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // blue
		game.move(LIEUTENANT, loc12, loc11); // red
		game.move(LIEUTENANT, loc13, loc03); // blue
		game.move(LIEUTENANT, loc11, loc12); // illegal move
	}
	
	@Test(expected=StrategyException.class)
	public void blueViolateRepetitionRule() throws StrategyException
	{
		game.move(LIEUTENANT, loc11, loc12); // red
		game.move(LIEUTENANT, loc14, loc13); // blue
		game.move(LIEUTENANT, loc12, loc11); // red
		game.move(LIEUTENANT, loc13, loc14); // blue
		game.move(SERGEANT, loc41, loc42); // red
		game.move(LIEUTENANT, loc14, loc13); // illegal move
	}
	
	// Helper methods
	private void addToConfiguration(PieceType type, PlayerColor color, int x, int y)
	{
		final PieceLocationDescriptor confItem = new PieceLocationDescriptor(
				new Piece(type, color),
				new Location2D(x, y));
		if (color == PlayerColor.RED) {
			redConfiguration.add(confItem);
		} else {
			blueConfiguration.add(confItem);
		}
	}
}
