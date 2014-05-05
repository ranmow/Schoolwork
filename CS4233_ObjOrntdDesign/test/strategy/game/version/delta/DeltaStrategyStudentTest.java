package strategy.game.version.delta;

/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

import static org.junit.Assert.*;
import static strategy.common.PlayerColor.*;
import static strategy.game.common.PieceType.*;
import static strategy.game.common.MoveResultStatus.*;

import java.awt.Color;
import java.util.ArrayList;

import org.junit.*;

import strategy.common.*;
import strategy.game.*;
import strategy.game.common.*;

/**
 * Test suite for DeltaStrategyMaster.
 * @author Chrissy & Ransom
 * @version Sep 12, 2013
 */
public class DeltaStrategyStudentTest
{
	private ArrayList<PieceLocationDescriptor> redConfiguration;
	private ArrayList<PieceLocationDescriptor> blueConfiguration;
	private final static StrategyGameFactory factory = StrategyGameFactory.getInstance();
	private StrategyGameController game;	// used for many tests
	
	// Helper method
	private void addToConfiguration(PieceType type, PlayerColor color, Location location)
	{
		final PieceLocationDescriptor confItem = new PieceLocationDescriptor(
				new Piece(type, color),
				location);
		if (color == PlayerColor.RED) {
			redConfiguration.add(confItem);
		} else {
			blueConfiguration.add(confItem);
		}
	}
	
	// Locations used in the test
	private Location
		loc00 = new Location2D(0, 0),
		loc01 = new Location2D(0, 1),
		loc02 = new Location2D(0, 2),
		loc03 = new Location2D(0, 3),
		loc04 = new Location2D(0, 4),
		loc05 = new Location2D(0, 5),
		loc06 = new Location2D(0,6),
		loc07 = new Location2D(0,7),
		loc08= new Location2D(0,8),
		loc09= new Location2D(0,9),
		loc10= new Location2D(1,0),
		loc11= new Location2D(1,1),
		loc12= new Location2D(1,2),
		loc13= new Location2D(1,3),
		loc14= new Location2D(1,4),
		loc15= new Location2D(1,5),
		loc16= new Location2D(1,6),
		loc17= new Location2D(1,7),
		loc18= new Location2D(1,8),
		loc19= new Location2D(1,9),
		loc20= new Location2D(2,0),
		loc21= new Location2D(2,1),
		loc22= new Location2D(2,2),
		loc23= new Location2D(2,3),
		loc24= new Location2D(2,4),
		loc25= new Location2D(2,5),
		loc26= new Location2D(2,6),
		loc27= new Location2D(2,7),
		loc28= new Location2D(2,8),
		loc29= new Location2D(2,9),
		loc30= new Location2D(3,0),
		loc31= new Location2D(3,1),
		loc32= new Location2D(3,2),
		loc33= new Location2D(3,3),
		loc34= new Location2D(3,4),
		loc35= new Location2D(3,5),
		loc36= new Location2D(3,6),
		loc37= new Location2D(3,7),
		loc38= new Location2D(3,8),
		loc39= new Location2D(3,9),
		loc40 = new Location2D(4, 0),
		loc41 = new Location2D(4, 1),
		loc42 = new Location2D(4, 2),
		loc43 = new Location2D(4, 3),
		loc44 = new Location2D(4, 4),
		loc45 = new Location2D(4, 5),
		loc46 = new Location2D(4,6),
		loc47 = new Location2D(4,7),
		loc48= new Location2D(4,8),
		loc49= new Location2D(4,9),
		loc50= new Location2D(5,0),
		loc51= new Location2D(5,1),
		loc52= new Location2D(5,2),
		loc53= new Location2D(5,3),
		loc54= new Location2D(5,4),
		loc55= new Location2D(5,5),
		loc56= new Location2D(5,6),
		loc57= new Location2D(5,7),
		loc58= new Location2D(5,8),
		loc59= new Location2D(5,9),
		loc60= new Location2D(6,0),
		loc61= new Location2D(6,1),
		loc62= new Location2D(6,2),
		loc63= new Location2D(6,3),
		loc64= new Location2D(6,4),
		loc65= new Location2D(6,5),
		loc66= new Location2D(6,6),
		loc67= new Location2D(6,7),
		loc68= new Location2D(6,8),
		loc69= new Location2D(6,9),
		loc70= new Location2D(7,0),
		loc71= new Location2D(7,1),
		loc72= new Location2D(7,2),
		loc73= new Location2D(7,3),
		loc74= new Location2D(7,4),
		loc75= new Location2D(7,5),
		loc76= new Location2D(7,6),
		loc77= new Location2D(7,7),
		loc78= new Location2D(7,8),
		loc79= new Location2D(7,9),	
		loc80= new Location2D(8,0),	
		loc81 = new Location2D(8, 1),
		loc82 = new Location2D(8, 2),
		loc83 = new Location2D(8, 3),
		loc84 = new Location2D(8, 4),
		loc85 = new Location2D(8, 5),
		loc86 = new Location2D(8,6),
		loc87 = new Location2D(8,7),
		loc88= new Location2D(8,8),
		loc89= new Location2D(8,9),
		loc90= new Location2D(9,0),
		loc91= new Location2D(9,1),
		loc92= new Location2D(9,2),
		loc93= new Location2D(9,3),
		loc94= new Location2D(9,4),
		loc95= new Location2D(9,5),
		loc96= new Location2D(9,6),
		loc97= new Location2D(9,7),
		loc98= new Location2D(9,8),
		loc99= new Location2D(9,9)


		;
	
	@Before
	public void setup() throws StrategyException
	{
		redConfiguration = new ArrayList<PieceLocationDescriptor>();
		blueConfiguration = new ArrayList<PieceLocationDescriptor>();
		addToConfiguration(BOMB, RED, loc00);
		addToConfiguration(BOMB, RED, loc10);
		addToConfiguration(BOMB, RED, loc20);
		addToConfiguration(BOMB, RED, loc30);
		addToConfiguration(BOMB, RED, loc40);
		addToConfiguration(SCOUT, RED, loc50);
		addToConfiguration(SCOUT, RED, loc60);
		addToConfiguration(SCOUT, RED, loc70);
		addToConfiguration(SCOUT, RED, loc80);
		addToConfiguration(SCOUT, RED, loc90);
		addToConfiguration(LIEUTENANT, RED, loc01);
		addToConfiguration(LIEUTENANT, RED, loc11);
		addToConfiguration(SERGEANT, RED, loc21);
		addToConfiguration(SERGEANT, RED, loc31);
		addToConfiguration(SERGEANT, RED, loc41);
		addToConfiguration(SERGEANT, RED, loc51);
		addToConfiguration(MINER, RED, loc61);
		addToConfiguration(MINER, RED, loc71);
		addToConfiguration(MINER, RED, loc81);
		addToConfiguration(SCOUT, RED, loc91);
		addToConfiguration(GENERAL, RED, loc02);
		addToConfiguration(COLONEL, RED, loc12);
		addToConfiguration(COLONEL, RED, loc22);
		addToConfiguration(MAJOR, RED, loc32);
		addToConfiguration(MAJOR, RED, loc42);
		addToConfiguration(MAJOR, RED, loc52);
		addToConfiguration(MINER, RED, loc62);
		addToConfiguration(CAPTAIN, RED, loc72);
		addToConfiguration(CAPTAIN, RED, loc82);
		addToConfiguration(CAPTAIN, RED, loc92);
		addToConfiguration(SCOUT, RED, loc03);
		addToConfiguration(FLAG, RED, loc13);
		addToConfiguration(LIEUTENANT, RED, loc23);
		addToConfiguration(LIEUTENANT, RED, loc33);
		addToConfiguration(SPY, RED, loc43);
		addToConfiguration(MARSHAL, RED, loc53);
		addToConfiguration(CAPTAIN, RED, loc63);
		addToConfiguration(MINER, RED, loc73);
		addToConfiguration(BOMB, RED, loc83);
		addToConfiguration(SCOUT, RED, loc93);
		
		addToConfiguration(BOMB, BLUE, loc06);
		addToConfiguration(SCOUT, BLUE, loc16);
		addToConfiguration(LIEUTENANT, BLUE, loc26);
		addToConfiguration(LIEUTENANT, BLUE, loc36);
		addToConfiguration(MARSHAL, BLUE, loc46);
		addToConfiguration(SPY, BLUE, loc56);
		addToConfiguration(SCOUT, BLUE, loc66);
		addToConfiguration(BOMB, BLUE, loc76);
		addToConfiguration(MINER, BLUE, loc86);
		addToConfiguration(FLAG, BLUE, loc96);
		addToConfiguration(GENERAL, BLUE, loc07);
		addToConfiguration(COLONEL, BLUE, loc17);
		addToConfiguration(COLONEL, BLUE, loc27);
		addToConfiguration(MAJOR, BLUE, loc37);
		addToConfiguration(MAJOR, BLUE, loc47);
		addToConfiguration(MAJOR, BLUE, loc57);
		addToConfiguration(CAPTAIN, BLUE, loc67);
		addToConfiguration(CAPTAIN, BLUE, loc77);
		addToConfiguration(CAPTAIN, BLUE, loc87);
		addToConfiguration(CAPTAIN, BLUE, loc97);
		addToConfiguration(LIEUTENANT, BLUE, loc08);
		addToConfiguration(LIEUTENANT, BLUE, loc18);
		addToConfiguration(SERGEANT, BLUE, loc28);
		addToConfiguration(SERGEANT, BLUE, loc38);
		addToConfiguration(SERGEANT, BLUE, loc48);
		addToConfiguration(SERGEANT, BLUE, loc58);
		addToConfiguration(MINER, BLUE, loc68);
		addToConfiguration(	MINER, BLUE, loc78);
		addToConfiguration(MINER, BLUE, loc88);
		addToConfiguration(MINER, BLUE, loc98);
		addToConfiguration(SCOUT, BLUE, loc09);
		addToConfiguration(SCOUT, BLUE, loc19);
		addToConfiguration(SCOUT, BLUE, loc29);
		addToConfiguration(SCOUT, BLUE, loc39);
		addToConfiguration(SCOUT, BLUE, loc49);
		addToConfiguration(SCOUT, BLUE, loc59);
		addToConfiguration(BOMB, BLUE, loc69);
		addToConfiguration(BOMB, BLUE, loc79);
		addToConfiguration(BOMB, BLUE, loc89);
		addToConfiguration(BOMB, BLUE, loc99);
		game = factory.makeDeltaStrategyGame(redConfiguration, blueConfiguration);
		game.startGame();
	}
	
	@Test
	public void bombBeatsAttacker() throws StrategyException {
		game.move(SCOUT, loc03, loc05);
		game.move(SCOUT, loc16, loc15);
		MoveResult result = game.move(SCOUT, loc05, loc06);
		assertEquals(new PieceLocationDescriptor(new Piece(BOMB, BLUE), loc06), 
				result.getBattleWinner());
	}
	
	@Test
	public void minerDefeatsBomb() throws StrategyException {
		game.move(SCOUT, loc03, loc05);
		game.move(MINER, loc86, loc85);
		game.move(SCOUT, loc05, loc03);
		game.move(MINER, loc85, loc84);
		game.move(SCOUT, loc03, loc04);
		MoveResult result = game.move(MINER, loc84, loc83);
		assertEquals(new PieceLocationDescriptor(new Piece(MINER, BLUE), loc83), 
				result.getBattleWinner());
	}
	
	@Test
	public void spyDefeatsMarshal() throws StrategyException {
		game.move(SPY, loc43, loc44);
		game.move(SCOUT, loc16, loc15);
		game.move(SPY, loc44, loc45);
		game.move(SCOUT, loc15, loc05);
		MoveResult testmove = game.move(SPY, loc45, loc46);
		assertEquals(new PieceLocationDescriptor(new Piece(SPY, RED), loc46), testmove.getBattleWinner());
	}
	
	@Test(expected=StrategyException.class)
	public void bombsDontMove() throws StrategyException {
		game.move(BOMB, loc83, loc84);
	}
	
	@Test
	public void scoutValidMove() throws StrategyException {
		MoveResult testmove = game.move(SCOUT, loc93, loc95);
		assertEquals(MoveResultStatus.OK, testmove.getStatus());
	}
	
	@Test(expected=StrategyException.class)
	public void scoutMoveAndAttack() throws StrategyException {
		game.move(SCOUT, loc92, loc96);
	}
	
	@Test(expected=StrategyException.class)
	public void cannotCreateDeltaStrategyWithNullConfigurations() throws StrategyException
	{
		factory.makeDeltaStrategyGame(null, null);
	}
//	
//	@Test
//	public void createGammaStrategyController() throws StrategyException
//	{
//		assertNotNull(factory.makeGammaStrategyGame(redConfiguration, blueConfiguration));
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redConfigurationHasTooFewItem() throws StrategyException
//	{
//		redConfiguration.remove(0);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void blueConfigurationHasTooManyItems() throws StrategyException
//	{
//		addToConfiguration(SERGEANT, BLUE, 0, 3);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void placeRedPieceOnInvalidRow() throws StrategyException
//	{
//		redConfiguration.remove(1);	// Marshall @ (0, 0)
//		addToConfiguration(MARSHAL, RED, 0, 3);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void placeRedPieceOnInvalidColumn() throws StrategyException
//	{
//		redConfiguration.remove(1);	// Marshall @ (0, 0)
//		addToConfiguration(MARSHAL, RED, -1, 0);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void placeBluePieceOnInvalidRow() throws StrategyException
//	{
//		blueConfiguration.remove(11);	// Sergeant @ (0, 4)
//		addToConfiguration(SERGEANT, BLUE, 0, 2);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void placeBluePieceOnInvalidColumn() throws StrategyException
//	{
//		blueConfiguration.remove(11);	// Sergeant @ (0, 4)
//		addToConfiguration(SERGEANT, BLUE, 6, 4);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void twoPiecesOnSameLocationInStartingConfiguration() throws StrategyException
//	{
//		redConfiguration.remove(1);	// Marshall @ (0, 0)
//		addToConfiguration(MARSHAL, RED, 0, 1); // Same place as RED Flag
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void usePieceNotInVersionInStartingConfiguration() throws StrategyException
//	{
//		redConfiguration.remove(1); // Marshall @ (0, 0)
//		addToConfiguration(BOMB, RED, 0, 0);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redHasOneColonelAndTwoSergeants() throws StrategyException
//	{
//		redConfiguration.remove(2); // Colonel @ (1, 0)
//		addToConfiguration(SERGEANT, RED, 1, 0);
//		factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void makeMoveBeforeCallingStartGame() throws StrategyException
//	{
//		game = factory.makeGammaStrategyGame(redConfiguration, blueConfiguration);
//		game.move(LIEUTENANT, loc11, loc12);
//	}
//	
//	@Test
//	public void redMakesValidFirstMoveStatusIsOK() throws StrategyException
//	{
//		final MoveResult result = game.move(LIEUTENANT, loc11, loc12);
//		assertEquals(OK, result.getStatus());
//	}
//	
//	@Test
//	public void redMakesValidFirstMoveAndBoardIsCorrect() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		assertNull(game.getPieceAt(loc11));
//		assertEquals(new Piece(LIEUTENANT, RED), game.getPieceAt(loc12));
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redAttemptsMoveFromEmptyLocation() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc12, loc13);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redMovesPieceNotOnFromLocation() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc31, loc12);
//	}
//	
//	@Test
//	public void blueMakesValidFirstMoveAndBoardIsCorrect() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(LIEUTENANT, loc04, loc03);
//		assertEquals(new Piece(LIEUTENANT, BLUE), game.getPieceAt(loc03));
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redMovesPieceNotInGame() throws StrategyException
//	{
//		game.move(SCOUT, loc11, loc12);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redMovesFromInvalidLocation() throws StrategyException
//	{
//		game.move(LIEUTENANT, badLoc, loc12);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void blueMovesToInvalidLocation() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(SERGEANT, loc24, badLoc);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redMoveOutOfTurn() throws StrategyException
//	{
//		game.move(SERGEANT, loc41, loc42);
//		game.move(SERGEANT, loc42, loc43);
//	}
//	
//	@Test
//	public void attemptToMoveAfterGameIsOver() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		/*game.move(SERGEANT, loc24, loc23);
//		game.move(LIEUTENANT, loc12, loc11);
//		game.move(SERGEANT, loc23, loc24);
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(SERGEANT, loc24, loc23);
//		game.move(LIEUTENANT, loc12, loc11);
//		game.move(SERGEANT, loc23, loc24);
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(SERGEANT, loc24, loc23);
//		game.move(LIEUTENANT, loc12, loc11);
//		game.move(SERGEANT, loc23, loc24);
//		game.move(LIEUTENANT, loc11, loc12);*/
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void moveWrongColorPiece() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc04, loc03);
//	}
//	
//	@Test
//	public void redWins() throws StrategyException
//	{
//		game.move(SERGEANT, loc51, loc52);
//		game.move(LIEUTENANT, loc04, loc03);
//		game.move(SERGEANT, loc52, loc53);
//		game.move(LIEUTENANT,  loc03,  loc02);
//		final MoveResult moveResult = game.move(SERGEANT, loc53, loc54);
//		assertEquals(RED_WINS, moveResult.getStatus());
//	}
//	
//	@Test
//	public void blueWins() throws StrategyException
//	{
//		game.move(SERGEANT, loc51, loc52);
//		game.move(LIEUTENANT, loc04, loc03);
//		game.move(SERGEANT, loc52, loc53);
//		game.move(LIEUTENANT,  loc03,  loc02);
//		game.move(SERGEANT, loc53, loc52);
//		final MoveResult moveResult = game.move(LIEUTENANT, loc02, loc01);
//		assertEquals(BLUE_WINS, moveResult.getStatus());
//	}
//	
//	@Test
//	public void redAttackerWinsStrike() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(LIEUTENANT, loc14, loc13);
//		game.move(LIEUTENANT, loc12, loc02);
//		game.move(LIEUTENANT, loc13, loc12);
//		game.move(COLONEL, loc10, loc11);
//		game.move(LIEUTENANT, loc04, loc03);
//		final MoveResult moveResult = game.move(COLONEL, loc11, loc12);
//		assertEquals(OK, moveResult.getStatus());
//		assertEquals(
//				new PieceLocationDescriptor(new Piece(COLONEL, RED), loc12),
//				moveResult.getBattleWinner());
//		assertNull(game.getPieceAt(loc11));
//		assertEquals(new Piece(COLONEL, RED), game.getPieceAt(loc12));
//	}
//	
//	@Test
//	public void blueAttackerWinsStrike() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // b
//		game.move(SERGEANT, loc51, loc52); // r
//		game.move(LIEUTENANT, loc13, loc03); // b
//		game.move(LIEUTENANT, loc12, loc13); // r
//		game.move(COLONEL, loc15, loc14); // b
//		game.move(SERGEANT, loc52, loc53); // r
//		final MoveResult moveResult = 
//				game.move(COLONEL, loc14, loc13); // blue attacks
//		assertEquals(OK, moveResult.getStatus());
//		assertEquals(
//				new PieceLocationDescriptor(new Piece(COLONEL, BLUE), loc13),
//				moveResult.getBattleWinner());
//		Piece pieceat13 = game.getPieceAt(loc13);
//		assertEquals(new Piece(COLONEL, BLUE), pieceat13); // blue defender wins
//	}
//	
//	@Test
//	public void redDefenderWinsStrike() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // blue
//		game.move(LIEUTENANT, loc12, loc02); // r
//		game.move(LIEUTENANT, loc13, loc12); // b
//		game.move(COLONEL, loc10, loc11); // r
//		final MoveResult moveResult = 
//				game.move(LIEUTENANT, loc12, loc11); // blue
//		assertEquals(OK, moveResult.getStatus());
//		assertEquals(
//				new PieceLocationDescriptor(new Piece(COLONEL, RED), loc12),
//				moveResult.getBattleWinner());
//		assertNull(game.getPieceAt(loc11));
//		Piece pieceat12 = game.getPieceAt(loc12);
//		assertEquals(new Piece(COLONEL, RED), pieceat12); // red defender wins
//	}
//	
//	@Test
//	public void blueDefenderWinsStrike() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // b
//		game.move(SERGEANT, loc51, loc52); // r
//		game.move(LIEUTENANT, loc13, loc03); // b
//		game.move(LIEUTENANT, loc12, loc13); // r
//		game.move(COLONEL, loc15, loc14); // b
//		final MoveResult moveResult = 
//				game.move(LIEUTENANT, loc13, loc14); // r
//		assertEquals(OK, moveResult.getStatus());
//		assertEquals(
//				new PieceLocationDescriptor(new Piece(COLONEL, BLUE), loc13),
//				moveResult.getBattleWinner());
//		Piece pieceat13 = game.getPieceAt(loc13);
//		assertEquals(new Piece(COLONEL, BLUE), pieceat13); // blue defender wins
//	}
//	
//	@Test
//	public void blueAttacksResultInDraw() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // b
//		game.move(SERGEANT, loc51, loc52); // r
//		game.move(LIEUTENANT, loc13, loc12); // blue attacks = draw
//	}
//	
//	@Test
//	public void strikeResultsInDraw() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		game.move(LIEUTENANT, loc14, loc13);
//		final MoveResult moveResult = game.move(LIEUTENANT, loc12, loc13);
//		assertEquals(OK, moveResult.getStatus());
//		assertNull(moveResult.getBattleWinner());
//		assertNull(game.getPieceAt(loc12));
//		assertNull(game.getPieceAt(loc13));
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void attemptToStrikeYourOwnPiece() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc21);
//	}
//	
//	@Test
//	public void attemptToMoveDiagonally() throws StrategyException
//	{
//		try {
//			game.move(LIEUTENANT, loc11, loc22);
//			fail("Exception expected");
//		} catch (StrategyException e) {
//			assertTrue(true);
//		} catch (StrategyRuntimeException e) {
//			assertTrue(true);
//		}
//	}
//
//	@Test(expected=StrategyException.class)
//	public void moveToSelfOccupiedLocation() throws StrategyException
//	{
//		game.move(MARSHAL, loc00, loc00);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void moveOffBoard() throws StrategyException
//	{
//		game.move(MARSHAL, loc00, new Location2D(-1,0));
//	}
//	
//	@Test
//	public void attemptToMoveFurtherThanOneLocation() throws StrategyException
//	{
//		try {
//			game.move(LIEUTENANT, loc11, loc13);
//			fail("Exception expected");
//		} catch (StrategyException e) {
//			assertTrue(true);
//		} catch (StrategyRuntimeException e) {
//			assertTrue(true);
//		}
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void attemptToMoveFlag() throws StrategyException
//	{
//		game.move(FLAG, loc01, loc02);
//	}
//	
//	
//	@Test(expected=StrategyException.class)
//	public void attemptToRestartGameInProgress() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12);
//		game.startGame();
//	}
//	
//	@Test (expected=StrategyException.class)
//	public void attemptToRestartCompletedGame() throws StrategyException
//	{
//		game.move(SERGEANT, loc51, loc52);
//		game.move(LIEUTENANT, loc04, loc03);
//		game.move(SERGEANT, loc52, loc53);
//		game.move(LIEUTENANT,  loc03,  loc02);
//		game.move(SERGEANT, loc53, loc54);
//		game.startGame();
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void makeMoveAfterEndGame() throws StrategyException
//	{
//		game.move(SERGEANT, loc51, loc52);
//		game.move(LIEUTENANT, loc04, loc03);
//		game.move(SERGEANT, loc52, loc53);
//		game.move(LIEUTENANT,  loc03,  loc02);
//		game.move(SERGEANT, loc53, loc54);
//		game.move(LIEUTENANT, loc02, loc03);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void moveChokePoint() throws StrategyException
//	{
//		game.move(CHOKE_POINT, loc22, loc12);
//	}
//	
//	@Test (expected=StrategyException.class)
//	public void moveOntoChokePoint() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc21, loc22);
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void redViolateRepetitionRule() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // blue
//		game.move(LIEUTENANT, loc12, loc11); // red
//		game.move(LIEUTENANT, loc13, loc03); // blue
//		game.move(LIEUTENANT, loc11, loc12); // illegal move
//	}
//	
//	@Test(expected=StrategyException.class)
//	public void blueViolateRepetitionRule() throws StrategyException
//	{
//		game.move(LIEUTENANT, loc11, loc12); // red
//		game.move(LIEUTENANT, loc14, loc13); // blue
//		game.move(LIEUTENANT, loc12, loc11); // red
//		game.move(LIEUTENANT, loc13, loc14); // blue
//		game.move(SERGEANT, loc41, loc42); // red
//		game.move(LIEUTENANT, loc14, loc13); // illegal move
//	}
}
