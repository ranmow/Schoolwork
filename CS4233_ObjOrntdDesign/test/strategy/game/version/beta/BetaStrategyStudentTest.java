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
import static java.lang.System.out;
import static org.junit.Assert.*;
import static strategy.common.PlayerColor.*;
import static strategy.game.common.PieceType.*;

import java.util.*;

import org.junit.*;

import strategy.common.*;
import strategy.game.StrategyGameController;
import strategy.game.StrategyGameFactory;
import strategy.game.common.*;
import strategy.game.version.alpha.AlphaStrategyGameController;

/**
 * Description
 * @author cguertin, ranmow
 * @version Sep 11, 2013
 */
public class BetaStrategyStudentTest
{
	/*
	 * # of pieces	type of piece (ranking)
	 * 1	Flag (1)
	 * 1	Marshal	(12)
	 * 2	Colonels (10)	
	 * 2	Captains (8)
	 * 3	Lieutenants	(7)
	 * 3	Sergeants (6)
	 */
	private ArrayList<PieceLocationDescriptor> redConfiguration;
	private ArrayList<PieceLocationDescriptor> blueConfiguration;
	private final static StrategyGameFactory factory = StrategyGameFactory.getInstance();
	private StrategyGameController game;
	
	// Enumeration coverage
	private GameVersion[] beta = GameVersion.values();
	private Coordinate[] coord = Coordinate.values();
	private MoveResultStatus[] results = MoveResultStatus.values();
	private PlayerColor[] colors = PlayerColor.values();
	
	private final Piece redFlag 		= 	new Piece(PieceType.FLAG, PlayerColor.RED);
	private final Piece redMarshal 		= 	new Piece(PieceType.MARSHAL, PlayerColor.RED);
	private final Piece redColonel1 	= 	new Piece(PieceType.COLONEL, PlayerColor.RED);
	private final Piece redColonel2 	= 	new Piece(PieceType.COLONEL, PlayerColor.RED);
	private final Piece redCaptain1 	= 	new Piece(PieceType.CAPTAIN, PlayerColor.RED);
	private final Piece redCaptain2 	= 	new Piece(PieceType.CAPTAIN, PlayerColor.RED);
	private final Piece redLieutenant1 	= 	new Piece(PieceType.LIEUTENANT, PlayerColor.RED);
	private final Piece redLieutenant2 	= 	new Piece(PieceType.LIEUTENANT, PlayerColor.RED);
	private final Piece redLieutenant3 	= 	new Piece(PieceType.LIEUTENANT, PlayerColor.RED);
	private final Piece redSergeant1 	= 	new Piece(PieceType.SERGEANT, PlayerColor.RED);
	private final Piece redSergeant2 	= 	new Piece(PieceType.SERGEANT, PlayerColor.RED);
	private final Piece redSergeant3 	=	new Piece(PieceType.SERGEANT, PlayerColor.RED);
	private final Location redFlagLocation 				= new Location2D(0, 1);
	private final Location redMarshalLocation 			= new Location2D(0, 0);
	private final Location redColonel1Location 			= new Location2D(1, 0);
	private final Location redColonel2Location 			= new Location2D(2, 0);
	private final Location redCaptain1Location 			= new Location2D(3, 0);
	private final Location redCaptain2Location 			= new Location2D(4, 1);
	private final Location redLieutenant1Location 		= new Location2D(5, 1);
	private final Location redLieutenant2Location 		= new Location2D(1, 1);
	private final Location redLieutenant3Location		= new Location2D(2, 1);
	private final Location redSergeant1Location 		= new Location2D(3, 1);
	private final Location redSergeant2Location 		= new Location2D(4, 1);
	private final Location redSergeant3Location 		= new Location2D(5, 1);
	
	private final Piece blueFlag 		= 	new Piece(PieceType.FLAG, PlayerColor.BLUE);
	private final Piece blueMarshal 	= 	new Piece(PieceType.MARSHAL, PlayerColor.BLUE);
	private final Piece blueColonel1 	= 	new Piece(PieceType.COLONEL, PlayerColor.BLUE);
	private final Piece blueColonel2 	= 	new Piece(PieceType.COLONEL, PlayerColor.BLUE);
	private final Piece blueCaptain1 	= 	new Piece(PieceType.CAPTAIN, PlayerColor.BLUE);
	private final Piece blueCaptain2 	= 	new Piece(PieceType.CAPTAIN, PlayerColor.BLUE);
	private final Piece blueLieutenant1 = 	new Piece(PieceType.LIEUTENANT, PlayerColor.BLUE);
	private final Piece blueLieutenant2 = 	new Piece(PieceType.LIEUTENANT, PlayerColor.BLUE);
	private final Piece blueLieutenant3 = 	new Piece(PieceType.LIEUTENANT, PlayerColor.BLUE);
	private final Piece blueSergeant1 	= 	new Piece(PieceType.SERGEANT, PlayerColor.BLUE);
	private final Piece blueSergeant2 	= 	new Piece(PieceType.SERGEANT, PlayerColor.BLUE);
	private final Piece blueSergeant3 	=	new Piece(PieceType.SERGEANT, PlayerColor.BLUE);
	private final Location blueFlagLocation 		= new Location2D(5, 4);
	private final Location blueMarshalLocation 		= new Location2D(0, 5);
	private final Location blueColonel1Location 	= new Location2D(1, 5);
	private final Location blueColonel2Location 	= new Location2D(2, 5);
	private final Location blueCaptain1Location		= new Location2D(3, 5);
	private final Location blueCaptain2Location		= new Location2D(4, 5);
	private final Location blueLieutenant1Location 	= new Location2D(5, 5);
	private final Location blueLieutenant2Location 	= new Location2D(1, 4);
	private final Location blueLieutenant3Location	= new Location2D(2, 4);
	private final Location blueSergeant1Location 	= new Location2D(3, 4);
	private final Location blueSergeant2Location 	= new Location2D(4, 4);
	private final Location blueSergeant3Location 	= new Location2D(0, 4);
	
	
	@Before
	public void setup()
	{
		redConfiguration = new ArrayList<PieceLocationDescriptor>();
		blueConfiguration = new ArrayList<PieceLocationDescriptor>();
		game = new BetaStrategyGameController();
		
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
		addToConfiguration(LIEUTENANT, BLUE, 1, 4);
		addToConfiguration(LIEUTENANT, BLUE, 2, 4);
		addToConfiguration(SERGEANT, BLUE, 3, 4);
		addToConfiguration(SERGEANT, BLUE, 4, 4);
		addToConfiguration(SERGEANT, BLUE, 0, 4);
		
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
	
	@Test
	public void correctPrintablePieceType() throws StrategyException
	{
		assertEquals(blueFlag.getType().getPrintableName(), "Flag");
	}
	
	@Test
	public void correctPieceTypeSymbol() throws StrategyException
	{
		assertEquals(blueFlag.getType().getSymbol(), "f");
	}
	
	@Test
	public void correctPieceTypeToString() throws StrategyException
	{
		assertEquals(blueFlag.getType().toString(), "Flag");
	}
	
	@Test
	public void correctPieceOwner() throws StrategyException
	{
		assertEquals(blueFlag.getOwner(), PlayerColor.BLUE);
	}
	
	@Test
	public void pieceToString() throws StrategyException
	{
		assertEquals(blueFlag.toString(), "BLUE Flag");
	}
	
	@Test
	public void pieceEqualsSame() throws StrategyException
	{
		assertTrue(blueFlag.equals(blueFlag));
	}
	
	@Test
	public void pieceNotEqualsSame() throws StrategyException
	{
		assertFalse(blueFlag.equals(redFlag));
	}

	@Test
	public void pieceNotEqualsPlace() throws StrategyException
	{
		assertFalse(blueFlag.equals(redCaptain1Location));
	}
	
	@Test
	public void pieceEqualsNewSamePiece() throws StrategyException
	{
		assertTrue(blueFlag.equals(new Piece(PieceType.FLAG, PlayerColor.BLUE)));
	}

	@Test
	public void correctHashCode() throws StrategyException
	{
		assertEquals(blueFlag.hashCode(), (blueFlag.getType().hashCode() * blueFlag.getOwner().hashCode()));
	}
	
	
	@Test
	public void createBetaStrategyController() throws StrategyException
	{
		assertNotNull(factory.makeBetaStrategyGame(redConfiguration, blueConfiguration));
	}
	
	@Test
	public void createAlphaStrategyController() throws StrategyException
	{
		assertNotNull(factory.makeAlphaStrategyGame());
	}
	
	@Test(expected=StrategyException.class)
	public void cannotCreateBetaStrategyWithNullConfigurations() throws StrategyException
	{
		factory.makeBetaStrategyGame(null, null);
	}

	@Test(expected=StrategyException.class)
	public void redConfigurationHasTooFewItem() throws StrategyException
	{
		redConfiguration.remove(0); // (FLAG, RED, 0, 1)
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void redConfigurationHasTooManyItem() throws StrategyException
	{
		addToConfiguration(FLAG, RED, 0, 1);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	

	@Test(expected=StrategyException.class)
	public void blueConfigurationHasTooFewItem() throws StrategyException
	{
		blueConfiguration.remove(0); // (FLAG, BLUE, 5, 4)
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void blueConfigurationHasTooManyItem() throws StrategyException
	{
		addToConfiguration(FLAG, BLUE, 5, 4);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnTooHighRow() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshal @ (0, 0)
		addToConfiguration(MARSHAL, RED, 0, 2);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnTooLowRow() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshal @ (0, 0)
		addToConfiguration(MARSHAL, RED, 0, -1);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnTooHighColumn() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshal @ (0, 0)
		addToConfiguration(MARSHAL, RED, 6, 0);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeRedPieceOnTooLowColumn() throws StrategyException
	{
		redConfiguration.remove(1);	// Marshal @ (0, 0)
		addToConfiguration(MARSHAL, RED, -1, 0);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void placeBluePieceOnInvalidRow() throws StrategyException
	{
		blueConfiguration.remove(11);	// Sergeant @ (0, 4)
		addToConfiguration(SERGEANT, BLUE, 0, 2);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyRedFlags() throws StrategyException
	{
		redConfiguration.remove(1); // (MARSHAL, RED, 0, 0)
		addToConfiguration(FLAG, RED, 0, 0); // 2 flags
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	
	@Test(expected=StrategyException.class)
	public void tooManyRedMarshals() throws StrategyException
	{
		redConfiguration.remove(0); // (MARSHAL, RED, 0, 0)
		addToConfiguration(MARSHAL, RED, 0, 1); // 2 marshals
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyRedColonels() throws StrategyException
	{
		redConfiguration.remove(0); // (MARSHAL, RED, 0, 0)
		addToConfiguration(COLONEL, RED, 0, 1); // 2 marshals
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyRedCaptains() throws StrategyException
	{
		redConfiguration.remove(0); // (MARSHAL, RED, 0, 0)
		addToConfiguration(CAPTAIN, RED, 0, 1); // 2 marshals
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyRedLieutenants() throws StrategyException
	{
		redConfiguration.remove(0); // (MARSHAL, RED, 0, 0)
		addToConfiguration(LIEUTENANT, RED, 0, 1); // 2 marshals
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyRedSergeants() throws StrategyException
	{
		redConfiguration.remove(0); // (MARSHAL, RED, 0, 0)
		addToConfiguration(SERGEANT, RED, 0, 1); // 2 marshals
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void tooManyBlueFlags() throws StrategyException
	{
		redConfiguration.remove(1); // (MARSHAL, BLUE, 0, 0)
		addToConfiguration(FLAG, BLUE, 0, 0); // 2 flags
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void testDefaultPieceType() throws StrategyException
	{
		redConfiguration.remove(1); // (MARSHAL, BLUE, 0, 0)
		addToConfiguration(SPY, BLUE, 0, 0);
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
	}
	
	@Test(expected=StrategyException.class)
	public void makeMoveBeforeInitialization() throws StrategyException
	{
		game.move(PieceType.MARSHAL, redFlagLocation, blueCaptain1Location);
	} 
	
	@Test(expected=StrategyException.class)
	public void moveTwoSpaces() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redSergeant3Location, new Location2D(5,3));
	}
	
	@Test(expected=StrategyException.class)
	public void moveDiagonally() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redSergeant3Location, new Location2D(4,2));
	}
	
	
	@Test(expected=StrategyException.class)
	public void moveOffBoard() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.MARSHAL, blueMarshalLocation, new Location2D(-1,5));
	}
	
	@Test (expected=StrategyException.class)
	public void moveFlag() throws StrategyException
	{
		game.startGame();
		game.move(PieceType.FLAG, redFlagLocation, new Location2D(1,1));
	}
	


	@Test(expected=StrategyException.class)
	public void blueMoveOutOfTurn() throws StrategyException
	{
		game.startGame();
		game.move(PieceType.SERGEANT, blueSergeant3Location, new Location2D(0,3));
	}
	
	
	@Test(expected=StrategyException.class)
	public void redMoveOutOfTurn() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.LIEUTENANT, redLieutenant3Location, new Location2D(2,2));
		game.move(PieceType.LIEUTENANT, redLieutenant2Location, new Location2D(1,2));
	}
	

	@Test(expected=StrategyException.class)
	public void wrongType() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redLieutenant3Location, new Location2D(2,2));
	}

	@Test(expected=StrategyException.class)
	public void successfullyMoveToEmptyLocation() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		assertEquals(game.move(PieceType.SERGEANT, blueLieutenant2Location, new Location2D(1,3)),
						new MoveResult(MoveResultStatus.OK, new PieceLocationDescriptor(blueLieutenant2, new Location2D(1,3))));
	}

	
	@Test 
	public void testRanks() throws StrategyException
	{
		assertEquals(12, redMarshal.getRank());
		assertEquals(10, redColonel1.getRank());
		assertEquals(8, redCaptain1.getRank());
		assertEquals(7, redLieutenant1.getRank());
		assertEquals(6, redSergeant1.getRank());
	}




	
	@Test(expected=StrategyException.class)
	public void moveNonexistentPiece() throws StrategyException
	{
		game.startGame();
		game.move(PieceType.CAPTAIN, new Location2D(0,2), new Location2D(0,3));
	}
	
	@Test(expected=StrategyException.class)
	public void movePieceToSameLocation() throws StrategyException
	{
		game.startGame();
		game.move(PieceType.CAPTAIN, new Location2D(0,2), new Location2D(0,2));
	}
	
	@Test(expected=StrategyException.class)
	public void moveToSelfOccupiedLocation() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.CAPTAIN, redCaptain1Location, new Location2D(3,1));
		
	}
	
	@Test (expected=StrategyException.class)
	public void gamesOverMove() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redSergeant3Location, new Location2D(5,2));
		game.move(PieceType.SERGEANT, blueSergeant3Location, new Location2D(0,3));
		game.move(PieceType.SERGEANT,  new Location2D(5,2), new Location2D(5,3)); //red
		game.move(PieceType.SERGEANT,  new Location2D(0,3), new Location2D(0,2)); //blue
		game.move(PieceType.SERGEANT,  new Location2D(5,3), new Location2D(5,2)); //red
		game.move(PieceType.SERGEANT,  new Location2D(0,2), new Location2D(0,1)); //blue
		game.move(PieceType.SERGEANT,  new Location2D(5,2), new Location2D(5,3)); //red
	}
	
	/*@Test
	public void multipleGames() throws StrategyException
	{
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redSergeant3Location, new Location2D(5,2));
		game.move(PieceType.SERGEANT, blueSergeant3Location, new Location2D(0,3));
		game.move(PieceType.SERGEANT,  new Location2D(5,2), new Location2D(5,3));
		game.move(PieceType.SERGEANT,  new Location2D(0,3), new Location2D(0,2));
		game.move(PieceType.SERGEANT,  new Location2D(5,3), new Location2D(5,2));
		assertEquals(game.move(PieceType.SERGEANT,  new Location2D(0,2), new Location2D(0,1)).getStatus(),
				MoveResultStatus.BLUE_WINS);
		game.startGame();
		factory.makeBetaStrategyGame(redConfiguration, blueConfiguration);
		game.move(PieceType.SERGEANT, redSergeant3Location, new Location2D(5,2));
		game.move(PieceType.SERGEANT, blueSergeant3Location, new Location2D(0,3));
		game.move(PieceType.SERGEANT,  new Location2D(5,2), new Location2D(5,3));
		game.move(PieceType.SERGEANT,  new Location2D(0,3), new Location2D(0,2));
		game.move(PieceType.SERGEANT,  new Location2D(5,3), new Location2D(5,2));
		assertEquals(game.move(PieceType.SERGEANT,  new Location2D(0,2), new Location2D(0,1)).getStatus(),
				MoveResultStatus.BLUE_WINS);
	}*/
	
	// These are unused methods that need to be called for coverage purposes
	@Test 
	public void testExtrasInLocation2D() throws StrategyException
	{
		assertEquals(0, redCaptain1Location.distanceTo(new Location2D(3,0)));
		assertEquals(16, redCaptain1Location.hashCode());
		assertEquals("(3,0)", redCaptain1Location.toString());
	}
	
	@Test (expected=StrategyRuntimeException.class)
	public void runTimeException() throws StrategyRuntimeException
	{
		throw new StrategyRuntimeException("testing");
	}
	
}
	
	


