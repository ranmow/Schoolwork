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

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;

import strategy.common.*;
import strategy.game.common.Coordinate;
import strategy.game.common.Location;
import strategy.game.common.Location2D;
import strategy.game.common.Piece;
import strategy.game.common.PieceLocationDescriptor;
import strategy.game.common.PieceType;
import strategy.game.version.alpha.AlphaStrategyGameController;
import strategy.game.version.beta.BetaStrategyGameController;
import strategy.game.version.gamma.GammaStrategyGameController;
import strategy.game.version.delta.DeltaStrategyGameController;

/**
 * <p>
 * Factory to produce various versions of the Strategy game. This is implemented
 * as a singleton.
 * </p>
 * <p>
 * NOTE: If an error occurs creating any game, that is not specified in the
 * particular factory method's documentation, the factory method should throw a
 * StrategyRuntimeException.
 * </p>
 * 
 * @author RMowris & CGuertin
 * @version Sep 10, 2013
 */
public class StrategyGameFactory {
	private final static StrategyGameFactory instance = new StrategyGameFactory();
	private final static Collection<PieceLocationDescriptor> redConfig = new ArrayList<PieceLocationDescriptor>();
	private final static Collection<PieceLocationDescriptor> blueConfig = new ArrayList<PieceLocationDescriptor>();
	private final Collection<PieceLocationDescriptor> chokeConfig = new ArrayList<PieceLocationDescriptor>();
	private static int boardsize = 0;

	/**
	 * Default private constructor to ensure this is a singleton.
	 */
	private StrategyGameFactory() {
		// Intentionally left empty.
	}
	
	public int getBoardSize() {
		return boardsize;
	}
	
	
	public Collection<PieceLocationDescriptor> getInitialRedConfigurations(){
		return redConfig;
	}
	
	public Collection<PieceLocationDescriptor> getInitialBlueConfigurations(){
		return blueConfig;
	}
	
	public Collection<PieceLocationDescriptor> getChokePointConfigurations(){
		return chokeConfig;
	}
	
	/**
	 * @return the instance
	 */
	public static StrategyGameFactory getInstance() {
		return instance;
	}

	/**
	 * Create an Alpha Strategy game.
	 * 
	 * @return the created Alpha Strategy game
	 */
	public StrategyGameController makeAlphaStrategyGame() {
		boardsize = 2;
		final AlphaStrategyGameController game = new AlphaStrategyGameController();
		// If we have reached this point in the code,
		// our game configuration is valid
		return game;
	}

	/**
	 * Create a new Beta Strategy game given the
	 * 
	 * @param redConfiguration
	 *            the initial starting configuration for the RED pieces
	 * @param blueConfiguration
	 *            the initial starting configuration for the BLUE pieces
	 * @return the Beta Strategy game instance with the initial configuration of
	 *         pieces
	 * @throws StrategyException
	 *             if either configuration is correct
	 */
	public StrategyGameController makeBetaStrategyGame(
			Collection<PieceLocationDescriptor> redConfiguration,
			Collection<PieceLocationDescriptor> blueConfiguration)
			throws StrategyException {
		boardsize = 6;
		final BetaStrategyGameController game = new BetaStrategyGameController();
		
		if ((redConfiguration == null) || (blueConfiguration == null)) {
			throw new StrategyException("Configurations must not be null.");
		} else if ((redConfiguration.size() < 12)
				|| (redConfiguration.size() > 12)) {
			throw new StrategyException(
					"Red does not have the correct number of pieces.");
		} else if ((blueConfiguration.size() < 12)
				|| (blueConfiguration.size() > 12)) {
			throw new StrategyException(
					"Blue does not have the correct number of pieces.");
		}
		
		// Check if a piece not in this version is used in red or blue configuration
		// throw strategy exception
		final Collection<PieceLocationDescriptor> allPieces = new ArrayList<PieceLocationDescriptor>();
		allPieces.addAll(redConfiguration);
		allPieces.addAll(blueConfiguration);
		final Iterator<PieceLocationDescriptor> allPiecesIterator = allPieces.iterator();
		while(allPiecesIterator.hasNext()) {
			PieceType currentPieceType = allPiecesIterator.next().getPiece().getType();
			if ((currentPieceType == PieceType.BOMB) 
					|| (currentPieceType == PieceType.GENERAL)
					|| (currentPieceType == PieceType.MAJOR)
					|| (currentPieceType == PieceType.MINER)
					|| (currentPieceType == PieceType.SCOUT)
					|| (currentPieceType == PieceType.SPY)){
				throw new StrategyException("You cannot create a piece that is not within this version of Strategy.");
			}
		}
		
		// This runs the below tests on red, then blue. This helps readability
		// and reduced
		// the amount of code we needed to write (we could have checked all this
		// for each color)
		for (int i = 0; i < 2; i++) {
			Iterator<PieceLocationDescriptor> pieces = null;
			Collection<Location> multiplePiecesOnLocation = new ArrayList<Location>();
			String currentColor = "";
			int numberOfFlags = 0;
			int numberOfMarshals = 0; // 1
			int numberOfColonels = 0; // 2
			int numberOfCaptains = 0; // 2
			int numberOfLieutenants = 0; // 3
			int numberOfSergeants = 0; // 3
			if (i == 0) {
				// Create the red iterator for the first run through
				pieces = redConfiguration.iterator(); 
				currentColor = "red";
			} else if (i == 1) {
				// Create the blue iterator for the second run through
				pieces = blueConfiguration.iterator(); 
				currentColor = "blue";
				// Reset number of flags
				numberOfFlags = 0;
				numberOfMarshals = 0;
				numberOfColonels = 0;
				numberOfCaptains = 0;
				numberOfLieutenants = 0;
				numberOfSergeants = 0;
			}
			while (pieces.hasNext()) {
				PieceLocationDescriptor next = pieces.next();
				
				// Check if this piece is being placed on the same location as another piece,
				// If not, add this piece to the list for the next iteration
				if (multiplePiecesOnLocation.contains(next.getLocation())) {
					throw new StrategyException("You cannot place multiple pieces on the same location.");
				}
				else multiplePiecesOnLocation.add(next.getLocation());
				
				int XCoordinate = next.getLocation().getCoordinate(Coordinate.X_COORDINATE);
				int YCoordinate = next.getLocation().getCoordinate(Coordinate.Y_COORDINATE);
				if ((XCoordinate < 0) || (XCoordinate > 5) 
						|| ((YCoordinate < 0) && (currentColor.equals("red")))
						|| ((YCoordinate > 1) && (currentColor.equals("red")))
						|| ((YCoordinate < 4) && (currentColor.equals("blue")))
						|| ((YCoordinate > 5) && (currentColor.equals("blue")))
						) {
					throw new StrategyException("A piece is not at a valid location.");
				}
				PieceType currentPiece = next.getPiece().getType();

				switch (currentPiece) {
				case FLAG:
					if (numberOfFlags == 1) {
						throw new StrategyException(
								"A side has too many flags.");
					} else {
						numberOfFlags++;
					}
					break;
				case MARSHAL:
					if (numberOfMarshals == 1) {
						throw new StrategyException(
								"A side has too many marshals.");
					} else {
						numberOfMarshals++;
					}
					break;
				case COLONEL:
					if (numberOfColonels == 2) {
						throw new StrategyException(
								"A side has too many Colonels.");
					} else {
						numberOfColonels++;
					}
					break;
				case CAPTAIN:
					if (numberOfCaptains == 2) {
						throw new StrategyException(
								"A side has too many Captains.");
					} else {
						numberOfCaptains++;
					}
					break;
				case LIEUTENANT:
					if (numberOfLieutenants == 3) {
						throw new StrategyException(
								"A side has too many Lieutenants.");
					} else {
						numberOfLieutenants++;
					}
					break;
				case SERGEANT:
					if (numberOfSergeants == 3) {
						throw new StrategyException(
								"A side has too many Sergeants.");
					} else {
						numberOfSergeants++;
					}
					break;
				default:
					break;
				}
			}
		}
		// Check if this is not the first game being played
		if (redConfig.size() > 0) { // Check if there are elements already configured
			redConfig.clear();
		}
		if (blueConfig.size() > 0) { // Check if there are elements already configured
			blueConfig.clear();
		}
		
		redConfig.addAll(redConfiguration);
		blueConfig.addAll(blueConfiguration);
//		
//		redConfig = redConfiguration;
//		blueConfig = blueConfiguration;
		
		// If we have reached this point in the code,
		// our game configuration is valid
		return game;
	}
	
	/**
	 * Create a new Gamma Strategy game given the
	 * 
	 * @param redConfiguration
	 *            the initial starting configuration for the RED pieces
	 * @param blueConfiguration
	 *            the initial starting configuration for the BLUE pieces
	 * @return the Beta Strategy game instance with the initial configuration of
	 *         pieces
	 * @throws StrategyException
	 *             if either configuration is correct
	 */
	public StrategyGameController makeDeltaStrategyGame(
			Collection<PieceLocationDescriptor> redConfiguration,
			Collection<PieceLocationDescriptor> blueConfiguration)
			throws StrategyException {
		boardsize = 10;
		final DeltaStrategyGameController game = new DeltaStrategyGameController();
		
		if ((redConfiguration == null) || (blueConfiguration == null)) {
			throw new StrategyException("Configurations must not be null.");
		} else if ((redConfiguration.size() < 40)
				|| (redConfiguration.size() > 40)) {
			throw new StrategyException(
					"Red does not have the correct number of pieces.");
		} else if ((blueConfiguration.size() < 40)
				|| (blueConfiguration.size() > 40)) {
			throw new StrategyException(
					"Blue does not have the correct number of pieces.");
		}
		

		final Collection<PieceLocationDescriptor> allPieces = new ArrayList<PieceLocationDescriptor>();
		allPieces.addAll(redConfiguration);
		allPieces.addAll(blueConfiguration);
		final Iterator<PieceLocationDescriptor> allPiecesIterator = allPieces.iterator();
		
		// This runs the below tests on red, then blue. This helps readability
		// and reduced
		// the amount of code we needed to write (we could have checked all this
		// for each color)
		for (int i = 0; i < 2; i++) {
			Iterator<PieceLocationDescriptor> pieces = null;
			Collection<Location> multiplePiecesOnLocation = new ArrayList<Location>();
			String currentColor = "";
			int numberOfFlags = 0;
			int numberOfMarshals = 0; // 1
			int numberOfGenerals = 0; // 1
			int numberOfColonels = 0; // 2
			int numberOfMajors = 0; //3
			int numberOfCaptains = 0; // 2
			int numberOfLieutenants = 0; // 3
			int numberOfSergeants = 0; // 3
			int numberOfBombs = 0; //6
			int numberOfSpy = 0; //1
			int numberOfMiners = 0; //5
			int numberOfScouts = 0; //8
			
			if (i == 0) {
				// Create the red iterator for the first run through
				pieces = redConfiguration.iterator(); 
				currentColor = "red";
			} else if (i == 1) {
				// Create the blue iterator for the second run through
				pieces = blueConfiguration.iterator(); 
				currentColor = "blue";
				// Reset number of flags
				numberOfFlags = 0;
				numberOfMarshals = 0;
				numberOfColonels = 0;
				numberOfCaptains = 0;
				numberOfLieutenants = 0;
				numberOfSergeants = 0;
			}
			while (pieces.hasNext()) {
				PieceLocationDescriptor next = pieces.next();
				
				// Check if this piece is being placed on the same location as another piece,
				// If not, add this piece to the list for the next iteration
				if (multiplePiecesOnLocation.contains(next.getLocation())) {
					throw new StrategyException("You cannot place multiple pieces on the same location.");
				}
				else multiplePiecesOnLocation.add(next.getLocation());
				
				int XCoordinate = next.getLocation().getCoordinate(Coordinate.X_COORDINATE);
				int YCoordinate = next.getLocation().getCoordinate(Coordinate.Y_COORDINATE);
				if (	(YCoordinate < 0)  || (YCoordinate > 9)
						||	(XCoordinate < 0) || (XCoordinate > 9) 
						|| ((YCoordinate > 3) && (currentColor.equals("red")))
						|| ((YCoordinate < 6) && (currentColor.equals("blue")))
						 ) 
						 {
					throw new StrategyException("A piece is not at a valid location.");
				}
				PieceType currentPiece = next.getPiece().getType();

				switch (currentPiece) {
				case FLAG:
					if (numberOfFlags == 1) {
						throw new StrategyException(
								"A side has too many flags.");
					} else {
						numberOfFlags++;
					}
					break;
				case MARSHAL:
					if (numberOfMarshals == 1) {
						throw new StrategyException(
								"A side has too many marshals.");
					} else {
						numberOfMarshals++;
					}
					break;
				case COLONEL:
					if (numberOfColonels == 2) {
						throw new StrategyException(
								"A side has too many Colonels.");
					} else {
						numberOfColonels++;
					}
					break;
				case MAJOR:
					if (numberOfMajors == 3) {
						throw new StrategyException(
								"A side has too many Majors.");
					} else {
						numberOfMajors++;
					}
					break;
				case CAPTAIN:
					if (numberOfCaptains == 4) {
						throw new StrategyException(
								"A side has too many Captains.");
					} else {
						numberOfCaptains++;
					}
					break;
				case LIEUTENANT:
					if (numberOfLieutenants == 4) {
						throw new StrategyException(
								"A side has too many Lieutenants.");
					} else {
						numberOfLieutenants++;
					}
					break;
				case SERGEANT:
					if (numberOfSergeants == 4) {
						throw new StrategyException(
								"A side has too many Sergeants.");
					} else {
						numberOfSergeants++;
					}
					break;
				case MINER:
					if (numberOfMiners == 5) {
						throw new StrategyException(
								"A side has too many Miners.");
					} else {
						numberOfMiners++;
					}
					break;
				case SCOUT:
					if (numberOfScouts == 8) {
						throw new StrategyException(
								"A side has too many Scouts.");
					} else {
						numberOfScouts++;
					}
					break;
				case SPY:
					if (numberOfSpy == 1) {
						throw new StrategyException(
								"A side has too many Spies.");
					} else {
						numberOfSpy++;
					}
					break;
				case BOMB:
					if (numberOfBombs == 6) {
						throw new StrategyException(
								"A side has too many Bombs.");
					} else {
						numberOfBombs++;
					}
					break;
				default:
					break;
				}
			}
		}
		
		
		
		// Check if this is not the first game being played
		if (redConfig.size() > 0) { // Check if there are elements already configured
			redConfig.clear();
		}
		if (blueConfig.size() > 0) { // Check if there are elements already configured
			blueConfig.clear();
		}
		if (chokeConfig.size() > 0){
			chokeConfig.clear();
		}
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(2, 4)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(2, 5)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(3, 4)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(3, 5)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(6, 5)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(6, 4)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(7, 5)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(7, 4)));
		redConfig.addAll(redConfiguration);
		blueConfig.addAll(blueConfiguration);
//		
//		redConfig = redConfiguration;
//		blueConfig = blueConfiguration;
		
		// If we have reached this point in the code,
		// our game configuration is valid
		return game;
	}
	/**
	 * Create a new Gamma Strategy game given the
	 * 
	 * @param redConfiguration
	 *            the initial starting configuration for the RED pieces
	 * @param blueConfiguration
	 *            the initial starting configuration for the BLUE pieces
	 * @return the Beta Strategy game instance with the initial configuration of
	 *         pieces
	 * @throws StrategyException
	 *             if either configuration is correct
	 */
	public StrategyGameController makeGammaStrategyGame(
			Collection<PieceLocationDescriptor> redConfiguration,
			Collection<PieceLocationDescriptor> blueConfiguration)
			throws StrategyException {
		boardsize = 6;
		final GammaStrategyGameController game = new GammaStrategyGameController();
		
		if ((redConfiguration == null) || (blueConfiguration == null)) {
			throw new StrategyException("Configurations must not be null.");
		} else if ((redConfiguration.size() < 12)
				|| (redConfiguration.size() > 12)) {
			throw new StrategyException(
					"Red does not have the correct number of pieces.");
		} else if ((blueConfiguration.size() < 12)
				|| (blueConfiguration.size() > 12)) {
			throw new StrategyException(
					"Blue does not have the correct number of pieces.");
		}
		
		// Check if a piece not in this version is used in red or blue configuration
		// throw strategy exception
		final Collection<PieceLocationDescriptor> allPieces = new ArrayList<PieceLocationDescriptor>();
		allPieces.addAll(redConfiguration);
		allPieces.addAll(blueConfiguration);
		final Iterator<PieceLocationDescriptor> allPiecesIterator = allPieces.iterator();
		while(allPiecesIterator.hasNext()) {
			PieceType currentPieceType = allPiecesIterator.next().getPiece().getType();
			if ((currentPieceType == PieceType.BOMB) 
					|| (currentPieceType == PieceType.GENERAL)
					|| (currentPieceType == PieceType.MAJOR)
					|| (currentPieceType == PieceType.MINER)
					|| (currentPieceType == PieceType.SCOUT)
					|| (currentPieceType == PieceType.SPY)){
				throw new StrategyException("You cannot create a piece that is not within this version of Strategy.");
			}
		}
		
		// This runs the below tests on red, then blue. This helps readability
		// and reduced
		// the amount of code we needed to write (we could have checked all this
		// for each color)
		for (int i = 0; i < 2; i++) {
			Iterator<PieceLocationDescriptor> pieces = null;
			Collection<Location> multiplePiecesOnLocation = new ArrayList<Location>();
			String currentColor = "";
			int numberOfFlags = 0;
			int numberOfMarshals = 0; // 1
			int numberOfColonels = 0; // 2
			int numberOfCaptains = 0; // 2
			int numberOfLieutenants = 0; // 3
			int numberOfSergeants = 0; // 3
			if (i == 0) {
				// Create the red iterator for the first run through
				pieces = redConfiguration.iterator(); 
				currentColor = "red";
			} else if (i == 1) {
				// Create the blue iterator for the second run through
				pieces = blueConfiguration.iterator(); 
				currentColor = "blue";
				// Reset number of flags
				numberOfFlags = 0;
				numberOfMarshals = 0;
				numberOfColonels = 0;
				numberOfCaptains = 0;
				numberOfLieutenants = 0;
				numberOfSergeants = 0;
			}
			while (pieces.hasNext()) {
				PieceLocationDescriptor next = pieces.next();
				
				// Check if this piece is being placed on the same location as another piece,
				// If not, add this piece to the list for the next iteration
				if (multiplePiecesOnLocation.contains(next.getLocation())) {
					throw new StrategyException("You cannot place multiple pieces on the same location.");
				}
				else multiplePiecesOnLocation.add(next.getLocation());
				
				int XCoordinate = next.getLocation().getCoordinate(Coordinate.X_COORDINATE);
				int YCoordinate = next.getLocation().getCoordinate(Coordinate.Y_COORDINATE);
				if ((XCoordinate < 0) || (XCoordinate > 5) 
						|| ((YCoordinate < 0) && (currentColor.equals("red")))
						|| ((YCoordinate > 1) && (currentColor.equals("red")))
						|| ((YCoordinate < 4) && (currentColor.equals("blue")))
						|| ((YCoordinate > 5) && (currentColor.equals("blue")))
						) {
					throw new StrategyException("A piece is not at a valid location.");
				}
				PieceType currentPiece = next.getPiece().getType();

				switch (currentPiece) {
				case FLAG:
					if (numberOfFlags == 1) {
						throw new StrategyException(
								"A side has too many flags.");
					} else {
						numberOfFlags++;
					}
					break;
				case MARSHAL:
					if (numberOfMarshals == 1) {
						throw new StrategyException(
								"A side has too many marshals.");
					} else {
						numberOfMarshals++;
					}
					break;
				case COLONEL:
					if (numberOfColonels == 2) {
						throw new StrategyException(
								"A side has too many Colonels.");
					} else {
						numberOfColonels++;
					}
					break;
				case CAPTAIN:
					if (numberOfCaptains == 2) {
						throw new StrategyException(
								"A side has too many Captains.");
					} else {
						numberOfCaptains++;
					}
					break;
				case LIEUTENANT:
					if (numberOfLieutenants == 3) {
						throw new StrategyException(
								"A side has too many Lieutenants.");
					} else {
						numberOfLieutenants++;
					}
					break;
				case SERGEANT:
					if (numberOfSergeants == 3) {
						throw new StrategyException(
								"A side has too many Sergeants.");
					} else {
						numberOfSergeants++;
					}
					break;
				default:
					break;
				}
			}
		}
		
		
		
		// Check if this is not the first game being played
		if (redConfig.size() > 0) { // Check if there are elements already configured
			redConfig.clear();
		}
		if (blueConfig.size() > 0) { // Check if there are elements already configured
			blueConfig.clear();
		}
		if (chokeConfig.size() > 0){
			chokeConfig.clear();
		}
		
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(2, 2)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(2, 3)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(3, 2)));
		chokeConfig.add(new PieceLocationDescriptor(new Piece(PieceType.CHOKE_POINT, null), new Location2D(3, 3)));
		redConfig.addAll(redConfiguration);
		blueConfig.addAll(blueConfiguration);
//		
//		redConfig = redConfiguration;
//		blueConfig = blueConfiguration;
		
		// If we have reached this point in the code,
		// our game configuration is valid
		return game;
	}
}
