/*******************************************************************************
 * This files was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

package strategy.game.testutil;

import static strategy.common.PlayerColor.*;
import static strategy.game.common.PieceType.*;
import java.util.*;
import strategy.common.PlayerColor;
import strategy.game.common.*;

/**
 * A single place to get configurations for the various tests.
 * 
 * @author gpollice
 * @version Sep 21, 2013
 */
public class TestConfigurationFactory
{
	private static TestConfigurationFactory instance = new TestConfigurationFactory();
	
	private ArrayList<PieceLocationDescriptor> currentConfiguration = null;
	
	private TestConfigurationFactory()
	{
		// Intentionally empty
	}
	
	public ArrayList<PieceLocationDescriptor> getRedBetaConfiguration()
	{
		currentConfiguration = new ArrayList<PieceLocationDescriptor>();
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
		return currentConfiguration;
	}
	
	public ArrayList<PieceLocationDescriptor> getBlueBetaConfiguration()
	{
		currentConfiguration = new ArrayList<PieceLocationDescriptor>();
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
		return currentConfiguration;
	}
	
	public ArrayList<PieceLocationDescriptor> getRedGammaConfiguration()
	{
		return getRedBetaConfiguration();
	}
	
	public ArrayList<PieceLocationDescriptor> getBlueGammaConfiguration()
	{
		return getBlueBetaConfiguration();
	}

	/**
	 * @return the instance
	 */
	public static TestConfigurationFactory getInstance()
	{
		return instance;
	}
	
	// Helper methods
	private void addToConfiguration(PieceType type, PlayerColor color, int x, int y)
	{
		final PieceLocationDescriptor confItem = new PieceLocationDescriptor(
				new Piece(type, color), new Location2D(x, y));
		if (color == PlayerColor.RED) {
			currentConfiguration.add(confItem);
		} else {
			currentConfiguration.add(confItem);
		}
	}
}
