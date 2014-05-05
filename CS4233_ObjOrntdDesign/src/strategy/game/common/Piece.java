/*******************************************************************************
 * This file was developed for CS4233: Object-Oriented Analysis & Design.
 * The course was taken at Worcester Polytechnic Institute.
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *******************************************************************************/

package strategy.game.common;

import strategy.common.PlayerColor;

/**
 * This class is simply a data structure that describes a particular piece in the
 * game.
 * @author cguertin, ranmow
 * @version Sep 7, 2013
 */
public class Piece
{
	private final PieceType type;
	private final PlayerColor owner;
	
	/**
	 * Default constructor
	 * @param type the piece type (rank)
	 * @param owner the color of the player who owns the piece
	 */
	public Piece(PieceType type, PlayerColor owner)
	{
		this.type = type;
		this.owner = owner;
	}

	/**
	 * @return the type
	 */
	public PieceType getType()
	{
		return type;
	}
	/**
	 * This function maps the ranks of each piecetype as follows:		
	 * 	Flag (1)
	 * 	Marshal	(12)
	 * 	Colonels (10)	
	 * 	Captains (8)
	 * 	Lieutenants	(7)
	 * 	Sergeants (6)
	 * @return the rank of a given PieceType
	 */
	public int getRank()
	{

		switch(type) {
		case MARSHAL:
			return 12;
		case GENERAL:
			return 11;
		case COLONEL:
			return 10;
		case MAJOR:
			return 9;
		case CAPTAIN:
			return 8;
		case LIEUTENANT:
			return 7;
		case SERGEANT:
			return 6;
		case MINER:
			return 5;
		case SCOUT:
			return 4;
		case SPY:
			return 3;
		case BOMB:
			return 2;
		case FLAG:
			return 1;
			
		default:
			return 100;
		}
	}

	/**
	 * @return the owner
	 */
	public PlayerColor getOwner()
	{
		return owner;
	}
	
	@Override
	public String toString()
	{
		return owner.toString() + ' ' + type;
	}
	
	@Override
	public boolean equals(Object other) {
		if (other == this) return true;
		if (!(other instanceof Piece)) return false;
		final Piece that = (Piece)other;
		return (type == that.type && owner == that.owner);
	}
	
	@Override
	public int hashCode()
	{
		return type.hashCode() * owner.hashCode();
	}
}
