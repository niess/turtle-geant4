/*
 * Copyright (C) 2018 Université Clermont Auvergne, CNRS/IN2P3, LPC
 * Author: Valentin NIESS (niess@in2p3.fr)
 *
 * Topographic Utilities for tRansporting parTicules over Long rangEs (TURTLE)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef G4TURTLE_HH
#define G4TURTLE_HH

#include "G4LogicalVolume.hh"
#include "G4Material.hh"
#include "G4Types.hh"
#include "G4PVPlacement.hh"

class G4Turtle
{
  public:
        virtual ~G4Turtle();

        /* Static getter */
        static G4Turtle * GetInstance();

        /* Geant4 API */
        G4LogicalVolume * GetLogicalVolume() const;
        G4VPhysicalVolume * GetPhysicalVolume() const;
        G4Material * GetAirMaterial() const;
        G4Material * GetRockMaterial() const;
        void SetAirMaterial(G4Material * material);
        void SetRockMaterial(G4Material * material);

        /* Geometry API */
        void SetTopographyData(G4String global="", G4String local="",
            G4String geoid="", G4double default_ground_level=-kInfinity);
        void SetBottomLevel(G4double level);
        G4double GetBottomLevel() const;
        void SetTopLevel(G4double level);
        G4double GetTopLevel() const;

        /* Stepping algorithm API */
        void SetSlopeFactor(G4double factor);
        G4double GetSlopeFactor() const;
        void SetResolutionFactor(G4double factor);
        G4double GetResolutionFactor() const;
        void SetApproximationRange(G4double range);
        G4double GetApproximationRange() const;

        /* ECEF conversion utilities */
        G4ThreeVector GetECEFPosition(
            G4double latitude, G4double longitude, G4double height) const;
        G4ThreeVector GetECEFDirection(G4double latitude,
            G4double longitude, G4double azimuth, G4double elevation) const;

        /* Utility for the placement of physical sub volumes. The azimuth
         * and zenith angles specify the orientation of the z-axis w.r.t.
         * the local vertical, i.e. azimuth = zenith = 0 is vertical. The
         * intrinsic rotation angle specifies a counter clock wise rotation
         * around the rotated z-axis */
        G4PVPlacement * PVPlacement(G4double latitude, G4double longitude,
            G4double height, G4double azimuth, G4double zenith,
            G4double intrinsic, G4LogicalVolume * logical,
            const G4String & name, G4bool many = false, G4int copyNo = 0);

  protected:
        G4Turtle();
};

#endif
