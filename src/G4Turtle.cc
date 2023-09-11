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

#include "G4Turtle.hh"
#include "turtle.h"
/* Standard Geant4 includes */
#include "G4EventManager.hh"
#include "G4ExceptionHandler.hh"
#include "G4NistManager.hh"
#include "G4PVPlacement.hh"
#include "G4Version.hh"
#include "G4VGraphicsScene.hh"
#include "G4VisExtent.hh"
#include "G4VoxelLimits.hh"
#include "G4VPVParameterisation.hh"
#include "G4VSolid.hh"
/* Standard library includes */
#include <list>

/* Error handler for TURTLE */
static void handle_turtle(
    enum turtle_return, turtle_function_t *, const char * message)
{
        G4ExceptionDescription description;
        description << G4endl << message << G4endl;
        G4Exception("G4Turtle", "LibraryError",
            FatalException, description);
}

/* Initialise the static data */
static G4VPhysicalVolume * gPhysical = 0;
static G4LogicalVolume * gLogical = 0;
static G4LogicalVolume * gAirLogical = 0;
static G4LogicalVolume * gRockLogical = 0;

static struct turtle_map * gGeoid = 0;
static struct turtle_stack * gStack = 0;
static struct turtle_map * gMap = 0;
static struct turtle_stepper * gStepper = 0;

static G4double gResolutionFactor = 1E-02;
static G4double gSlopeFactor = 0.4;
static G4double gApproximationRange = 1.;

static G4double gBottomLevel = -11.5E+03;
static G4double gTopLevel = 9.5E+03;

enum medium { MEDIUM_UNKNOWN, MEDIUM_AIR, MEDIUM_ROCK, MEDIUM_EXIT,
    MEDIUM_OTHER };
static enum medium gMedium = MEDIUM_UNKNOWN;

#ifdef G4USE_STD11
/* If C++11 is used we can store the daughter data in a Tuple */
typedef std::tuple<G4VPhysicalVolume *, G4VSolid *, G4AffineTransform> Daughter;
#define DAUGHTER_GET_VOLUME(I) std::get<0>(*I)
#define DAUGHTER_GET_SOLID(I) std::get<1>(*I)
#define DAUGHTER_GET_TRANSFORM(I) std::get<2>(*I)
#else
/* Otherwise let us pack the data in a structure */
struct Daughter {
        Daughter(G4VPhysicalVolume * volume, G4VSolid * solid,
            const G4AffineTransform & transform) : fVolume(volume),
            fSolid(solid), fTransform(transform) {}

        G4VPhysicalVolume * fVolume;
        G4VSolid * fSolid;
        G4AffineTransform fTransform;
};
#define DAUGHTER_GET_VOLUME(I) I->fVolume
#define DAUGHTER_GET_SOLID(I) I->fSolid
#define DAUGHTER_GET_TRANSFORM(I) I->fTransform
#endif
static std::list<Daughter> gDaughters;

/* Build a compilation of the daughter volumes and of their transforms */
static void BuildDaughters()
{
        G4int n_daughters = gLogical->GetNoDaughters() - 4;
        if (!n_daughters || !gDaughters.empty()) return;

        for (G4int i = 0; i < n_daughters; i++) {
                G4VPhysicalVolume * physical =
                    gLogical->GetDaughter(i + 4);
                G4VSolid * solid =
                    physical->GetLogicalVolume()->GetSolid();
                /* We use a G4AffineTransform to be consistent with Geant4
                 * navigation
                 */
                G4AffineTransform transform = G4AffineTransform(
                    physical->GetRotation(),
                    physical->GetTranslation());
                transform.Invert();

                gDaughters.push_back(
                    Daughter(physical, solid, transform));
        }
}

/* Get the medium at the given position */
static enum medium GetMedium(double position[3], G4bool protect=false)
{
        enum medium medium = MEDIUM_EXIT;
        double altitude, ground[2];
        int layer[2];
        turtle_stepper_step(gStepper, position, NULL, NULL, NULL, &altitude,
            ground, NULL, layer);

        /* Check the sub-volumes */
        if (!gDaughters.empty()) {
                const double m = CLHEP::m;
                G4ThreeVector global(position[0] * m, position[1] * m,
                    position[2] * m); 

                G4VPhysicalVolume * current = 0;
                if (protect) {
                        /* Get the current volume from the tracking */
                        G4TrackingManager * tracking =
                            G4EventManager::GetEventManager()
                            ->GetTrackingManager();
                        current = tracking->GetSteppingManager()
                            ->GetfCurrentVolume();
                }

                /* Loop over the daughter volumes */
                for (std::list<Daughter>::iterator i = gDaughters.begin();
                    i != gDaughters.end(); ++i) {
                        if (protect && current == DAUGHTER_GET_VOLUME(i)) {
                                /* This is an exit step. To be consistent
                                 * with the Geant4 navigation, the current
                                 * volume is disabled
                                 */
                                continue;
                        }
                        G4ThreeVector local =
                            DAUGHTER_GET_TRANSFORM(i).TransformPoint(global);
                        if (DAUGHTER_GET_SOLID(i)->Inside(local) != kOutside)
                                return MEDIUM_OTHER;
                }
        }

        if ((layer[0] < 0) || (altitude > gTopLevel) || (altitude < gBottomLevel))
                medium = MEDIUM_EXIT;
        else if (layer[0] == 1)
                medium = MEDIUM_AIR;
        else
                medium = MEDIUM_ROCK;

        return medium;
}

/* Get the current medium, protecting against the exiting one according to
 * the Geant4 navigation
 */
static enum medium GetMedium(const G4ThreeVector & p)
{
        BuildDaughters();

        const double m = 1. / CLHEP::m;
        double position[3] = { p[0] * m, p[1] * m, p[2] * m };
        return GetMedium(position, true);
}

/* Get the optimistic step length, taking daughter volumes into account */
static G4double GetStepLength(const G4ThreeVector & p, const G4ThreeVector & v)
{
        BuildDaughters();

        const double m = 1. / CLHEP::m;
        double position0[3] = { p[0] * m, p[1] * m, p[2] * m };

        /* Compute the step length */
        double step;
        turtle_stepper_step(gStepper, position0, NULL, NULL, NULL, NULL,
            NULL, &step, NULL);

        /* Restrict the step for sub-volumes */
        for (std::list<Daughter>::iterator i = gDaughters.begin();
            i != gDaughters.end(); ++i) {
                const G4AffineTransform & transform = DAUGHTER_GET_TRANSFORM(i);
                G4ThreeVector r = transform.TransformPoint(p);
                G4ThreeVector u = transform.TransformAxis(v);

                const G4double d =
                    DAUGHTER_GET_SOLID(i)->DistanceToIn(r, u) / CLHEP::m;
                if (d < step && d > 0.)
                        step = d;
        }

        /* Get the data for the new position. Due to rounding errors we
         * proceed through a G4ThreeVector using Geant units, as if this
         * was a Geant4 step
         */
        const double gstep = step * CLHEP::m;
        G4ThreeVector p1(p[0] + v[0] * gstep, p[1] + v[1] * gstep,
            p[2] + v[2] * gstep);
        double position1[3] = { p1[0] * m, p1[1] * m, p1[2] * m };

        double altitude, ground[2];
        int layer[2];
        turtle_stepper_step(gStepper, position1, NULL, NULL, NULL, &altitude,
            ground, NULL, layer);

        /* Check the end step medium */
        enum medium medium0 = gMedium, medium1 = GetMedium(position1);
        if (medium0 == medium1) return step * CLHEP::m;

        /* Locate the medium change with a binary search */
        double ds0 = 0., ds1 = step;
        while (ds1 - ds0 > 1E-08) {
                const double ds2 = 0.5 * (ds0 + ds1);
                double position2[3] = { position0[0] + v[0] * ds2,
                    position0[1] + v[1] * ds2, position0[2] + v[2] * ds2 };

                enum medium medium2 = GetMedium(position2);
                if (medium2 == medium0) {
                        ds0 = ds2;
                } else {
                        ds1 = ds2;
                }
        }
        step = ds1;

        return step * CLHEP::m;
}

/* Generic solid for encapsulating TURTLE */
class Solid : public G4VSolid
{
  public:
        /* Dummy implementation of the G4VSolid methods */
        Solid() : G4VSolid("G4Turtle::Solid") {};

        virtual G4VSolid * Clone() const
        { return new Solid(*this); }

        virtual G4bool CalculateExtent(const EAxis, const G4VoxelLimits &,
            const G4AffineTransform &, G4double &, G4double &) const
        { return false; }

        virtual G4ThreeVector SurfaceNormal( const G4ThreeVector &) const
        { return G4ThreeVector(0, 0, 0); }

        virtual EInside Inside(const G4ThreeVector &) const
        { return kOutside; }

        virtual G4double DistanceToIn(
            const G4ThreeVector &, const G4ThreeVector &) const
        { return 0; }

        virtual G4double DistanceToIn(const G4ThreeVector &) const
        { return 0; }

        virtual G4double DistanceToOut(const G4ThreeVector &,
            const G4ThreeVector &, const G4bool, G4bool *,
            G4ThreeVector *) const
        { return 0; }

        virtual G4double DistanceToOut(const G4ThreeVector &) const
        { return 0; }

        virtual G4GeometryType GetEntityType() const
        { return G4String("G4Turtle::Solid"); }

        virtual std::ostream& StreamInfo(std::ostream& os) const
        { return os; }

        virtual G4ThreeVector GetPointOnSurface() const
        { return G4ThreeVector(0, 0, 0); }

        virtual void DescribeYourselfTo(G4VGraphicsScene& scene) const
        { scene.AddSolid(*this); }

        virtual G4VisExtent GetExtent() const
        { return G4VisExtent (0, 0, 0, 0, 0, 0); }

        virtual G4Polyhedron * GetPolyhedron () const
        { return 0; }

        virtual G4Polyhedron * CreatePolyhedron() const
        { return 0; }
};

/* Envelope for the World volume */
class Envelope : public Solid
{
  public:
        virtual EInside Inside(const G4ThreeVector & p) const
        {
                gMedium = GetMedium(p);
                return (gMedium == MEDIUM_EXIT) ? kOutside : kInside;
        }

        G4double DistanceToOut(const G4ThreeVector &, const G4ThreeVector &,
            const G4bool calcNorm, G4bool * validNorm, G4ThreeVector *) const
        {
                if (calcNorm) *validNorm = false;
                return (gMedium == MEDIUM_EXIT) ? 0 : kInfinity;
        }

        G4double DistanceToOut(const G4ThreeVector &) const
        {
                return (gMedium == MEDIUM_EXIT) ? 0 : kInfinity;
        }
};

/* Fake volume for a chunk of air */
class AirChunk : public Solid
{
  public:
        virtual EInside Inside(const G4ThreeVector &) const
        {
                if (gMedium == MEDIUM_AIR) return kInside;
                else return kOutside;
        }

        using Solid::DistanceToIn;
        virtual G4double DistanceToIn(
            const G4ThreeVector &, const G4ThreeVector &) const
        {
                if (gMedium == MEDIUM_AIR) return 0;
                else return kInfinity;
        }

        using Solid::DistanceToOut;
        G4double DistanceToOut(const G4ThreeVector & p, const G4ThreeVector & v,
            const G4bool calcNorm, G4bool * validNorm, G4ThreeVector *) const
        {
                if (calcNorm) *validNorm = false;
                if (gMedium != MEDIUM_AIR) return 0;
                else return GetStepLength(p, v);
        }
};

/* Fake volume for a chunk of rock */
class RockChunk : public Solid
{
  public:
        virtual EInside Inside(const G4ThreeVector &) const
        {
                if (gMedium == MEDIUM_ROCK) return kInside;
                else return kOutside;
        }

        using Solid::DistanceToIn;
        virtual G4double DistanceToIn(
            const G4ThreeVector &, const G4ThreeVector &) const
        {
                if (gMedium == MEDIUM_ROCK) return 0;
                else return kInfinity;
        }

        using Solid::DistanceToOut;
        G4double DistanceToOut(const G4ThreeVector & p, const G4ThreeVector & v,
            const G4bool calcNorm, G4bool * validNorm, G4ThreeVector *) const
        {
                if (calcNorm) *validNorm = false;
                if (gMedium != MEDIUM_ROCK) return 0;
                else return GetStepLength(p, v);
        }
};

/* Protected constructor : initialise the static data here since only one
 * instance of this object can exist
 */
G4Turtle::G4Turtle()
{
        /* Register the exception error handler for TURTLE */
        turtle_error_handler_set(&handle_turtle);

        /* Fetch or build the default materials */
        G4NistManager * nist = G4NistManager::Instance();
        const double gcm3 = CLHEP::g / CLHEP::cm3;
        static G4Material airMaterial("G4Turtle::Air", 1.205E-03 * gcm3,
            nist->FindOrBuildMaterial("G4_AIR"), kStateGas);

        static G4Element rockElement("StandardRock",
                "StandardRock", 11., 22.*CLHEP::g / CLHEP::mole);
        const int nElements = (G4VERSION_NUMBER < 1020) ? 2 : 1;
        static G4Material rockMaterial(
            "G4Turtle::Rock", 2.65 * gcm3, nElements, kStateSolid);
        rockMaterial.AddElement(&rockElement, 1);
#if (G4VERSION_NUMBER < 1020)
        /* This is a hack for Standard Rock because of a bug in GEANT4 patched
         * after version 10.1. See:
         * https://bugzilla-geant4.kek.jp/show_bug.cgi?id=1765
         */
        rockMaterial.AddElement(&rockElement, 1);
#endif
        G4IonisParamMat * rockParameters = rockMaterial.GetIonisation();
        rockParameters->SetMeanExcitationEnergy(136.4 * CLHEP::eV);

        /* Build the geometry */
        static Envelope envelopeSolid;
        static G4LogicalVolume envelopeLogical(
            &envelopeSolid, &airMaterial, "G4Turtle::Envelope", 0, 0, 0);
        envelopeLogical.SetOptimisation(false);
        gLogical = &envelopeLogical;
        static G4PVPlacement envelopePhysical(0,
            G4ThreeVector(0., 0., 0.), &envelopeLogical, "G4Turtle::Envelope",
            0, false, 0, 0);
        gPhysical = &envelopePhysical;

        static AirChunk airSolid;
        static G4LogicalVolume airLogical(
            &airSolid, &airMaterial, "G4Turtle::AirChunk", 0, 0, 0);
        airLogical.SetOptimisation(false);
        gAirLogical = &airLogical;
        G4ThreeVector zero(0, 0, 0);
        static G4PVPlacement airPhysical0(0, zero, &airLogical,
            "G4Turtle::AirChunk0", &envelopeLogical, true, 0, 0);
        static G4PVPlacement airPhysical1(0, zero, &airLogical,
            "G4Turtle::AirChunk1", &envelopeLogical, true, 1, 0);

        static RockChunk rockSolid;
        static G4LogicalVolume rockLogical(
            &rockSolid, &rockMaterial, "G4Turtle::RockChunk", 0, 0, 0);
        rockLogical.SetOptimisation(false);
        gRockLogical = &rockLogical;
        static G4PVPlacement rockPhysical0(0, zero, &rockLogical,
            "G4Turtle::RockChunk0", &envelopeLogical, true, 0, 0);
        static G4PVPlacement rockPhysical1(0, zero, &rockLogical,
            "G4Turtle::RockChunk1", &envelopeLogical, true, 1, 0);
}

G4LogicalVolume * G4Turtle::GetLogicalVolume() const
{
        return gLogical;
}

G4VPhysicalVolume * G4Turtle::GetPhysicalVolume() const
{
        return gPhysical;
}

G4LogicalVolume * G4Turtle::GetRockLogicalVolume() const
{
        return gRockLogical;
}

G4LogicalVolume * G4Turtle::GetAirLogicalVolume() const
{
        return gAirLogical;
}

/* Get an instance of the G4turtle */
G4Turtle * G4Turtle::GetInstance()
{
        static G4Turtle instance;
        return &instance;
}

/* Clean the TURTLE allocated data */
static void clean_turtle()
{
        turtle_map_destroy(&gGeoid);
        turtle_map_destroy(&gMap);
        turtle_stack_destroy(&gStack);
        turtle_stepper_destroy(&gStepper);
}

G4Turtle::~G4Turtle()
{
        clean_turtle();
}

G4Material * G4Turtle::GetAirMaterial() const
{
        return gLogical->GetMaterial();
}

G4Material * G4Turtle::GetRockMaterial() const
{
        return gRockLogical->GetMaterial();
}

void G4Turtle::SetAirMaterial(G4Material * material)
{
        gLogical->SetMaterial(material);
        gAirLogical->SetMaterial(material);
}

void G4Turtle::SetRockMaterial(G4Material * material)
{
        gRockLogical->SetMaterial(material);
}

void G4Turtle::SetTopographyData(G4String global, G4String local,
    G4String geoid, G4double default_ground_level)
{
        /* Clean any previous data */
        clean_turtle();

        /* Create the topography providers */
        if (global.length())
                turtle_stack_create(&gStack, global.c_str(), 0, NULL, NULL);
        if (local.length())
                turtle_map_load(&gMap, local.c_str());
        if (geoid.length())
                turtle_map_load(&gGeoid, geoid.c_str());

        /* Create the stepper */
        turtle_stepper_create(&gStepper);
        turtle_stepper_range_set(gStepper, gApproximationRange);
        turtle_stepper_slope_set(gStepper, gSlopeFactor);
        turtle_stepper_resolution_set(gStepper, gResolutionFactor);
        if (gGeoid != NULL)
                turtle_stepper_geoid_set(gStepper, gGeoid);
        if (default_ground_level > -6371E+03)
                turtle_stepper_add_flat(gStepper, default_ground_level);
        if (gStack != NULL)
                turtle_stepper_add_stack(gStepper, gStack, 0);
        if (gMap != NULL)
                turtle_stepper_add_map(gStepper, gMap, 0.);
}

G4double G4Turtle::GetBottomLevel() const
{
        return gBottomLevel * CLHEP::m;
}

void G4Turtle::SetBottomLevel(G4double level)
{
        gBottomLevel = level / CLHEP::m;
}

G4double G4Turtle::GetTopLevel() const
{
        return gTopLevel * CLHEP::m;
}

void G4Turtle::SetTopLevel(G4double level)
{
        gTopLevel = level / CLHEP::m;
}

void G4Turtle::SetSlopeFactor(G4double factor)
{
        gSlopeFactor = factor;
        if (gStepper != NULL)
                turtle_stepper_slope_set(gStepper, gSlopeFactor);
}

G4double G4Turtle::GetSlopeFactor() const
{
        return gSlopeFactor;
}

void G4Turtle::SetResolutionFactor(G4double factor)
{
        gResolutionFactor = factor / CLHEP::m;
        if (gStepper != NULL)
                turtle_stepper_resolution_set(gStepper, gResolutionFactor);
}

G4double G4Turtle::GetResolutionFactor() const
{
        return gResolutionFactor * CLHEP::m;
}

void G4Turtle::SetApproximationRange(G4double range)
{
        gApproximationRange = range / CLHEP::m;
        if (gStepper != NULL)
                turtle_stepper_range_set(gStepper, gApproximationRange);
}

G4double G4Turtle::GetApproximationRange() const
{
        return gApproximationRange * CLHEP::m;
}

G4ThreeVector G4Turtle::GetECEFPosition(
    G4double latitude, G4double longitude, G4double height) const
{
        if (gStepper == 0) {
                G4ExceptionDescription description;
                description << "Missing topography data. "
                            << "Call G4Turtle::SetTopographyData first"
                            << G4endl;
                G4Exception("G4Turtle::GetECEFPosition", "MissingData",
                    FatalException, description);
        }

        double position[3];
        turtle_stepper_position(gStepper, latitude, longitude,
            height / CLHEP::m, 0, position, NULL);

        return G4ThreeVector(position[0], position[1], position[2]) * CLHEP::m;
}

G4ThreeVector G4Turtle::GetECEFDirection(G4double latitude,
    G4double longitude, G4double azimuth, G4double elevation) const
{
        double direction[3];
        const double ideg = 1. / CLHEP::deg;
        turtle_ecef_from_horizontal(latitude, longitude, azimuth * ideg,
            elevation * ideg, direction);

        return G4ThreeVector(direction[0], direction[1], direction[2]);
}

G4PVPlacement * G4Turtle::PVPlacement(G4double latitude, G4double longitude,
    G4double height, G4double azimuth, G4double zenith, G4double intrinsic,
    G4LogicalVolume * logical, const G4String & name, G4bool many, G4int copyNo)
{
        /* Get the translation vector */
        G4ThreeVector translation = this->GetECEFPosition(
            latitude, longitude, height);

        /* Build the rotation matrix */
        const G4double pi2 = 90. * CLHEP::deg;
        G4ThreeVector ux = this->GetECEFDirection(
            latitude, longitude, azimuth - pi2, 0.);
        G4ThreeVector uy = this->GetECEFDirection(
            latitude, longitude, azimuth - 2 * pi2, zenith);
        G4ThreeVector uz = this->GetECEFDirection(
            latitude, longitude, azimuth, pi2 - zenith);
        G4RotationMatrix * rotation = new G4RotationMatrix;
        rotation->setRows(ux, uy, uz);
        if (intrinsic)
                *rotation *= G4RotationMatrix(uz, intrinsic).inverse();

        /* Place the volume */
        return new G4PVPlacement(rotation, translation, logical, name,
            gLogical, many, copyNo, false);
}
