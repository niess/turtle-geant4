/*
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org>
 */

#include "run.hh"
/* For the run and UI managers */
#include "G4RunManager.hh"
/* For the detector construction */
#include "G4Turtle.hh"
#include "G4PVPlacement.hh"
#include "G4Box.hh"
/* For the Physics */
#include "G4DecayPhysics.hh"
#include "G4EmStandardPhysics.hh"
#include "G4EmExtraPhysics.hh"
/* For the primary generator */
#include "G4MuonPlus.hh"
#include "G4MuonMinus.hh"
#include "G4ParticleGun.hh"

/* The detector location and orientation */
static G4double gLatitude = 45.76415653;
static G4double gLongitude = 2.95536402;
static G4double gHeight = 0.5 * CLHEP::m;
static G4double gAzimuth = 26. * CLHEP::deg;
static G4double gElevation = 10. * CLHEP::deg;

/* Build the geometry */
G4VPhysicalVolume * DetectorConstruction::Construct()
{
        /* Configure the topography */
        G4Turtle * turtle = G4Turtle::GetInstance();
        turtle->SetTopographyData("share/topography");

        /* Add a detector bounding box filled with air */
        G4double hw = 0.5 * CLHEP::m;
        G4Box * detectorSolid = new G4Box("Detector", hw, hw, hw);
        G4LogicalVolume * detectorLogical =
            new G4LogicalVolume(detectorSolid, turtle->GetAirMaterial(),
            "Detector", 0, 0, 0);

        turtle->PVPlacement(gLatitude, gLongitude, gHeight,
            CLHEP::pi + gAzimuth, gElevation, 0., detectorLogical, "Detector");

        /* Return the topography as world volume */
        return turtle->GetPhysicalVolume();
}

PhysicsList::PhysicsList(G4double cut) : G4VModularPhysicsList()
{
        /* Set the cut value */
        defaultCutValue = cut;

        /* Instanciate the default Physics constructors */
        fDecayPhysics = new G4DecayPhysics();
        fEmPhysics = new G4EmStandardPhysics();
        fExtraPhysics = new G4EmExtraPhysics();
}

PhysicsList::~PhysicsList()
{
        delete fDecayPhysics;
        delete fEmPhysics;
        delete fExtraPhysics;
}

void PhysicsList::ConstructParticle()
{
        fDecayPhysics->ConstructParticle();
}

void PhysicsList::ConstructProcess()
{
        AddTransportation();
        fDecayPhysics->ConstructProcess();
        fEmPhysics->ConstructProcess();
        fExtraPhysics->ConstructProcess();
}

void PhysicsList::SetCuts()
{
        G4int temp = GetVerboseLevel();
        SetVerboseLevel(0);
        SetCutsWithDefault();
        SetVerboseLevel(temp);
}

PrimaryGenerator::PrimaryGenerator() : G4VUserPrimaryGeneratorAction()
{
        fParticleGun = new G4ParticleGun(1);
        fParticleGun->SetParticleDefinition(G4MuonMinus::Definition());
        fParticleGun->SetParticleEnergy(10. * CLHEP::TeV);
}

PrimaryGenerator::~PrimaryGenerator()
{
        delete fParticleGun;
        fParticleGun = NULL;
}

void PrimaryGenerator::GeneratePrimaries(G4Event * event)
{
        G4Turtle * turtle = G4Turtle::GetInstance();
        G4ThreeVector position = turtle->GetECEFPosition(
            gLatitude, gLongitude, gHeight);
        G4ThreeVector direction = turtle->GetECEFDirection(
            gLatitude, gLongitude, gAzimuth, gElevation);
        position += 3E+03 * CLHEP::m * direction;

        fParticleGun->SetParticlePosition(position);
        fParticleGun->SetParticleMomentumDirection(-direction);
        fParticleGun->GeneratePrimaryVertex(event);
}

G4ClassificationOfNewTrack StackingAction::ClassifyNewTrack(
    const G4Track * track)
{
        G4ClassificationOfNewTrack classification = fWaiting;
        G4ParticleDefinition * particleType = track->GetDefinition();

        if ((particleType == G4MuonPlus::MuonPlusDefinition()) ||
                (particleType == G4MuonMinus::MuonMinusDefinition()))
                classification = fUrgent;
        else
                classification = fKill;

        return classification;
}

int main(int argc, char * argv[])
{
        /* Parse any argument(s) */
        G4int events = 1;
        G4int verbosity = 2;
        G4bool killSecondaries = false;
        G4double cut = 1. * CLHEP::m;
        if (argc && --argc) events = (G4int)atoi(*++argv);
        if (argc && --argc) verbosity = (G4int)atoi(*++argv);
        if (argc && --argc) killSecondaries = (G4bool)atoi(*++argv);
        if (argc && --argc) cut = (G4double)atof(*++argv) * CLHEP::m;

        /* Initialize the G4 kernel */
        G4RunManager * run = new G4RunManager;
        run->SetUserInitialization(new DetectorConstruction);
        run->SetUserInitialization(new PhysicsList(cut));
        run->SetUserAction(new PrimaryGenerator);
        if (killSecondaries)
                run->SetUserAction(new StackingAction);
        run->Initialize();

        /* Run the scan */
        G4EventManager * event = G4EventManager::GetEventManager();
        G4TrackingManager * tracking = event->GetTrackingManager();
        tracking->SetVerboseLevel(verbosity);

        run->BeamOn(events);

        /* Free the store etc */
        delete run;
        exit(EXIT_SUCCESS);
}
