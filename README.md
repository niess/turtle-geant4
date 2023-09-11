[![Turtle version](https://img.shields.io/badge/turtle-v0.11-blue.svg)](https://github.com/niess/turtle/releases/v0.11)
[![Build Status](https://travis-ci.com/niess/turtle-geant4.svg?branch=master)](https://travis-ci.com/niess/turtle-geant4)

# G4TURTLE
( **G**eant**4** **T**opographic **U**tilities for t**R**ansporting par**T**icules over **L**ong rang**E**s )

## Description
This is a [Geant4][Geant4] wrapper of the [TURTLE][TURTLE] library. It allows to
efficiently import topographic data in [Geant4][Geant4] as a custom
[G4VPhysicalVolume][G4VPhysicalVolume]. The [TURTLE][TURTLE] library is
encapsulated in a [G4Turtle][G4Turtle] class. Below is a minimal
example for building a topography as a Geant4 geometry with [TURTLE][TURTLE]:

```C++
#include "G4Turtle.hh"

G4VPhysicalVolume * DetectorConstruction::Construct()
{
        /* Fetch the topography */
        G4Turtle * turtle = G4Turtle::GetInstance();
        turtle->SetTopographyData("path/to/global/model");

        /* Return the topography as world volume */
        return turtle->GetPhysicalVolume();
}
```

Note that the Geant4 geometry for the topography is emulated from the data,
on the fly. Therefore it can't be visualized. This mechanism however allows
to handle very large topographic data sets.

Extra volumes can be placed inside the topography with the
`G4Turtle::PVPlacement` method, e.g. a detector. The daughter volumes are placed
by specifying their geographic coordinates, i.e. latitude, longitude, height
above ground, etc. Note that they prevail over the topography.

## Installation

Building the [G4Turtle][G4Turtle] class requires a vanilla Geant4 installation,
including both header files and libraries. Builds have been successfully tested
with v10.0 to v10.5 of Geant4, e.g. using [Ubuntu Docker images](https://hub.docker.com/r/andreadotti/geant4-dev/).

A [CMakeLists.txt](CMakeLists.txt) file is provided for building both
[TURTLE][TURTLE] and [G4Turtle][G4Turtle] as shared libraries. This can be done
for example as:
```bash
cd g4turtle
mkdir build && cd build
cmake ..
make install
```

- By default the installation is done to the `g4turtle` source directory, under
  `lib` and `include`. You can specify a different install location as:
  ```bash
  cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install
  ```

- The [test](test) executable can be built as:
  ```bash
  make run-g4goupil
  ```

- The default build of [TURTLE][TURTLE] requires `libpng-dev` and `libtiff-dev`
  for reading GeoTIFF and PNG topography data. On Debian and derivatives (e.g.
  Ubuntu) these dependencies can be installed as:
  ```bash
  sudo apt install pkg-config libpng-dev libtiff-dev
  ```

- Alternatively, you can build your own version of [TURTLE][TURTLE], e.g. with
  GEOTIFF disabled and link to it as:
  ```bash
  cmake .. -DTURTLE_PREFIX=/path/to/turtle/install
  ```


## License

The TURTLE library and the [G4Turtle][G4Turtle] wrapper are under the **GNU
LGPLv3** license. See the provided [`LICENSE`](LICENSE) and
[`COPYING.LESSER`](COPYING.LESSER) files. The [test](test) program however has a
separate public domain license allowing it to be copied without restrictions.

[Geant4]: https://geant4.web.cern.ch/
[G4VPhysicalVolume]: http://www.apc.univ-paris7.fr/~franco/g4doxy/html/classG4VPhysicalVolume.html
[TURTLE]: https://niess.github.io/turtle-pages
[G4Turtle]: src/G4Turtle.hh
