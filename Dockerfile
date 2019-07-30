FROM andreadotti/geant4-dev:10.4.p02

MAINTAINER niess "https://github.com/niess"

# Get git and some extra dependencies for the vanilla TURTLE library
RUN apt-get update
RUN apt-get install -y git pkg-config libpng-dev libtiff-dev
RUN apt-get clean

# Upload the sources to the container
WORKDIR /usr/local/turtle-geant4
COPY CMakeLists.txt .
COPY src ./src/
COPY test ./test/
COPY .travis ./.travis/
