FROM andreadotti/geant4-dev:10.4.p02

MAINTAINER niess "https://github.com/niess"

# Get git and some extra dependencies for the vanilla TURTLE library
RUN apt-get update
RUN apt-get install -y git pkg-config libpng-dev libtiff-dev
RUN apt-get clean

# Upload the sources to the container
WORKDIR /usr/local/turtle-geant4
COPY Makefile .
COPY src ./src/
COPY test ./test/

# Fetch the HEAD of the TURTLE library
RUN git clone https://github.com/niess/turtle
