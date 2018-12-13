CC := gcc
CXX := g++
CFLAGS := -O3 -Wall -Wshadow -Iturtle/include -Wno-unused-result
CXXFLAGS := $(CFLAGS) $(shell geant4-config --cflags)
CFLAGS += -Iturtle/src
LIBS := -Llib -lG4Turtle $(shell geant4-config --libs) 

# Flags for supported data formats
TURTLE_USE_GRD := 1
TURTLE_USE_HGT := 1
TURTLE_USE_PNG := 1
TURTLE_USE_TIFF := 1

.PHONY: clean lib test

lib: lib/libG4Turtle.a

test: bin/test-run

clean:
	@rm -rf bin build lib

MODS := client ecef error io list map projection stack stepper tinydir

ifeq ($(TURTLE_USE_GRD), 1)
	MODS += grd
else
	CFLAGS += -DTURTLE_NO_GRD
endif

# Flag for HGT files
ifeq ($(TURTLE_USE_HGT), 1)
	MODS += hgt
else
	CFLAGS += -DTURTLE_NO_HGT
endif

ifeq ($(TURTLE_USE_PNG), 1)
	CFLAGS += $(shell pkg-config --cflags libpng)
	MODS += jsmn png16
	LIBS += -lpng
else
	CFLAGS += -DTURTLE_NO_PNG
endif

ifeq ($(TURTLE_USE_TIFF), 1)
	MODS += geotiff16
	LIBS += -ltiff
else
	CFLAGS += -DTURTLE_NO_TIFF
endif

lib/libG4Turtle.a: $(addsuffix .o, $(addprefix build/, G4Turtle $(MODS)))
	@echo "Building library $@"
	@mkdir -p lib
	@ar -r $@ $^

define build
	@echo "Building object $@"
	@mkdir -p build
	@$(1) -o $@ $(2) -c $<
endef

build/%.o: turtle/src/turtle/%.c turtle/src/turtle/%.h
	@$(call build, $(CC), $(CFLAGS))

build/%.o: turtle/src/turtle/%.c
	@$(call build, $(CC), $(CFLAGS))

build/%.o: turtle/src/turtle/io/%.c
	@$(call build, $(CC), $(CFLAGS))

build/%.o: turtle/src/deps/%.c turtle/src/deps/%.h
	@$(call build, $(CC), $(CFLAGS))

build/%.o: src/%.cc src/%.hh
	@$(call build, $(CXX), $(CXXFLAGS))

bin/test-%: test/%.cc test/%.hh lib/libG4Turtle.a
	@echo "Building binary $@"
	@mkdir -p bin
	@$(CXX) -o $@ $(CXXFLAGS) -Isrc $< $(LIBS)
