LIBS = $(shell pkg-config --libs nimage nmath)
.SUFFIXES = .cc .o

all: extract_colors highres_boundary

extract_colors: extract_colors.cc
	g++ ${LIBS} -DEXTRACT_COLORS_MAIN=1 extract_colors.cc -o extract_colors

%.o: %.cc
	g++ -c $< -o $@	

highres_boundary: highres_boundary.cc extract_colors.o
	g++ ${LIBS} highres_boundary.cc extract_colors.o -o highres_boundary
