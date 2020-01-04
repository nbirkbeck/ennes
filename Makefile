LIBS = $(shell pkg-config --libs nimage nmath protobuf)
.SUFFIXES = .cc .o
INC=-I/home/birkbeck/svn/misc -I${HOME}/svn

FLAGS=-O3

all: extract_colors highres_boundary mesh_utils simple_renderer renderer

extract_colors: extract_colors.cc
	g++ ${LIBS} -DEXTRACT_COLORS_MAIN=1 extract_colors.cc -o extract_colors

%.o: %.cc
	g++ ${FLAGS} ${INC} -I${HOME}/code/6502  -fPIC -c $< -o $@	

render_dump.so: render_dump.o proto/render_state.pb.o
	g++ render_dump.o proto/render_state.pb.o ${LIBS} -fPIC -shared -o $@

proto/render_state.pb.cc: render_state.proto
	protoc render_state.proto --cpp_out proto

highres_boundary: highres_boundary.cc extract_colors.o
	g++ -DHIGHRES_BOUNDARY_MAIN ${LIBS} highres_boundary.cc extract_colors.o -o highres_boundary

SIMPLE_RENDER_OBJS = simple_renderer.o proto/render_state.pb.o render_utils.o sprite.o extract_colors.o screen.o nes.o
simple_renderer: ${SIMPLE_RENDER_OBJS}
	g++ ${FLAGS} ${LIBS} ${SIMPLE_RENDER_OBJS} -o simple_renderer 

mesh_utils: mesh_utils.cc
	g++ ${FLAGS} mesh_utils.cc -DMESH_UTILS_MAIN=1 ${LIBS} $(shell pkg-config --libs levset) ~/svn/misc/poly/cpp/delaunay.o	-I/home/birkbeck/svn/misc -o mesh_utils

RENDER_OBJS = renderer.o proto/render_state.pb.o render_utils.o sprite.o extract_colors.o highres_boundary.o mesh_utils.o screen.o nes.o
renderer: ${RENDER_OBJS}
	g++ ${FLAGS} ${LIBS} ${RENDER_OBJS} -lglut -lGL -lGLU -lGLEW  $(shell pkg-config --libs levset) ~/svn/misc/poly/cpp/delaunay.o  -o renderer

clean:
	rm -fv *.o

