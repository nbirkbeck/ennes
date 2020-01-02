LIBS = $(shell pkg-config --libs nimage nmath protobuf)
.SUFFIXES = .cc .o

all: extract_colors highres_boundary

extract_colors: extract_colors.cc
	g++ ${LIBS} -DEXTRACT_COLORS_MAIN=1 extract_colors.cc -o extract_colors

%.o: %.cc
	g++ -I${HOME}/code/6502  -fPIC -c $< -o $@	

proto/render_state.pb.cc: render_state.proto
	protoc render_state.proto --cpp_out proto

highres_boundary: highres_boundary.cc extract_colors.o
	g++ -DHIGHRES_BOUNDARY_MAIN ${LIBS} highres_boundary.cc extract_colors.o -o highres_boundary

SIMPLE_RENDER_OBJS = simple_renderer.o proto/render_state.pb.o render_utils.o sprite.o extract_colors.o
simple_renderer: ${SIMPLE_RENDER_OBJS}
	g++ -O3 ${LIBS} ${SIMPLE_RENDER_OBJS} -o simple_renderer 

render_dump.so: render_dump.o proto/render_state.pb.o
	g++ render_dump.o proto/render_state.pb.o ${LIBS} -fPIC -shared -o $@
