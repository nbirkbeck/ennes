#include "proto/render_state.pb.h"
#include "render_utils.h"
#include <stdio.h>
#include <string>

int main(int ac, char* av[]) {
  nes::RenderSequence all;
  for (int i = 1; i < ac; ++i) {
    nes::RenderSequence seq;
    if (!LoadRenderSequence(av[i], &seq)) {
      break;
    }
    all.MergeFrom(seq);
  }
  std::string bytes;
  all.SerializeToString(&bytes);

  FILE* file = fopen("/tmp/merged.pb", "w");
  fwrite(bytes.data(), 1, bytes.size(), file);
  fclose(file);
  return 0;
}
