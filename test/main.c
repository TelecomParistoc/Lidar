#include "test_map.h"
#include <stdio.h>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    printf("Invalid usage: %s <filename>", argv[0]);
    return 1;
  }

  test_map(argv[1]);
}
