// One malloc-family call inside an explicit allocshim collection window.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char** argv)
{
  if (argc != 2)
    return 2;
  void (*begin)(void) = dlsym(RTLD_DEFAULT, "perf_alloc_begin");
  void (*end)(void) = dlsym(RTLD_DEFAULT, "perf_alloc_end");
  void* (*allocate)(size_t) = dlsym(RTLD_DEFAULT, "malloc");
  void* (*zero)(size_t, size_t) = dlsym(RTLD_DEFAULT, "calloc");
  void* (*resize)(void*, size_t) = dlsym(RTLD_DEFAULT, "realloc");
  void* (*align)(size_t, size_t) = dlsym(RTLD_DEFAULT, "memalign");
  void* (*aligned)(size_t, size_t) = dlsym(RTLD_DEFAULT, "aligned_alloc");
  int (*posix)(void**, size_t, size_t) = dlsym(RTLD_DEFAULT, "posix_memalign");
  if (!begin || !end || !allocate || !zero || !resize || !align || !aligned
      || !posix)
    return 2;
  void* previous = allocate(32);
  void* pointer = NULL;
  int error = 0;
  begin();
  if (!strcmp(argv[1], "malloc"))
    pointer = allocate(64);
  else if (!strcmp(argv[1], "calloc"))
    pointer = zero(2, 32);
  else if (!strcmp(argv[1], "realloc")) {
    pointer = resize(previous, 64);
    if (pointer)
      previous = NULL;
  } else if (!strcmp(argv[1], "memalign"))
    pointer = align(64, 64);
  else if (!strcmp(argv[1], "aligned_alloc"))
    pointer = aligned(64, 64);
  else if (!strcmp(argv[1], "posix_memalign"))
    error = posix(&pointer, 64, 64);
  else
    error = 2;
  end();
  int failed = error || !pointer;
  free(previous);
  free(pointer);
  return failed;
}
