// Native allocation counts inside the collection window, excluding warmup.
// GNU libc only; World::step interposition or explicit stepAndRead hooks.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern void* __libc_malloc(size_t);
extern void* __libc_calloc(size_t, size_t);
extern void* __libc_realloc(void*, size_t);
extern void* __libc_memalign(size_t, size_t);

static void* (*next_malloc)(size_t);
static void* (*next_calloc)(size_t, size_t);
static void* (*next_realloc)(void*, size_t);
static void* (*next_memalign)(size_t, size_t);
static void* (*next_aligned_alloc)(size_t, size_t);
static int (*next_posix_memalign)(void**, size_t, size_t);
static atomic_int in_step;
static atomic_ulong allocations, bytes;
static unsigned long steps, measured, skip;
static const char* library = "unresolved";
static void (*real_step)(void*, _Bool);
static int explicit_window;

static void resolve_step(void)
{
  real_step = dlsym(RTLD_NEXT, "_ZN4dart10simulation5World4stepEb");
  Dl_info info;
  if (real_step && dladdr((void*)real_step, &info))
    library = info.dli_fname;
}

// Resolve outside the measured region. During dlsym recursion use libc;
// otherwise forward to heappad if it follows us in LD_PRELOAD.
__attribute__((constructor)) static void initialize(void)
{
  next_malloc = dlsym(RTLD_NEXT, "malloc");
  next_calloc = dlsym(RTLD_NEXT, "calloc");
  next_realloc = dlsym(RTLD_NEXT, "realloc");
  next_memalign = dlsym(RTLD_NEXT, "memalign");
  next_aligned_alloc = dlsym(RTLD_NEXT, "aligned_alloc");
  next_posix_memalign = dlsym(RTLD_NEXT, "posix_memalign");
  const char* warmup = getenv("PERF_WARMUP");
  skip = warmup ? strtoul(warmup, NULL, 10) : 0;
  const char* window = getenv("PERF_WINDOW");
  explicit_window = window && strcmp(window, "stepAndRead") == 0;
  // Micro rows may never step, but must still identify their loaded libdart.
  resolve_step();
}

static void note(size_t size)
{
  if (atomic_load_explicit(&in_step, memory_order_relaxed)) {
    atomic_fetch_add_explicit(&allocations, 1, memory_order_relaxed);
    atomic_fetch_add_explicit(&bytes, size, memory_order_relaxed);
  }
}

void* malloc(size_t size)
{
  note(size);
  return next_malloc ? next_malloc(size) : __libc_malloc(size);
}

void* calloc(size_t count, size_t size)
{
  note(count * size);
  return next_calloc ? next_calloc(count, size) : __libc_calloc(count, size);
}

void* realloc(void* pointer, size_t size)
{
  note(size);
  return next_realloc ? next_realloc(pointer, size)
                      : __libc_realloc(pointer, size);
}

void* memalign(size_t alignment, size_t size)
{
  note(size);
  return next_memalign ? next_memalign(alignment, size)
                       : __libc_memalign(alignment, size);
}

void* aligned_alloc(size_t alignment, size_t size)
{
  note(size);
  return next_aligned_alloc ? next_aligned_alloc(alignment, size)
                            : __libc_memalign(alignment, size);
}

int posix_memalign(void** output, size_t alignment, size_t size)
{
  note(size);
  if (next_posix_memalign)
    return next_posix_memalign(output, alignment, size);
  if (alignment < sizeof(void*) || (alignment & (alignment - 1)))
    return EINVAL;
  void* pointer = __libc_memalign(alignment, size);
  if (!pointer)
    return ENOMEM;
  *output = pointer;
  return 0;
}

void perf_alloc_begin(void)
{
  if (steps >= skip)
    atomic_fetch_add_explicit(&in_step, 1, memory_order_relaxed);
}

void perf_alloc_end(void)
{
  if (steps >= skip) {
    atomic_fetch_sub_explicit(&in_step, 1, memory_order_relaxed);
    ++measured;
  }
  ++steps;
}

// void dart::simulation::World::step(bool)
void _ZN4dart10simulation5World4stepEb(void* world, _Bool reset)
{
  if (!real_step) {
    resolve_step();
    if (!real_step) {
      fputs("STEPALLOC cannot resolve World::step\n", stderr);
      abort();
    }
  }
  if (!explicit_window)
    perf_alloc_begin();
  real_step(world, reset);
  if (!explicit_window)
    perf_alloc_end();
}

__attribute__((destructor)) static void report(void)
{
  fprintf(
      stderr,
      "STEPALLOC steps=%lu measured=%lu allocs=%lu bytes=%lu libdart=%s\n",
      steps,
      measured,
      atomic_load(&allocations),
      atomic_load(&bytes),
      library);
}
