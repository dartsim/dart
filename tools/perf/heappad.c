// GNU libc heap perturbations. HEAPPAD selects start4k/start100k,
// size16/size48, or random1/random2 (xorshift padding in 16-byte units).
#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern void* __libc_malloc(size_t);
extern void* __libc_calloc(size_t, size_t);
extern void* __libc_realloc(void*, size_t);
extern void* __libc_memalign(size_t, size_t);

static size_t extra;
static atomic_uint_fast64_t seed;
static void* volatile startup;

__attribute__((constructor)) static void initialize(void)
{
  const char* mode = getenv("HEAPPAD");
  if (!mode)
    return;
  if (!strcmp(mode, "start4k"))
    startup = __libc_malloc(4096);
  else if (!strcmp(mode, "start100k"))
    startup = __libc_malloc(100000);
  else if (!strcmp(mode, "size16"))
    extra = 16;
  else if (!strcmp(mode, "size48"))
    extra = 48;
  else if (!strcmp(mode, "random1"))
    atomic_store(&seed, 1);
  else if (!strcmp(mode, "random2"))
    atomic_store(&seed, 2);
  else
    abort();
}

static size_t padding(void)
{
  uint_fast64_t previous = atomic_load_explicit(&seed, memory_order_relaxed);
  if (!previous)
    return extra;
  uint_fast64_t next;
  do {
    next = previous;
    next ^= next << 13;
    next ^= next >> 7;
    next ^= next << 17;
  } while (!atomic_compare_exchange_weak_explicit(
      &seed, &previous, next, memory_order_relaxed, memory_order_relaxed));
  return (next % 64) * 16;
}

static size_t padded(size_t size)
{
  size_t pad = padding();
  if (size > SIZE_MAX - pad) {
    errno = ENOMEM;
    return 0;
  }
  return size + pad;
}

void* malloc(size_t size)
{
  size_t count = padded(size);
  return count < size ? NULL : __libc_malloc(count);
}

void* calloc(size_t count, size_t size)
{
  if (size && count > SIZE_MAX / size) {
    errno = ENOMEM;
    return NULL;
  }
  size_t total = count * size;
  size_t with_pad = padded(total);
  return with_pad < total ? NULL : __libc_calloc(1, with_pad);
}

void* realloc(void* pointer, size_t size)
{
  if (!size)
    return __libc_realloc(pointer, 0);
  size_t count = padded(size);
  return count < size ? NULL : __libc_realloc(pointer, count);
}

static void* padded_memalign(size_t alignment, size_t size)
{
  size_t count = padded(size);
  return count < size ? NULL : __libc_memalign(alignment, count);
}

void* memalign(size_t alignment, size_t size)
{
  return padded_memalign(alignment, size);
}

void* aligned_alloc(size_t alignment, size_t size)
{
  return padded_memalign(alignment, size);
}

int posix_memalign(void** output, size_t alignment, size_t size)
{
  if (alignment < sizeof(void*) || (alignment & (alignment - 1)))
    return EINVAL;
  void* pointer = padded_memalign(alignment, size);
  if (!pointer)
    return ENOMEM;
  *output = pointer;
  return 0;
}
