#pragma once

#include "MEM_guardedalloc.h"

#include "BLI_asan.h"
#include "BLI_assert.h"
#include "BLI_compiler_attrs.h"
#include "BLI_vector.hh"

#include <algorithm>
#include <type_traits>
#include <vector>

namespace blender {

#ifdef WITH_ASAN
constexpr int AsanPad = 16;
#endif

template<typename T, typename UserData = void *> struct ElemCallbacks {
  static void move_elem(T *old_loc, T *new_loc, UserData userdata, int hive) {}
};

template<typename T> struct ElemSizeOf {
  static size_t size(void *) noexcept
  {
    return sizeof(T);
  }
};

/**
 * Hive-based allocator.  Elements are allocated in distinct "hives"
 * and can be moved between them.
 */
template<typename T,
         typename Callbacks = ElemCallbacks<T>,
         typename UserData = void *,
         int ChunkSize = 4096,
         typename SizeOf = ElemSizeOf<T>>
class HiveAllocator {
  static constexpr char freeword[5] = "FREE";
  UserData userdata;

  class DynamicChunk {
    char *mem;
    size_t size;
    UserData userdata;

   public:
    int used = 0;

    DynamicChunk(void *mem_, size_t size_, UserData userdata_) noexcept
        : mem(static_cast<char *>(mem_)), size(size_), userdata(userdata_)
    {
#ifdef WITH_ASAN
      BLI_asan_poison(mem_, size_);
#endif
    }

    DynamicChunk(const DynamicChunk &b) noexcept
    {
      mem = b.mem;
      userdata = b.userdata;
      size = b.size;
      used = b.used;
    }

    DynamicChunk &operator=(const DynamicChunk &b) noexcept
    {
      mem = b.mem;
      userdata = b.userdata;
      size = b.size;
      used = b.used;

      return *this;
    }

    T &operator[](int index) noexcept
    {
#ifdef WITH_ASAN
      size_t size = SizeOf::size(userdata) + AsanPad;
      return *reinterpret_cast<T *>(mem + size * size_t(index));
#else
      return *reinterpret_cast<T *>(mem + SizeOf::size(userdata) * size_t(index));
#endif
    }

    void free_data()
    {
      if (mem) {
        MEM_freeN(static_cast<void *>(mem));
        mem = nullptr;
      }
    }

    T *data()
    {
      return reinterpret_cast<T *>(mem);
    }

    T *end()
    {
      return reinterpret_cast<T *>(mem + size);
    }

    void set_userdata(UserData data)
    {
      userdata = data;
    }
  };

  struct Hive {
    std::vector<DynamicChunk> chunks;
    int chunksize;
    Vector<int> freelist;
    UserData userdata;
    int hive_index;
    int used = 0;

    Hive(UserData userdata_, int index, int reserved = ChunkSize)
        : chunksize(reserved), userdata(userdata_), hive_index(index)
    {
#ifdef WITH_ASAN
      size_t size = SizeOf::size(userdata) + AsanPad;
#else
      size_t size = SizeOf::size(userdata);
#endif

      DynamicChunk chunk(MEM_malloc_arrayN(size, chunksize, "HiveAlloc elements"),
                         size_t(chunksize) * size,
                         userdata);

      for (int i = chunksize - 1; i >= 0; i--) {
        freelist.append(i);
        set_free_elem(&chunk[i]);
      }

      chunks.push_back(chunk);
    }

    ~Hive()
    {
      for (DynamicChunk &chunk : chunks) {
        if (!chunk.data()) {
          continue;
        }

        for (int i = 0; i < chunksize; i++) {
          if (!is_free(&chunk[i])) {
            chunk[i].~T();
          }
        }

        if (chunk.data()) {
          chunk.free_data();
        }
      }
    }

    Hive(Hive &&b) noexcept
    {
      userdata = std::move(b.userdata);
      chunks = std::move(b.chunks);
      freelist = std::move(b.freelist);
      chunksize = b.chunksize;
      hive_index = b.hive_index;
      used = b.used;

      b.chunksize = 0;
    }

    Hive(const Hive &b) = delete;

    T *alloc()
    {
      used++;

      if (freelist.size() == 0) {
        append_chunk();
      }

      int i = freelist.pop_last();
      int chunk_i = i / chunksize;
      i = i % chunksize;

      chunks[chunk_i].used++;
      T *ret = &chunks[chunk_i][i];

      BLI_assert(is_free(ret));

      clear_free_elem(ret);
      return new (reinterpret_cast<void *>(ret)) T();
    }

    void free(T *elem)
    {
      if (is_free(elem)) {
        printf("%s: error, double free!\n", __func__);
        return;
      }

      elem->~T();
      set_free_elem(elem);
      used--;

      for (int i : IndexRange(chunks.size())) {
        DynamicChunk &chunk = chunks[i];

        if (elem >= chunk.data() && elem < chunk.end()) {
          char *c1 = reinterpret_cast<char *>(chunk.data());
          char *c2 = reinterpret_cast<char *>(elem);
#ifdef WITH_ASAN
          size_t size = SizeOf::size(userdata) + AsanPad;
#else
          size_t size = SizeOf::size(userdata);
#endif
          int idx = int(size_t(c2 - c1) / size);

          chunk.used--;

          freelist.append(i * chunksize + idx);
          return;
        }
      }

      printf("%s: error!\n", __func__);
      BLI_assert_unreachable();
    }

    bool has_elem(const T *elem, int *r_chunk_i = nullptr)
    {
      int i = 0;
      for (DynamicChunk &chunk : chunks) {
        if (elem >= chunk.data() && elem < chunk.end()) {
          if (r_chunk_i) {
            *r_chunk_i = i;
          }
          return true;
        }

        i++;
      }

      return false;
    }

    void append_chunk()
    {
#ifdef WITH_ASAN
      size_t size = SizeOf::size(userdata) + AsanPad;
#else
      size_t size = SizeOf::size(userdata);
#endif

      DynamicChunk chunk(
          MEM_malloc_arrayN(size, chunksize, "hive chunk"), size * chunksize, userdata);

      int start = chunks.size() * chunksize;
      int end = (chunks.size() + 1) * chunksize;
      int j = chunksize - 1;

      for (int i = end - 1; i >= start; i--, j--) {
        freelist.append(i);
        set_free_elem(&chunk[j]);
      }

      chunks.push_back(chunk);
    }

    void set_free_elem(T *elem)
    {
      void *ptr = static_cast<void *>(elem);

#ifdef WITH_ASAN
      BLI_asan_unpoison(ptr, SizeOf::size(userdata));
#endif

      /* Make sure to corrupt BMHeader.htype by setting first 16 bytes
       * to 255. This is used as a way to detect freed elements.
       */
      if (SizeOf::size(userdata) >= 16) {
        memcpy(POINTER_OFFSET(ptr, 4), static_cast<const void *>("WORDDEADBADD"), 12);
      }

      memcpy(ptr, static_cast<const void *>(freeword), 4);

#ifdef WITH_ASAN
      BLI_asan_poison(ptr, SizeOf::size(userdata));
#endif
    }

    void clear_free_elem(T *elem)
    {
#ifdef WITH_ASAN
      BLI_asan_unpoison(static_cast<void *>(elem), SizeOf::size(userdata));
#endif

      char *ptr = reinterpret_cast<char *>(elem);
      ptr[0] = ptr[1] = ptr[2] = ptr[3] = 0;
    }

    bool is_free(const T *elem)
    {
#ifdef WITH_ASAN
      BLI_asan_unpoison(static_cast<void *>(const_cast<T *>(elem)), 4);
#endif

      const char *ptr = reinterpret_cast<const char *>(elem);
      for (int i = 0; i < 4; i++) {
        if (ptr[i] != freeword[i]) {
          return false;
        }
      }

#ifdef WITH_ASAN
      BLI_asan_poison(static_cast<void *>(const_cast<T *>(elem)), 4);
#endif
      return true;
    }

    void set_userdata(UserData data)
    {
      userdata = data;

      for (DynamicChunk &chunk : chunks) {
        chunk.set_userdata(data);
      }
    }

    size_t get_mem_size()
    {
      size_t size = SizeOf::size(userdata);
#ifdef WITH_ASAN
      size += AsanPad;
#endif

      return chunks.size() * chunksize * size;
    }

    bool compact()
    {
      /* Don't remove last chunk to avoid memory thrashing. */
      if (chunks.size() <= 1) {
        return false;
      }

      std::vector<DynamicChunk> chunks2;
      bool modified = false;

      for (DynamicChunk &chunk2 : chunks) {
        if (chunk2.used != 0) {
          chunks2.push_back(chunk2);
        }
      }

      if (chunks.size() != chunks2.size()) {
        printf("HiveAllocator::Hive::compact: pruned %d chunks\n",
               int(chunks.size() - chunks2.size()));

        /* Regenerate free list */
        freelist.clear_and_shrink();
        int elem_i = 0;

        for (DynamicChunk &chunk : chunks2) {
          for (int i = 0; i < chunksize; i++, elem_i++) {
            if (is_free(&chunk[i])) {
              freelist.append(elem_i);
            }
          }
        }

        modified = true;
      }

      chunks = chunks2;
      return modified;
    }
  };

  std::vector<Hive> hives;

 public:
  class Iterator {
    int hive_i;
    int chunk_i;
    int elem_i;
    HiveAllocator *alloc;

   public:
    Iterator(HiveAllocator *alloc_, int hive_i_, int elem_i_)
        : hive_i(hive_i_), chunk_i(0), elem_i(elem_i_), alloc(alloc_)
    {
      /* Find first element. */
      if (hive_i == 0 && elem_i == 0) {
        elem_i--;
        Iterator::operator++();
      }
    }

    Iterator(const Iterator &b)
        : hive_i(b.hive_i), elem_i(b.elem_i), chunk_i(b.chunk_i), alloc(b.alloc)
    {
    }

    Iterator &operator=(const Iterator &b)
    {
      alloc = b.alloc;
      hive_i = b.hive_i;
      chunk_i = b.chunk_i;
      elem_i = b.elem_i;

      return *this;
    }

    bool done()
    {
      if (hive_i < alloc->hives.size() && alloc->hives[hive_i].is_free(**this)) {
        printf("eek!\n");
        *this = operator++();
      }

      return hive_i >= alloc->hives.size();
    }

    bool operator==(const Iterator &b)
    {
      return b.hive_i == hive_i && b.elem_i == elem_i && b.chunk_i == chunk_i;
    }
    bool operator!=(const Iterator &b)
    {
      return !(this->operator==(b));
    }

    const Iterator &operator++()
    {
      while (hive_i < alloc->hives.size()) {
        Hive *hive = &alloc->hives[hive_i];

        elem_i++;
        if (elem_i >= hive->chunksize) {
          elem_i = 0;
          chunk_i++;

          if (chunk_i >= hive->chunks.size()) {
            chunk_i = 0;
            hive_i++;
          }

          if (hive_i >= alloc->hives.size()) {
            break;
          }

          hive = &alloc->hives[hive_i];
        }

        if (!hive->is_free(&hive->chunks[chunk_i][elem_i])) {
          return *this;
        }
      }

      return *this;
    }

    T *operator*()
    {
      return &alloc->hives[hive_i].chunks[chunk_i][elem_i];
    }

   public:
  };

  HiveAllocator(UserData data) : userdata(data) {}

  Iterator begin()
  {
    return Iterator(this, 0, 0);
  }

  Iterator end()
  {
    return Iterator(this, hives.size(), 0);
  }

  void add_hive()
  {
    hives.push_back(Hive(userdata, hives.size()));
  }

  Hive &get_hive(int hive)
  {
    return hives[hive];
  }

  int find_hive(const T *elem)
  {
    if (hives.size() == 1) {
      return 0;
    }

    int i = 0;
    for (Hive &hive : hives) {
      if (hive.has_elem(elem)) {
        return i;
      }
      i++;
    }

    printf("Elem not in hive\n");

    return -1;
  }
  int hives_count()
  {
    return hives.size();
  }

  int max_hive()
  {
    return hives.size() == 0 ? 0 : hives.size() - 1;
  }

  void ensure_hives(int count)
  {
    while (hives.size() < count) {
      add_hive();
    }
  }

  T *alloc(int hive = 0)
  {
    if (hives.size() == 0) {
      add_hive();
    }

    return hives[hive].alloc();
  }

  int get_chunk(T *elem)
  {
    int i = 0;
    for (Hive &hive : hives) {
      i += hive.chunks.size();
      int i_out = 0;

      if (hive.has_elem(elem, &i_out)) {
        return i + i_out;
      }
    }
    return -1;
  }

  void free(T *elem)
  {
    // printf("free\n");

    for (Hive &hive : hives) {
      if (hive.has_elem(elem)) {
        hive.free(elem);
        return;
      }
    }

    BLI_assert_unreachable();
  }

  T *move(T *elem, int new_hive)
  {
    int old_hive = find_hive(elem);

    if (old_hive == new_hive) {
      return elem;
    }

    BLI_assert(old_hive != -1);

    T *new_elem = hives[new_hive].alloc();
    *new_elem = *elem;

    Callbacks::move_elem(elem, new_elem, userdata, new_hive);
    hives[old_hive].free(elem);

    return new_elem;
  }

  T *at_index(int index)
  {
    int i = 0;

    for (T *elem : *this) {
      if (i == index) {
        return elem;
      }
      i++;
    }

    return nullptr;
  }

  void set_userdata(UserData data)
  {
    userdata = data;

    for (Hive &hive : hives) {
      hive.set_userdata(data);
    }
  }

  size_t get_mem_size()
  {
    size_t sum = 0;

    for (Hive &hive : hives) {
      sum += hive.get_mem_size();
    }

    return sum;
  }

  bool compact()
  {
    bool modified = false;

    for (Hive &hive : hives) {
      modified |= hive.compact();
    }

    return modified;
  }
};
}  // namespace blender
