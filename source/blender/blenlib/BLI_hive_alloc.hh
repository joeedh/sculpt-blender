#pragma once

#include "MEM_guardedalloc.h"

#include "BLI_assert.h"
#include "BLI_compiler_attrs.h"
#include "BLI_vector.hh"

#include <algorithm>
#include <type_traits>
#include <vector>

namespace blender {

template<typename T, typename UserData = void *> struct ElemCallbacks {
  static void move_elem(T *old_loc, T *new_loc, UserData userdata, int hive) {}
};

template<typename T> struct ElemSizeOf {
  static size_t size(void *userdata) noexcept
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
    }

    T &operator[](int index) noexcept
    {
      return *reinterpret_cast<T *>(mem + SizeOf::size(userdata) * size_t(index));
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

    ATTR_NO_OPT Hive(UserData userdata_, int index, int reserved = ChunkSize)
        : chunksize(reserved), userdata(userdata_), hive_index(index)
    {
      DynamicChunk chunk(
          MEM_malloc_arrayN(SizeOf::size(userdata), chunksize, "HiveAlloc elements"),
          size_t(chunksize) * SizeOf::size(userdata),
          userdata);

      for (int i = chunksize - 1; i >= 0; i--) {
        freelist.append(i);
        set_free_elem(&chunk[i]);
      }

      chunks.push_back(chunk);
    }

    ATTR_NO_OPT ~Hive()
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
          MEM_freeN(static_cast<void *>(chunk.data()));
        }
      }
    }

    ATTR_NO_OPT Hive(Hive &&b) noexcept
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

    ATTR_NO_OPT void free(T *elem)
    {
      elem->~T();
      set_free_elem(elem);
      used--;

      for (int i : IndexRange(chunks.size())) {
        DynamicChunk &chunk = chunks[i];

        if (elem >= chunk.data() && elem < chunk.end()) {
          char *c1 = reinterpret_cast<char *>(chunk.data());
          char *c2 = reinterpret_cast<char *>(elem);
          int idx = int(size_t(c2 - c1) / SizeOf::size(userdata));

          chunk.used--;

          freelist.append(i * chunksize + idx);
          return;
        }
      }

      printf("%s: error!\n", __func__);
      BLI_assert_unreachable();
    }

    ATTR_NO_OPT bool has_elem(T *elem, int *r_chunk_i = nullptr)
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

    ATTR_NO_OPT void append_chunk()
    {
      DynamicChunk chunk(MEM_malloc_arrayN(SizeOf::size(userdata), chunksize, "hive chunk"),
                         SizeOf::size(userdata) * chunksize,
                         userdata);

      int start = chunks.size() * chunksize;
      int end = (chunks.size() + 1) * chunksize;
      int j = chunksize - 1;

      for (int i = end - 1; i >= start; i--, j--) {
        freelist.append(i);
        set_free_elem(&chunk[j]);
      }

      chunks.push_back(chunk);
    }

    ATTR_NO_OPT void set_free_elem(T *elem)
    {
      /* Make sure to corrupt BMHeader.htype by setting first 16 bytes
       * to 255. This is used as a way to detect freed elements.
       */
      if (SizeOf::size(userdata) > 16) {
        memset(static_cast<void *>(elem), 255, 16);
      }

      char *ptr = reinterpret_cast<char *>(elem);
      ptr[0] = freeword[0];
      ptr[1] = freeword[1];
      ptr[2] = freeword[2];
      ptr[3] = freeword[3];
    }

    ATTR_NO_OPT void clear_free_elem(T *elem)
    {
      char *ptr = reinterpret_cast<char *>(elem);
      ptr[0] = ptr[1] = ptr[2] = ptr[3] = 0;
    }

    bool is_free(T *elem)
    {
      char *ptr = reinterpret_cast<char *>(elem);
      for (int i = 0; i < 4; i++) {
        if (ptr[i] != freeword[i]) {
          return false;
        }
      }

      return true;
    }

    ATTR_NO_OPT void set_userdata(UserData data)
    {
      userdata = data;

      for (DynamicChunk &chunk : chunks) {
        chunk.set_userdata(data);
      }
    }

    size_t get_mem_size()
    {
      return chunks.size() * chunksize * SizeOf::size(userdata);
    }

    ATTR_NO_OPT bool compact()
    {
      std::vector<DynamicChunk> chunks2;
      bool modified = false;

      for (DynamicChunk &chunk2 : chunks) {
        if (chunk2.used != 0) {
          chunks2.push_back(chunk2);
        }
      }

      if (chunks.size() != chunks2.size()) {
        printf("HiveAllocator::Hive::compact: pruned %d chunks\n", chunks.size() - chunks2.size());

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

  int get_hive(T *elem)
  {
    int i = 0;
    for (Hive &hive : hives) {
      if (hive.has_elem(elem)) {
        return i;
      }
      i++;
    }

    return -1;
  }

 public:
  class Iterator {
    int hive_i;
    int chunk_i;
    int elem_i;
    HiveAllocator *alloc;

   public:
    ATTR_NO_OPT Iterator(HiveAllocator *alloc_, int hive_i_, int elem_i_)
        : hive_i(hive_i_), chunk_i(0), elem_i(elem_i_), alloc(alloc_)
    {
      /* Find first element. */
      if (hive_i == 0 && elem_i == 0) {
        elem_i--;
        Iterator::operator++();
      }
    }

    ATTR_NO_OPT Iterator(const Iterator &b)
        : hive_i(b.hive_i), elem_i(b.elem_i), chunk_i(b.chunk_i), alloc(b.alloc)
    {
    }

    ATTR_NO_OPT Iterator &operator=(const Iterator &b)
    {
      alloc = b.alloc;
      hive_i = b.hive_i;
      chunk_i = b.chunk_i;
      elem_i = b.elem_i;

      return *this;
    }

    bool done()
    {
      return hive_i >= alloc->hives.size();
    }

    ATTR_NO_OPT bool operator==(const Iterator &b)
    {
      return b.hive_i == hive_i && b.elem_i == elem_i && b.chunk_i == chunk_i;
    }
    ATTR_NO_OPT bool operator!=(const Iterator &b)
    {
      return !(this->operator==(b));
    }

    ATTR_NO_OPT const Iterator &operator++()
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

    ATTR_NO_OPT T *operator*()
    {
      return &alloc->hives[hive_i].chunks[chunk_i][elem_i];
    }

   public:
  };

  ATTR_NO_OPT HiveAllocator(UserData data) : userdata(data) {}

  ATTR_NO_OPT Iterator begin()
  {
    return Iterator(this, 0, 0);
  }

  ATTR_NO_OPT Iterator end()
  {
    return Iterator(this, hives.size(), 0);
  }

  ATTR_NO_OPT void add_hive()
  {
    hives.push_back(Hive(userdata, hives.size()));
  }

  Hive &get_hive(int hive)
  {
    return hives[hive];
  }

  ATTR_NO_OPT int hives_count()
  {
    return hives.size();
  }

  void ensure_hives(int count)
  {
    while (hives.size() < count) {
      add_hive();
    }
  }

  ATTR_NO_OPT T *alloc(int hive = -1)
  {
    if (hives.size() == 0) {
      add_hive();
    }

    if (hive == -1) {
      // XXX rotate through the hives?
      hive = 0;
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

  ATTR_NO_OPT void free(T *elem)
  {
    for (Hive &hive : hives) {
      if (hive.has_elem(elem)) {
        hive.free(elem);
        return;
      }
    }

    BLI_assert_unreachable();
  }

  ATTR_NO_OPT T *move(T *elem, int new_hive)
  {
    int old_hive = get_hive(elem);

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

  ATTR_NO_OPT T *at_index(int index)
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

  ATTR_NO_OPT void set_userdata(UserData data)
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

  ATTR_NO_OPT bool compact()
  {
    bool modified = false;

    for (Hive &hive : hives) {
      modified |= hive.compact();
    }

    return modified;
  }
};
}  // namespace blender
