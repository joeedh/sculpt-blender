#pragma once

#include "MEM_guardedalloc.h"

#include "BLI_assert.h"
#include "BLI_vector.hh"
#include <algorithm>
#include <vector>

namespace blender {

template<typename T, typename UserData = void *> struct ElemCallbacks {
  static void move_elem(T *old_loc, T *new_loc, UserData userdata, int hive) {}
};

/**
 * Hive-based allocator.  Elements are allocated in distinct "hives"
 * and can be moved between them.
 */
template<typename T,
         typename Callbacks = ElemCallbacks<T>,
         typename UserData = void *,
         int DefaultSize = 512>
class HiveAllocator {
  static constexpr char freeword[5] = "FREE";
  UserData userdata;

  struct Hive {
    T *elements;
    Vector<int> freelist;
    int size;

   public:
    Hive(int reserved = DefaultSize) : size(reserved)
    {
      elements = static_cast<T *>(MEM_malloc_arrayN(sizeof(T), reserved, "HiveAlloc elements"));

      for (int i = 0; i < size; i++) {
        freelist.append(i);
        set_free_elem(&elements[i]);
      }
    }

    ~Hive()
    {
      for (int i = 0; i < size; i++) {
        if (!is_free(elements + i)) {
          elements[i].~T();
        }
      }

      MEM_SAFE_FREE(elements);
    }

    Hive(Hive &&b)
    {
      elements = b.elements;
      freelist = std::move(freelist);
      size = b.size;

      b.elements = nullptr;
    }

    Hive(const Hive &b) = delete;

    T *alloc()
    {
      int i;

      if (freelist.size() > 0) {
        i = freelist.pop_last();
      }
      else {
        realloc();
        i = freelist.pop_last();
      }

      T *ret = elements + i;
      BLI_assert(is_free(ret));

      clear_free_elem(ret);
      return new (reinterpret_cast<void *>(ret)) T();
    }

    void free(T *elem)
    {
      elem->~T();
      set_free_elem(elem);
      freelist.append(elem - elements);
    }

    bool has_elem(T *elem)
    {
      return elem > elements && elem < elements + size;
    }

    void realloc()
    {
      int new_size = (size + 1) << 1;

      elements = static_cast<T *>(MEM_malloc_arrayN(sizeof(T), new_size, "HiveAlloc elements"));

      for (int i = 0; i < size; i++) {
        T *old_elem = elements + i;
        T *new_elem = new_elems + i;

        if (!is_free(old_elem)) {
          Callbacks.move_elem(old_elem, new_elem);
        }
        else {
          set_free_elem(new_elem);
        }
      }

      for (int i = size; i < new_size; i++) {
        freelist.append(i);
        set_free_elem(new_elems + i);
      }

      if (elements) {
        delete[] elements;
      }
      size = new_size;
      elements = new_elems;
    }

    void set_free_elem(T *elem)
    {
      char *ptr = reinterpret_cast<char *>(elem);
      ptr[0] = freeword[0];
      ptr[1] = freeword[1];
      ptr[2] = freeword[2];
      ptr[3] = freeword[3];
    }

    void clear_free_elem(T *elem)
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
    int elem_i;
    HiveAllocator *alloc;

   public:
    Iterator(HiveAllocator *alloc_, int hive_i_, int elem_i_)
        : hive_i(hive_i_), elem_i(elem_i_), alloc(alloc_)
    {
    }

    Iterator(const Iterator &b) : hive_i(b.hive_i), elem_i(b.elem_i), alloc(b.alloc) {}

    Iterator &operator=(const Iterator &b)
    {
      alloc = b.alloc;
      hive_i = b.hive_i;
      elem_i = b.elem_i;

      return *this;
    }

    bool done()
    {
      return hive_i >= alloc->hives.size();
    }

    bool operator==(const Iterator &b)
    {
      return b.hive_i == hive_i && b.elem_i == elem_i;
    }
    bool operator!=(const Iterator &b)
    {
      return !(this->operator==(b));
    }

    const Iterator &operator++()
    {
      while (hive_i < alloc->hives.size()) {
        Hive &hive = alloc->hives[hive_i];

        elem_i++;
        if (elem_i > hive.size) {
          elem_i = 0;
          hive_i++;
          continue;
        }

        if (!hive.is_free(hive.elements + elem_i)) {
          return *this;
        }
      }

      return *this;
    }

    T *operator*()
    {
      return alloc->hives[hive_i].elements + elem_i;
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
    hives.push_back(Hive());
  }

  int hives_count()
  {
    return hives.size();
  }

  T *alloc(int hive)
  {
    return hives[i].alloc();
  }

  void free(T *elem)
  {
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
    int old_hive = get_hive(elem);

    if (old_hive == new_hive) {
      return;
    }

    BLI_assert(old_hive != -1);

    T *new_elem = new_hive.alloc();
    Callbacks.move_elem(elem, new_elem);
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
};
}  // namespace blender
