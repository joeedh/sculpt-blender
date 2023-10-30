# Introduction to TriMesh

TriMesh is a triangle mesh kernel loosely modelled on the BMesh API and designed to be
multithreaded.  It has its own memory pool implementation for that purpose.

# Topological Lists

TriMesh stores topological links in simple pointer arrays instead of linked lists.
So the edges around a vertex is stored in v->edges, which is (initially) pool-allocated.
These pointer arrays have the following public api:

<pre>

struct simplelist {
	void **items;
	int length;
};
</ptr>

Note that, like BMesh, no topological constraints is enforced, which is why
TriMesh is not a half-edge data structure (in many ways I would have preferred to use half-edge,
but oh well).

# SafePool

TriMesh uses a new (hopefully) thread-safe memory pool allocator, BLI_safepool.  It stores one pool per thread
and uses a bare minimum of read-write locks to synchronize the structure.  The following rules apply:

# Iterating over the entire pool is allowed only in single-threaded code.
# Freed elements go into the freelist of the thread pool that freed them not the one that allocated them.
# Locks are kept to an absolute bare minimum--un-thread-safe code will crash or deadlock.
# Unlike mempool, elements have a single pointer header that points back at the original allocating pool (if in use)
  or the freeing pool if freed.

# Threaded Mesh Kernel

The concurrent api works like this:

Step 1: Mesh is split into independent islands
Step 2: Geometry along the boundaries of each islands are marked, and will be procesed 
        single-threaded after all threads exit.
Step 3: The islands are fed to a bunch of threads
Step 4: Once all threads exist, the boundary elements are processed on the main thread.

Note that this has not been tested yet because of all the performance bugs I've been finding 
in the single-threaded dyntopo code.

The high-level function for this is:

<pre>
typedef void (*OptTriMeshJob)(TM_TriMesh *tm, void **elements, int totelem, int threadnr, void *userdata);
void TM_foreach_tris(TM_TriMesh *tm, TMFace **tris, int tottri, OptTriMeshJob job, int maxthread, void *userdata);
</pre>

# TableGSet

This is a simple wrapper around GHash that stores a flat pointer array.  It doesn't provide much of
a performance boost--in this branch, where I've basically inlined all of GHash into BLI_ghash.h.  TableGSet
should allow me to revert that.  It would be nice if I could also do away with BLI_hashmap.

# BLI_hashmap

I did not write this because I wanted to.  BLI_hashmap wraps a thread-safe, SIMD C++ hashmap class in C.
It's extremely finicky code; I ended up doing templates-via-C-macros (gross), and the entire endeavor is
highly dependent on the compiler sucessfully inlining some extremely sphagetti-like C++ template code.
GCC and Clang do; msvc does not.

Hopefully I won't end up needing this library.  If I do need it I may try porting some of the features I want 
into my old BLI_smallhash library.  

# CustomData

I'm thinking of adding extremely basic support for customdata interpolation, basically UVs and vertex colors.
These would live in a single function, like so:

<pre>
static void trimesh_customdata_interp(TMElement *e, TMElement *ins, float *ws, int *types, int *offsets, int totelem, int totlayer) {
	for (int i=0; i<totlayer; i++, types++, offsets++) {
		int type = *types;
		int offset = *offsets;
		int size = 0;
			
		void *dst = TM_elem_cd_get(e, offset);

		switch (type) {
			case CD_UV: {
				for (int j=0; j<totelem; j++) {
					//interpolate uv
				}
				break;
			}
			case CD_MCOL: {
				//interpolate mcol
				break;
			}
			case CD_PROP_FLOAT: {
				//interpolate mask
				break;
			}
		}
	}
}
</pre>

It might be worth investigating whether one can tell a C or C++ compiler to take an external C function
and inline it into a switch jump table.

Anyway, this would be extremely (extremely) basic.

# Sculpt Indices

Generic vertex index in the sculpt code now use intptr_t, allowing me to eliminate much of the overhead 
assocated with keeping vertex/face pointer arrays up to date (profiling revealed this to be a major
bottleneck).

# Log

The log code is unfinished.  The key performance improvement I made was to store original 
coordinates/normals in TMVert.  This will have to be updated as undo steps happen.




