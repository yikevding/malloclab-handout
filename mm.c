/*
 * mm.c - Dynamic memory allocator using an implicit free list.
 *
 * BLOCK STRUCTURE:
 *   Each block has a 4-byte header and a 4-byte footer, both storing
 *   the block's total size (including overhead) and a 1-bit allocation flag.
 *   Layout: [Header(4B)] [Payload...] [Footer(4B)]
 *   All block sizes are multiples of 8 (double-word aligned).
 *   Minimum block size is 16 bytes (header + 8B payload + footer).
 *
 * FREE LIST ORGANIZATION:
 *   Implicit free list — ALL blocks (free and allocated) are linked
 *   by physical adjacency. Traversal starts from heap_listp (prologue
 *   footer) and walks forward via NEXT_BLKP until the epilogue (size 0).
 *
 * HEAP STRUCTURE:
 *   [4B padding] [8B prologue block] [free/alloc blocks...] [4B epilogue]
 *   The prologue (always allocated, size=8) and epilogue (always allocated,
 *   size=0) simplify boundary conditions in coalesce.
 *
 * ALLOCATION STRATEGY:
 *   - mm_malloc: best-fit search across the entire heap to minimize
 *     fragmentation, then extend_heap if no fit is found.
 *   - mm_free: immediate coalescing with adjacent free neighbors (4 cases).
 *   - mm_realloc: in-place shrink/expand before falling back to
 *     allocate-copy-free.
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

/*********************************************************
 * NOTE TO STUDENTS: Before you do anything else, please
 * provide your team information in the following struct.
 ********************************************************/
team_t team = {
    /* Team name */
    "ateam",
    /* First member's full name */
    "Harry Bovik",
    /* First member's email address */
    "bovik@cs.cmu.edu",
    /* Second member's full name (leave blank if none) */
    "",
    /* Second member's email address (leave blank if none) */
    ""};

/* Basic constants */
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define WSIZE 4             /* Word and header/footer size (bytes) */
#define DSIZE 8             /* Double word size (bytes) */
#define CHUNKSIZE (1 << 12) /* Extend heap by this amount (bytes) */

/* Alignment */
#define ALIGNMENT 8
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~0x7)
#define SIZE_T_SIZE (ALIGN(sizeof(size_t)))

/* Basic heap operations */
#define GET(p) (*(unsigned int *)(p))
#define PUT(p, val) (*(unsigned int *)(p) = (val))
#define PACK(size, alloc) ((size) | (alloc))

/* Unpack size and allocation bit from header/footer */
#define GET_SIZE(p) (GET(p) & ~0x7)
#define GET_ALLOC(p) (GET(p) & 0x1)

/* Given block ptr bp, compute address of its header and footer */
#define HDRP(bp) ((char *)(bp) - WSIZE)
#define FTRP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

/* Given block ptr bp, compute address of next and previous blocks */
#define NEXT_BLKP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)))
#define PREV_BLKP(bp) ((char *)(bp) - GET_SIZE((char *)(bp) - DSIZE))

/* Global variable: pointer to first block of heap */
static char *heap_listp = 0;

/*
 * coalesce - Merge bp with any adjacent free blocks.
 *
 * Uses the four standard cases based on the alloc bits of the previous
 * and next blocks (read from their footers/headers before any writes):
 *   Case 1: both neighbors allocated   → no merge, return bp
 *   Case 2: only prev is free          → merge with prev, return prev
 *   Case 3: only next is free          → merge with next, return bp
 *   Case 4: both neighbors are free    → merge all three, return prev
 *
 * IMPORTANT: NEXT_BLKP is evaluated before updating HDRP(bp) in cases
 * 3 and 4, since it depends on the current block size stored in HDRP(bp).
 */
static void *coalesce(void *bp)
{
    size_t current_size = GET_SIZE(HDRP(bp));
    int prev_allocated = GET_ALLOC(FTRP(PREV_BLKP(bp)));
    int next_allocated = GET_ALLOC(HDRP(NEXT_BLKP(bp)));

    if (prev_allocated && next_allocated) /* Case 1: no free neighbors */
    {
        return bp;
    }
    else if (!prev_allocated && next_allocated) /* Case 2: prev is free */
    {
        size_t new_size = current_size + GET_SIZE(HDRP(PREV_BLKP(bp)));
        PUT(HDRP(PREV_BLKP(bp)), PACK(new_size, 0));
        PUT(FTRP(bp), PACK(new_size, 0));
        return PREV_BLKP(bp);
    }
    else if (prev_allocated && !next_allocated) /* Case 3: next is free */
    {
        /* Must read NEXT_BLKP before writing HDRP(bp), since NEXT_BLKP
         * uses the size stored in HDRP(bp) to compute the next address. */
        size_t new_size = current_size + GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(new_size, 0));
        PUT(HDRP(bp), PACK(new_size, 0));
        return bp;
    }
    else /* Case 4: both neighbors are free */
    {
        /* Capture NEXT_BLKP before any writes to avoid stale pointer. */
        size_t new_size = current_size + GET_SIZE(HDRP(PREV_BLKP(bp))) + GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(PREV_BLKP(bp)), PACK(new_size, 0));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(new_size, 0));
        return PREV_BLKP(bp);
    }
}

/*
 * place - place an allocated block of asize bytes at the start of free block bp.
 * Split the block if the remainder is large enough to be a free block (>= 2*DSIZE).
 */
static void place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp)); /* total size of the free block */

    if ((csize - asize) >= (2 * DSIZE))
    {
        /* Split: allocate the first asize bytes, leave remainder as free */
        PUT(HDRP(bp), PACK(asize, 1));         /* allocated header */
        PUT(FTRP(bp), PACK(asize, 1));         /* allocated footer */
        bp = NEXT_BLKP(bp);                    /* move bp to remainder block */
        PUT(HDRP(bp), PACK(csize - asize, 0)); /* free header for remainder */
        PUT(FTRP(bp), PACK(csize - asize, 0)); /* free footer for remainder */
    }
    else
    {
        /* No split: use the entire block, accept internal fragmentation */
        PUT(HDRP(bp), PACK(csize, 1));
        PUT(FTRP(bp), PACK(csize, 1));
    }
}
/*
 * extend_heap - Extend the heap by `words` words and return a pointer
 * to the new free block.
 *
 * Rounds words up to an even count to preserve double-word alignment.
 * mem_sbrk returns a pointer to the old epilogue location, which becomes
 * the header of the new free block. A new epilogue is written at the end.
 * Calls coalesce to merge with any preceding free block.
 */
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size;

    /* Round up to even word count to maintain 8-byte alignment */
    size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;

    /* mem_sbrk returns pointer to start of new region (old epilogue position) */
    if ((bp = mem_sbrk(size)) == (void *)-1)
        return NULL;

    /* Overwrite old epilogue with new free block header and footer */
    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));

    /* Plant new epilogue at end of heap */
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1));

    /* Coalesce with previous free block if applicable */
    return coalesce(bp);
}
/*
 * mm_init - Initialize the heap with a prologue block and one initial free chunk.
 *
 * Heap layout after init:
 *   [4B padding] [4B prologue hdr] [4B prologue ftr] [4B epilogue]
 *            ^-- heap_listp points here (prologue footer)
 * Then extend_heap is called to add the initial CHUNKSIZE free block.
 * Returns 0 on success, -1 on error.
 */
int mm_init(void)
{
    if ((heap_listp = mem_sbrk(4 * WSIZE)) == (void *)-1)
        return -1;
    PUT(heap_listp, 0);                          /* Alignment padding */
    PUT(heap_listp + WSIZE, PACK(DSIZE, 1));     /* Prologue header */
    PUT(heap_listp + 2 * WSIZE, PACK(DSIZE, 1)); /* Prologue footer */
    PUT(heap_listp + 3 * WSIZE, PACK(0, 1));     /* Epilogue header */
    heap_listp += DSIZE;                         /* Point to prologue footer */
    if (extend_heap(CHUNKSIZE / WSIZE) == NULL)
        return -1;
    return 0;
}

/*
 * mm_malloc - Allocate a block of at least `size` payload bytes.
 *
 * Adjusts size up to the nearest multiple of DSIZE (min 2*DSIZE) to
 * include header/footer overhead and satisfy alignment. Performs a
 * best-fit search over all blocks starting from heap_listp. If no
 * fit is found, extends the heap by MAX(adjusted_size, CHUNKSIZE).
 * Returns a pointer to the allocated payload, or NULL on failure.
 */
void *mm_malloc(size_t size)
{
    size_t adjusted_block_size;
    if (size == 0)
        return NULL;

    if (size <= DSIZE)
        adjusted_block_size = 2 * DSIZE;
    else
        adjusted_block_size = DSIZE * ((size + DSIZE + (DSIZE - 1)) / DSIZE);

    /* Search for best-fit free block: smallest block that is large enough */
    void *bp;
    void *best_fit = NULL;
    size_t best_size = (size_t)-1; /* largest possible value */

    for (bp = heap_listp; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp))
    {
        size_t block_size = GET_SIZE(HDRP(bp));
        if (!GET_ALLOC(HDRP(bp)) && block_size >= adjusted_block_size)
        {
            if (block_size < best_size) /* smaller fit found */
            {
                best_size = block_size;
                best_fit = bp;
            }
        }
    }

    if (best_fit != NULL)
    {
        place(best_fit, adjusted_block_size);
        return best_fit;
    }

    // if not best fit we need to extend the heap
    size_t extend_size = MAX(adjusted_block_size, CHUNKSIZE);
    if ((bp = extend_heap(extend_size / WSIZE)) == NULL)
        return NULL;
    place(bp, adjusted_block_size);
    return bp;
}

/*
 * mm_free - Free the block pointed to by ptr.
 *
 * Marks the block's header and footer as unallocated (alloc bit = 0),
 * then immediately coalesces with adjacent free neighbors to prevent
 * fragmentation. No-op if ptr is NULL.
 */
void mm_free(void *ptr)
{
    if (ptr == NULL)
        return;

    size_t size = GET_SIZE(HDRP(ptr));
    PUT(HDRP(ptr), PACK(size, 0));
    PUT(FTRP(ptr), PACK(size, 0));
    coalesce(ptr);
}

/*
 * mm_realloc - Resizes block at ptr to size bytes.
 * Tries in-place expansion before falling back to allocate-copy-free.
 */
void *mm_realloc(void *ptr, size_t size)
{
    /* Case 1: ptr == NULL is equivalent to malloc */
    if (ptr == NULL)
        return mm_malloc(size);

    /* Case 2: size == 0 is equivalent to free */
    if (size == 0)
    {
        mm_free(ptr);
        return NULL;
    }

    size_t old_size = GET_SIZE(HDRP(ptr));

    /* Compute adjusted new size (same formula as mm_malloc) */
    size_t new_size;
    if (size <= DSIZE)
        new_size = 2 * DSIZE;
    else
        new_size = DSIZE * ((size + DSIZE + (DSIZE - 1)) / DSIZE);

    /* Case 3: same size or shrinking — return in place */
    if (new_size <= old_size)
    {
        /* Only split if remainder is large enough to be a valid free block */
        if ((old_size - new_size) >= (2 * DSIZE))
        {
            PUT(HDRP(ptr), PACK(new_size, 1));
            PUT(FTRP(ptr), PACK(new_size, 1));
            void *remainder = NEXT_BLKP(ptr);
            PUT(HDRP(remainder), PACK(old_size - new_size, 0));
            PUT(FTRP(remainder), PACK(old_size - new_size, 0));
            coalesce(remainder);
        }
        return ptr;
    }

    /* Case 4: growing — try in-place expansion */
    void *next = NEXT_BLKP(ptr);
    size_t next_size = GET_SIZE(HDRP(next));
    int next_alloc = GET_ALLOC(HDRP(next));

    /* Case 4a: next block is free and combined size is sufficient */
    if (!next_alloc && (old_size + next_size) >= new_size)
    {
        size_t combined = old_size + next_size;
        PUT(HDRP(ptr), PACK(combined, 1));
        PUT(FTRP(ptr), PACK(combined, 1));
        return ptr;
    }

    /* Case 4b: next block is epilogue — extend heap by exact difference */
    if (next_size == 0)
    {
        size_t extend_size = new_size - old_size;
        if (mem_sbrk(extend_size) == (void *)-1)
            return NULL;
        PUT(HDRP(ptr), PACK(old_size + extend_size, 1));
        PUT(FTRP(ptr), PACK(old_size + extend_size, 1));
        PUT(HDRP(NEXT_BLKP(ptr)), PACK(0, 1)); /* new epilogue */
        return ptr;
    }

    /* Case 4c: cannot expand in place — allocate new, copy, free old */
    void *newptr = mm_malloc(size);
    if (newptr == NULL)
        return NULL;
    memcpy(newptr, ptr, old_size - DSIZE); /* copy only payload bytes */
    mm_free(ptr);
    return newptr;
}

/*
 * mm_check - Heap consistency checker.
 *
 * Scans every block in the heap and verifies the following invariants:
 *   1. Every block is 8-byte aligned.
 *   2. Every block's header and footer match (same size and alloc bit).
 *   3. No two consecutive free blocks exist (missed coalescing).
 *   4. Every free block has size >= 2*DSIZE (minimum valid block size).
 *   5. Every block pointer lies within the heap boundaries.
 *   6. The prologue is intact (size=8, allocated).
 *   7. The epilogue is intact (size=0, allocated).
 *
 * Returns 1 (nonzero) if the heap is consistent, 0 if an error is found.
 * Call mm_check() after any heap operation during debugging; remove all
 * calls before final submission to avoid throughput penalties.
 */
int mm_check(void)
{
    void *bp;
    char *heap_lo = mem_heap_lo();
    char *heap_hi = mem_heap_hi();
    int consistent = 1;

    /* Check 6: prologue block is always allocated with size = DSIZE (8 bytes) */
    void *prologue = heap_listp; /* heap_listp points to prologue footer */
    if (GET_SIZE(HDRP(prologue)) != DSIZE || !GET_ALLOC(HDRP(prologue)) ||
        GET_SIZE(FTRP(prologue)) != DSIZE || !GET_ALLOC(FTRP(prologue)))
    {
        fprintf(stderr, "mm_check: prologue block corrupted\n");
        consistent = 0;
    }

    int prev_free = 0; /* track whether the previous block was free */

    for (bp = heap_listp; GET_SIZE(HDRP(bp)) > 0; bp = NEXT_BLKP(bp))
    {
        size_t hdr_size = GET_SIZE(HDRP(bp));
        size_t ftr_size = GET_SIZE(FTRP(bp));
        int hdr_alloc = GET_ALLOC(HDRP(bp));
        int ftr_alloc = GET_ALLOC(FTRP(bp));

        /* Check 5: block pointer is within heap bounds */
        if ((char *)bp < heap_lo || (char *)bp > heap_hi)
        {
            fprintf(stderr, "mm_check: block %p is out of heap bounds [%p, %p]\n",
                    bp, heap_lo, heap_hi);
            consistent = 0;
        }

        /* Check 1: block payload is 8-byte aligned */
        if ((size_t)bp % ALIGNMENT != 0)
        {
            fprintf(stderr, "mm_check: block %p is not 8-byte aligned\n", bp);
            consistent = 0;
        }

        /* Check 2: header and footer must agree on size and alloc bit */
        if (hdr_size != ftr_size || hdr_alloc != ftr_alloc)
        {
            fprintf(stderr, "mm_check: block %p header/footer mismatch "
                            "(hdr: size=%zu alloc=%d, ftr: size=%zu alloc=%d)\n",
                    bp, hdr_size, hdr_alloc, ftr_size, ftr_alloc);
            consistent = 0;
        }

        if (!hdr_alloc) /* free block checks */
        {
            /* Check 3: no two consecutive free blocks */
            if (prev_free)
            {
                fprintf(stderr, "mm_check: consecutive free blocks at %p "
                                "(coalescing escaped)\n",
                        bp);
                consistent = 0;
            }

            /* Check 4: free block must be at least 2*DSIZE bytes */
            if (hdr_size < 2 * DSIZE)
            {
                fprintf(stderr, "mm_check: free block %p has size %zu < minimum %d\n",
                        bp, hdr_size, 2 * DSIZE);
                consistent = 0;
            }
        }

        prev_free = !hdr_alloc;
    }

    /* Check 7: epilogue must be allocated with size = 0 */
    if (GET_SIZE(HDRP(bp)) != 0 || !GET_ALLOC(HDRP(bp)))
    {
        fprintf(stderr, "mm_check: epilogue block corrupted at %p\n", bp);
        consistent = 0;
    }

    return consistent;
}
