/*
 * mm-naive.c - The fastest, least memory-efficient malloc package.
 *
 * In this naive approach, a block is allocated by simply incrementing
 * the brk pointer.  A block is pure payload. There are no headers or
 * footers.  Blocks are never coalesced or reused. Realloc is
 * implemented directly using mm_malloc and mm_free.
 *
 * NOTE TO STUDENTS: Replace this header comment with your own header
 * comment that gives a high level description of your solution.
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

static void *coalesce(void *bp)
{
    // 1=allocated 0=free
    size_t current_size = GET_SIZE(HDRP(bp));
    int prev_allocated = GET_ALLOC(FTRP(PREV_BLKP(bp)));
    int next_allocated = GET_ALLOC(HDRP(NEXT_BLKP(bp)));
    if (prev_allocated && next_allocated) // neither are free
    {
        return bp;
    }
    else if (!prev_allocated && next_allocated) // prev is free
    {
        size_t new_size = current_size + GET_SIZE(HDRP(PREV_BLKP(bp)));
        PUT(HDRP(PREV_BLKP(bp)), PACK(new_size, 0));
        PUT(FTRP(bp), PACK(new_size, 0));
        return PREV_BLKP(bp);
    }
    else if (prev_allocated && !next_allocated) // next is free
    {
        size_t new_size = current_size + GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(new_size, 0));
        PUT(HDRP(bp), PACK(new_size, 0));
        return bp;
    }
    else // both are free
    {
        size_t new_size = current_size + GET_SIZE(HDRP(PREV_BLKP(bp))) + GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(PREV_BLKP(bp)), PACK(new_size, 0));
        PUT(FTRP(NEXT_BLKP(bp)), PACK(new_size, 0));
        return PREV_BLKP(bp);
    }
};

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
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size;

    /* Step 1: Round words up to nearest even number to maintain alignment */
    size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;

    /* Step 2: Request more memory from the OS */
    // bp= block pointer, pointes to the start of the payload, not the entire block
    if ((bp = mem_sbrk(size)) == (void *)-1)
        return NULL;

    // write new header and footer
    PUT(HDRP(bp), PACK(size, 0));
    PUT(FTRP(bp), PACK(size, 0));

    // write new epilogue
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 1));
    return coalesce(bp);
};
/*
 * mm_init - initialize the malloc package.
 padding, prol header, prol footer, epilogue header
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
 * mm_malloc - Allocate a block by incrementing the brk pointer.
 *     Always allocate a block whose size is a multiple of the alignment.
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
 * mm_free - Freeing a block does nothing.
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
