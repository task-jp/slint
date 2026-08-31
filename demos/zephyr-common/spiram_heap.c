/* Copyright © SixtyFPS GmbH <info@slint.dev>
 * SPDX-License-Identifier: MIT
 *
 * Route the C allocator family to the external PSRAM.
 *
 * Slint's freestanding Rust allocator and C++'s operator new both end up
 * in malloc, whose heap lives in the little internal DRAM that is left
 * over (~160 KB on the ESP32). Zephyr's CONFIG_ESP_SPIRAM only feeds the
 * shared multi heap, which nothing reaches by default, so a UI of any
 * size starves while megabytes of PSRAM sit idle.
 *
 * This file replaces the whole allocator family on top of the shared
 * multi heap. CONFIG_COMMON_LIBC_MALLOC must be disabled so Zephyr's own
 * malloc drops out; defining every entry point here keeps picolibc's
 * nano allocator from being dragged in for the aligned variants.
 * Each block carries a 16-byte header with its size and the offset back
 * to the real allocation, which serves realloc, usable_size and the
 * aligned frontends alike.
 */

#ifdef CONFIG_ESP_SPIRAM

#include <zephyr/kernel.h>
#include <zephyr/multi_heap/shared_multi_heap.h>
#include <string.h>
#include <errno.h>

#define HDR_SIZE 16

struct hdr {
	size_t size;
	size_t offset; /* returned pointer minus allocation base */
};

static void *alloc_common(size_t align, size_t size)
{
	if (size == 0) {
		size = 1;
	}
	if (align < HDR_SIZE) {
		align = HDR_SIZE;
	}
	uint8_t *base = shared_multi_heap_aligned_alloc(SMH_REG_ATTR_EXTERNAL, align,
							size + align);
	if (base == NULL) {
		return NULL;
	}
	uint8_t *ptr = base + align;
	struct hdr *h = (struct hdr *)(ptr - HDR_SIZE);

	h->size = size;
	h->offset = align;
	return ptr;
}

void *malloc(size_t size)
{
	return alloc_common(HDR_SIZE, size);
}

void free(void *ptr)
{
	if (ptr != NULL) {
		struct hdr *h = (struct hdr *)((uint8_t *)ptr - HDR_SIZE);

		shared_multi_heap_free((uint8_t *)ptr - h->offset);
	}
}

void *calloc(size_t nmemb, size_t size)
{
	if (size != 0 && nmemb > SIZE_MAX / size) {
		return NULL;
	}
	size_t total = nmemb * size;
	void *p = malloc(total);

	if (p != NULL) {
		memset(p, 0, total);
	}
	return p;
}

void *realloc(void *ptr, size_t size)
{
	if (ptr == NULL) {
		return malloc(size);
	}
	if (size == 0) {
		free(ptr);
		return NULL;
	}
	struct hdr *h = (struct hdr *)((uint8_t *)ptr - HDR_SIZE);
	void *p = malloc(size);

	if (p != NULL) {
		memcpy(p, ptr, h->size < size ? h->size : size);
		free(ptr);
	}
	return p;
}

void *aligned_alloc(size_t align, size_t size)
{
	return alloc_common(align, size);
}

void *memalign(size_t align, size_t size)
{
	return alloc_common(align, size);
}

int posix_memalign(void **memptr, size_t align, size_t size)
{
	if (align < sizeof(void *) || (align & (align - 1)) != 0) {
		return EINVAL;
	}
	void *p = alloc_common(align, size);

	if (p == NULL) {
		return ENOMEM;
	}
	*memptr = p;
	return 0;
}

size_t malloc_usable_size(void *ptr)
{
	if (ptr == NULL) {
		return 0;
	}
	return ((struct hdr *)((uint8_t *)ptr - HDR_SIZE))->size;
}

#endif /* CONFIG_ESP_SPIRAM */
