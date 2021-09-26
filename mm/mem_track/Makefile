subdir-ccflags-y += -I$(srctree)/mm
subdir-ccflags-y += -I$(srctree)/drivers/staging/android/ion

obj-$(CONFIG_MM_PAGE_TRACE) += mem_trace.o buddy_track.o vmalloc_track.o slub_track.o mm_ion_dump.o

obj-$(CONFIG_MM_PAGE_TRACE_TEST) += mem_trace_test.o

obj-$(CONFIG_MEMORY_STATIC_FOOTPRINT) += reserved_memory_debug.o
