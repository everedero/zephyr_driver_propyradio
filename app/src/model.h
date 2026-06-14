/*
 * model.h
 *
 * Simple model management for RC remote controller application.
 * A "model" is a named set of settings (opaque byte buffer).
 * Supports up to MODEL_MAX_COUNT models.
 */
#ifndef RC_REMOTE_CONTROLLER_MODEL_H
#define RC_REMOTE_CONTROLLER_MODEL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration */
#ifndef MODEL_MAX_COUNT
#define MODEL_MAX_COUNT 6
#endif

#ifndef MODEL_NAME_MAX
#define MODEL_NAME_MAX 32
#endif

/* Channel mapping function type and structure
 * These match the application's channel mapping concept and allow
 * models to store per-channel mapping settings directly.
 */
typedef uint8_t (*map_t)(
    const uint16_t min,
    const uint16_t max,
    const uint16_t center,
          uint16_t data,
    const bool is_reversed);

struct channel_map {
    uint16_t min;
    uint16_t max;
    uint16_t center;
    uint16_t input;
    map_t map;
};

/* Model stores a named `struct channel_map` as its payload. */
typedef struct {
    bool used;                          /* slot in use */
    char name[MODEL_NAME_MAX];          /* NUL-terminated name */
    struct channel_map ch[MAX_CHANNELS];              /* channel mapping data */
} model_t;

#ifndef MODEL_FLASH_SECTION
#define MODEL_FLASH_SECTION __attribute__((section(".rodata.model_data")))
#endif

/* The persistent model list is placed in a dedicated linker section.
 * Using `.rodata.model_data` keeps the array in read-only flash storage,
 * and __attribute__((used)) prevents the linker from discarding it.
 */
extern model_t model_storage[MODEL_MAX_COUNT];
extern uint8_t active_model_index;
extern uint8_t def_map(
	const uint16_t min,
	const uint16_t max,
	const uint16_t center,
	      uint16_t data,
	const bool is_reversed);

/* Create a new model with the given name .
 * Returns index (0..MODEL_MAX_COUNT-1) on success or 0xFF on error.
 */
uint8_t model_create(const char *name);

/* Overwrite an existing model's data. Returns 0 on success, -1 on error. */
int8_t model_save(int index, const void *data);

/* Remove a model at index. Returns 0 on success, -1 on error. */
int8_t model_remove(int index);

/* Get model name by index, or NULL if invalid/unused. */
const char *model_name_get(int index);

/* Load the active model into `out_model`.
 * Parameters:
 *  - `active_model_index`: pointer to store the active model index (may be NULL)
 *  - `out_model`: pointer to a model_t buffer to receive the model data (may be NULL)
 *
 * If the persisted active index is 0xFF, this will set the active index to 0
 * and initialize `out_model` with default parameters. On success returns
 * a pointer to `out_model`, or NULL on error.
 */
int8_t load_model(uint8_t *active_model_index, struct channel_map *ch_array);

#ifdef __cplusplus
}
#endif

#endif /* RC_REMOTE_CONTROLLER_MODEL_H */
