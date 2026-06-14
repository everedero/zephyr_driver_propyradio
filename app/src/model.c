/*
 * model.c
 *
 * Persistent model storage section definition.
 * The model array is placed in a dedicated linker section so it can be
 * stored in flash / read-only storage.
 */

#include "model.h"

#ifndef MODEL_FLASH_SECTION
#define MODEL_FLASH_SECTION __attribute__((section(".rodata.model_data")))
#endif

#include <string.h>

/* Persistent storage for the active model index. A value of 0xFF means
 * "not set" and should cause initialization to the first model (index 0).
 * Placed in the same read-only model data section so tools/linker can
 * locate it in persistent storage imagery.
 */
uint8_t active_model_index = 0xFF;
model_t model_storage[MODEL_MAX_COUNT] = {0};

/**
 * @brief Create a new model slot.
 *
 * Allocates the first available model slot, initializes the channel map
 * data to safe defaults, and stores the model name.
 *
 * @param name Null-terminated model name.
 * @return Model index (0..MODEL_MAX_COUNT-1) on success, 0xFF on error.
 */
uint8_t model_create(const char *name)
{
    for (uint8_t i = 0; i < MODEL_MAX_COUNT; i++) {
        if (model_storage[i].used == false) {
            memset(model_storage[i].name, 0, MODEL_NAME_MAX);
            for (uint8_t j = 0; j < MAX_CHANNELS; j++) {
                model_storage[i].ch[j].min = 0+CALIBRATION_OFFSET;
                model_storage[i].ch[j].max = ADC_MAX_VALUE-CALIBRATION_OFFSET;
                model_storage[i].ch[j].center = ADC_MAX_VALUE / 2;
                model_storage[i].ch[j].input = ADC_MAX_VALUE / 2;
                model_storage[i].ch[j].map = def_map;
            }
            model_storage[i].used = true;
            strncpy(model_storage[i].name, name, MODEL_NAME_MAX - 1);
            model_storage[i].name[MODEL_NAME_MAX - 1] = '\0';
            return i;
        }
    }
    return 0xFF; // No available slot
}

/**
 * @brief Get a model name by index.
 *
 * @param index Model index.
 * @return Pointer to the model name string, or NULL if invalid or unused.
 */
const char *model_name_get(int index)
{
    if (index < 0 || index >= MODEL_MAX_COUNT || !model_storage[index].used) {
        return NULL;
    }
    return model_storage[index].name;
}

/**
 * @brief Load the active model channel map.
 *
 * If the passed-in active index is 0xFF, the function creates a default
 * model and uses that slot. The active index is updated to the selected
 * model index.
 *
 * @param active_model_index Pointer to the active model index.
 * @param ch_array Destination channel map array.
 * @return 0 on success, -1 on error.
 */
int8_t load_model(uint8_t *active_model_index, struct channel_map *ch_array)
{
	uint8_t idx = *active_model_index;

	if (idx == 0xFF) {
		/* First-time: select first entry and populate defaults into out_model */
		idx = model_create("Default Model");
	}

	if (idx >= MODEL_MAX_COUNT) {
		return -1;
	}

	memcpy(ch_array, model_storage[idx].ch, sizeof(model_storage[idx].ch));
	*active_model_index = idx;
	
	return 0;
}

/**
 * @brief Save channel map data into an existing model slot.
 *
 * @param index Model index.
 * @param data Pointer to channel map data.
 * @return 0 on success, -1 on invalid index or null data.
 */
int8_t model_save(int index, const void *data)
{
    if (index >= MODEL_MAX_COUNT || data == NULL) {
        return -1; // Invalid index or null data
    }

    memcpy(model_storage[index].ch, data, sizeof(model_storage[index].ch));
    return 0; // Success
}

/**
 * @brief Remove a model slot 
 * Warning this function does not manage the active_model_index,
 * so if you remove the active model, you should set the
 * active_model_index to 0xFF or another valid index.
 *
 * Marks the model slot as unused and clears its data.
 * @param index Model index to remove.
 * @return 0 on success, -1 on invalid index.
 */
int8_t model_remove(int index)
{
    if (index >= MODEL_MAX_COUNT) {
        return -1; // Invalid index
    }

    model_storage[index].used = false;
    memset(model_storage[index].name, 0, MODEL_NAME_MAX);
    memset(model_storage[index].ch, 0, sizeof(model_storage[index].ch));

    return 0; // Success
}

