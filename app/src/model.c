/*
 * model.c
 *
 * Persistent model storage section definition.
 * The model array is placed in a dedicated linker section so it can be
 * stored in flash / read-only storage.
 */

#include "model.h"
#include "vars.h"

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
                model_storage[i].ch[j].is_reversed = false;
                model_storage[i].ch[j].map = def_map;
            }
            /* Default CH1..CH4 selections: ROULIS, TANGAGE, GAZ, LACET. */
            model_storage[i].channel_selection[0] = ROULIS;
            model_storage[i].channel_selection[1] = TANGAGE;
            model_storage[i].channel_selection[2] = GAZ;
            model_storage[i].channel_selection[3] = LACET;
            model_storage[i].used = true;
            strncpy(model_storage[i].name, name, MODEL_NAME_MAX - 1);
            model_storage[i].name[MODEL_NAME_MAX - 1] = '\0';
            set_var_device_name(model_storage[i].name);
            set_var_device_list(model_storage[i].name);
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
    if (index < 0 || index >= MODEL_MAX_COUNT){
        return NULL;
    }
    return model_storage[index].name;
}

/**
 * @brief Load the active model channel map.
 *
 * Loads the model identified by the given name into the provided channel map array.
 *
 * @param name Pointer to model name.
 * @param ch_array Destination channel map array.
 * @return active model index or 0xFF in case of error.
 */
uint8_t load_model(const char *model_name, struct channel_map *ch_array)
{
    uint8_t channel_selection[MODEL_SELECTION_COUNT] = {ROULIS, TANGAGE, GAZ, LACET};

	/* Find model by name */
	uint8_t idx = 0xFF;
	for (uint8_t i = 0; i < MODEL_MAX_COUNT; i++) {
		if (strcmp(model_storage[i].name, model_name) == 0) {
			idx = i;
			break;
		}
	}
	if (idx == 0xFF) {
		return 0xFF; // Model not found
	}

	if (idx >= MODEL_MAX_COUNT) {
		return 0xFF;
	}

	memcpy(ch_array, model_storage[idx].ch, sizeof(model_storage[idx].ch));

    memcpy(channel_selection,
               model_storage[idx].channel_selection,
               MODEL_SELECTION_COUNT * sizeof(model_storage[idx].channel_selection[0]));
    set_var_selection1(channel_selection[0]);
	set_var_selection2(channel_selection[1]);
	set_var_selection3(channel_selection[2]);
	set_var_selection4(channel_selection[3]);
    model_storage[idx].used = true; // Ensure the slot is marked as used
    set_var_device_name(model_storage[idx].name);
	
	return 0;
}

/**
 * @brief Save channel map data into an existing model slot.
 *
 * @param index Model index.
 * @param data Pointer to channel map data.
 * @return 0 on success, 1 on invalid index or null data.
 */
uint8_t model_save(int index, const struct channel_map *map, const uint8_t *sel)
{
    if (index >= MODEL_MAX_COUNT || map == NULL || !model_storage[index].used) {
        return 1; // Invalid index, null map, or unused slot
    }
    model_storage[index].used = true; // Ensure the slot is marked as used
    memcpy(model_storage[index].name, get_var_device_name(), MODEL_NAME_MAX);
    memcpy(model_storage[index].ch, map, sizeof(model_storage[index].ch));
    if (sel != NULL) {
        memcpy(model_storage[index].channel_selection,
               sel,
               MODEL_SELECTION_COUNT * sizeof(model_storage[index].channel_selection[0]));
    }
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
 * @return 0 on success, 1 on invalid index.
 */
uint8_t model_remove(int index)
{
    if (index >= MODEL_MAX_COUNT) {
        return 1; // Invalid index
    }

    model_storage[index].used = false;
    memset(model_storage[index].name, 0, MODEL_NAME_MAX);
    memset(model_storage[index].ch, 0, sizeof(model_storage[index].ch));

    return 0; // Success
}

