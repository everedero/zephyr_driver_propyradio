/*
 * model.c
 *
 * Persistent model storage section definition.
 * The model array is placed in a dedicated linker section so it can be
 * stored in flash / read-only storage.
 */

#include "model.h"
#include "model_fs.h"
#include "vars.h"
#include <zephyr/logging/log.h>

#include <errno.h>
#include <string.h>

LOG_MODULE_REGISTER(model, CONFIG_LOG_DEFAULT_LEVEL);

/* This variable holds the currently active model data used by the UI.
 * It also holds rf data from the remote controller.
 */

model_t active_model;

/* this is a temporary model used for creating or adding new models
 * into the persistent storage */

static model_t new_model;

#define MODEL_MIN_SPAN (127)

/* Adjust the minimum value of target RF based on a percentage
   For the minimum value, we want
        1. 100% to 0 (lowest value)
        2. 0% to MODEL_MIN_SPAN (Middle range value)
*/

static uint8_t model_percentage_to_min(int32_t percentage)
{
    if (percentage < 0) {
        percentage = 0;
    } else if (percentage > 100) {
        percentage = 100;
    }

    return ((100 - percentage) * MODEL_MIN_SPAN) / 100;
}

static uint8_t model_min_to_percentage(uint8_t min)
{
    if (min < 0) {
        min = 0;
    } else if (min > MODEL_MIN_SPAN) {
        min = MODEL_MIN_SPAN;
    }

    return 100 - ((min * 100) / MODEL_MIN_SPAN);
}

static uint8_t model_percentage_to_max(uint8_t percentage)
{
    if (percentage < 0) {
        percentage = 0;
    } else if (percentage > 100) {
        percentage = 100;
    }

    return 255 - ((100 - percentage) * MODEL_MIN_SPAN) / 100;
}

static uint8_t model_max_to_percentage(uint8_t max)
{
    if (max < MODEL_MIN_SPAN) {
        max = MODEL_MIN_SPAN;
    } else if (max > 255) {
        max = 255;
    }

    return 100 - ((255 - max) * 100) / MODEL_MIN_SPAN;
}

/**
 * @brief Create a new model with default settings.
 *
 * use local static temporary model.
 *
 * @param name Null-terminated model name.
 * @return 0 or error code.
 */
static int model_create(const char *name)
{
    memset(new_model.name, '\0', MODEL_NAME_MAX);

    for (uint8_t j = 0; j < MAX_CHANNELS; j++) {
        new_model.ch[j].min = 0+CALIBRATION_OFFSET;
        new_model.ch[j].max = ADC_MAX_VALUE-CALIBRATION_OFFSET;
        new_model.ch[j].center = ADC_MAX_VALUE / 2;
        new_model.ch[j].input = ADC_MAX_VALUE / 2;
        new_model.ch[j].min_end_point = 0;
        new_model.ch[j].max_end_point = 255;
        new_model.ch[j].is_reversed = false;
        new_model.ch[j].map = def_map;
    }
            
    new_model.used = true;
    strncpy(new_model.name, name, MODEL_NAME_MAX - 1);
    new_model.name[MODEL_NAME_MAX - 1] = '\0';

    return 0;
}
/**
 * @brief Initialize the model system and load the active model.
 * if no model is found, create a default model.
 * 
 * @param none
 * 
 * @return 0 on success, error code on failure.
 */ 
int model_init(void)
{
    char buf[MODEL_MAX_COUNT][MODEL_NAME_MAX] = {0};
	size_t models_num = 0;
    int rc = model_fs_init();
    
	if (rc != 0) {
		LOG_ERR("model_fs_init failed (%d)", rc);
        return rc;
    }

	rc = model_fs_list_models(buf, &models_num);
	if (rc < 0) {
		LOG_ERR("model_fs_list_models failed (%d)", rc);
		return rc;
	}

	if (models_num == 0) {
		LOG_INF("No models found, creating default model");
		rc = model_add("Default Model\0");
		if (rc != 0) {
			LOG_ERR("Failed to create default model");
			return rc;
		}
		LOG_INF("Default model created successfully");

		rc = model_fs_list_models(buf, &models_num);
		if (rc < 0 || models_num == 0U) {
			LOG_ERR("Failed to list newly created default model: %d", rc);
			return rc < 0 ? rc : -EIO;
		}
	} else {
		LOG_INF("%zu models found", models_num);
	}
	/* Set the first model as the active device */
	rc = model_load(buf[0]);
	if (rc != 0) {
		LOG_ERR("Failed to load initial model");
		return rc;
	}

	/* Populate device list from file system */
	for (size_t i = 0; i < models_num; i++) {
    	set_var_device_list(buf[i]);
	}
    return 0;
}
/**
 * @brief Add a new model with the given name.
 * 
 * @param name Null-terminated model name.
 * @return 0 on success, error code on failure.
 */
int model_add(const char *name)
{
    int rc = model_create(name);
    if (rc != 0) {
        LOG_ERR("Failed to create model '%s': %d", name, rc);
        return rc;
    }
    rc = model_fs_save_model(name, &new_model);
    if (rc != 0) {
        LOG_ERR("Failed to save model '%s' to storage: %d", name, rc);
        return rc;
    }
    /* Set the newly created model as the active device */
    model_load(name);
    /* Update device list in UI */
    set_var_device_list(name);

    return 0;
}

/* Copy a model from one name to another.
 * Returns 0 or error code.
 */
int model_copy(const char *src_name, const char *dst_name){

    int rc = model_fs_load_model(src_name, &new_model);
    if (rc != 0) {
        LOG_ERR("Failed to load source model '%s': %d", src_name, rc);
        return rc;
    }
    strncpy(new_model.name, dst_name, MODEL_NAME_MAX - 1);
    new_model.name[MODEL_NAME_MAX - 1] = '\0';

    rc = model_fs_save_model(dst_name, &new_model);
    if (rc != 0) {
        LOG_ERR("Failed to save copied model '%s': %d", dst_name, rc);
        return rc;
    }
    /* Set the newly copied model as the active device */
    model_load(dst_name);
    /* Update device list in UI */
    set_var_device_list(dst_name);

    return 0;
}

/* Reset a model's settings to their factory defaults.
 * Returns 0 or error code.
 */
int model_reset(const char *model_name) {
    int rc = model_create(model_name);
    if (rc != 0) {
        LOG_ERR("Failed to create default model '%s': %d", model_name, rc);
        return rc;
    }
    /* First delete the existing model and save the reset one */
    rc = model_fs_delete_model(model_name);
    if (rc != 0) {
        LOG_ERR("Failed to delete existing model '%s': %d", model_name, rc);
        return rc;
    }
    rc = model_fs_save_model(model_name, &new_model);
    if (rc != 0) {
        LOG_ERR("Failed to save reset model '%s' to storage: %d", model_name, rc);
        return rc;
    }
    /* Set the reset model as the active device */
    model_load(model_name);
    /* Update device list in UI */
    set_var_device_list(model_name);

    return 0;
}


/**
 * @brief Load the active model channel map.
 *
 * Loads the model identified by the given name into the active model
 * Set UI parameters from active_model.
 *
 * @param name Pointer to model name.
 * @return 0 or error code.
 */
int model_load(const char *model_name)
{
    int rc = model_fs_load_model(model_name, &active_model);

    if (rc != 0) {
        LOG_ERR("Failed to load model '%s' from storage: %d", model_name, rc);
        return rc;
    }
    memcpy(active_model.name, model_name, MODEL_NAME_MAX);

    set_var_reverse1(active_model.ch[ROULIS].is_reversed);
	set_var_reverse2(active_model.ch[TANGAGE].is_reversed);
	set_var_reverse3(active_model.ch[GAZ].is_reversed);
	set_var_reverse4(active_model.ch[LACET].is_reversed);
    active_model.used = true; // Ensure the slot is marked as used
    set_var_device_name(active_model.name);
    set_var_ch1_min(model_min_to_percentage(active_model.ch[ROULIS].min_end_point));
    set_var_ch2_min(model_min_to_percentage(active_model.ch[TANGAGE].min_end_point));
    set_var_ch3_min(model_min_to_percentage(active_model.ch[GAZ].min_end_point));
    set_var_ch4_min(model_min_to_percentage(active_model.ch[LACET].min_end_point));
    set_var_ch1_max(model_max_to_percentage(active_model.ch[ROULIS].max_end_point));
    set_var_ch2_max(model_max_to_percentage(active_model.ch[TANGAGE].max_end_point));
    set_var_ch3_max(model_max_to_percentage(active_model.ch[GAZ].max_end_point));
    set_var_ch4_max(model_max_to_percentage(active_model.ch[LACET].max_end_point));

	return rc;
}

/**
 * @brief Save channel map data into an existing model slot.
 *
 * @param map Pointer to channel map data.
 * @return 0 on success, error code on failure.
 */
int model_store(const struct channel_map *map)
{
    active_model.used = true; // Ensure the slot is marked as used
    memcpy(active_model.name, get_var_device_name(), MODEL_NAME_MAX);
    for (int i = 0; i < MAX_CHANNELS; i++) {
        memcpy(&active_model.ch[i], &map[i], sizeof(struct channel_map));
    }

    active_model.ch[ROULIS].is_reversed = get_var_reverse1();
	active_model.ch[TANGAGE].is_reversed = get_var_reverse2();
	active_model.ch[GAZ].is_reversed = get_var_reverse3();
	active_model.ch[LACET].is_reversed = get_var_reverse4();

    active_model.ch[ROULIS].min_end_point = model_percentage_to_min(get_var_ch1_min());
    active_model.ch[TANGAGE].min_end_point = model_percentage_to_min(get_var_ch2_min());
    active_model.ch[GAZ].min_end_point = model_percentage_to_min(get_var_ch3_min());
    active_model.ch[LACET].min_end_point = model_percentage_to_min(get_var_ch4_min());
    active_model.ch[ROULIS].max_end_point = model_percentage_to_max(get_var_ch1_max());
    active_model.ch[TANGAGE].max_end_point = model_percentage_to_max(get_var_ch2_max());
    active_model.ch[GAZ].max_end_point = model_percentage_to_max(get_var_ch3_max());
    active_model.ch[LACET].max_end_point = model_percentage_to_max(get_var_ch4_max());

    /* Save the model to persistent storage */
    return model_fs_save_model(active_model.name, &active_model);
}

int model_apply_calibration(const uint16_t min[CALIBRATION_AXIS_COUNT],
			    const uint16_t max[CALIBRATION_AXIS_COUNT])
{
	uint16_t previous_min[CALIBRATION_AXIS_COUNT];
	uint16_t previous_max[CALIBRATION_AXIS_COUNT];
	int rc;

	if (min == NULL || max == NULL) {
		return -EINVAL;
	}
	if (!active_model.used) {
		return -ENODEV;
	}

	for (size_t i = 0U; i < CALIBRATION_AXIS_COUNT; i++) {
		if (min[i] >= max[i] || max[i] > ADC_MAX_VALUE) {
			return -EINVAL;
		}
		previous_min[i] = active_model.ch[i].min;
		previous_max[i] = active_model.ch[i].max;
	}

	for (size_t i = 0U; i < CALIBRATION_AXIS_COUNT; i++) {
		active_model.ch[i].min = min[i];
		active_model.ch[i].max = max[i];
	}

	rc = model_fs_save_model(active_model.name, &active_model);
	if (rc != 0) {
		for (size_t i = 0U; i < CALIBRATION_AXIS_COUNT; i++) {
			active_model.ch[i].min = previous_min[i];
			active_model.ch[i].max = previous_max[i];
		}
		LOG_ERR("Failed to save calibration for model '%s': %d",
			active_model.name, rc);
	}

	return rc;
}

int model_reset_calibration(void)
{
	uint16_t default_min[CALIBRATION_AXIS_COUNT];
	uint16_t default_max[CALIBRATION_AXIS_COUNT];

	if (!active_model.used) {
		return -ENODEV;
	}

	for (size_t i = 0U; i < CALIBRATION_AXIS_COUNT; i++) {
		default_min[i] = CALIBRATION_OFFSET;
		default_max[i] = ADC_MAX_VALUE - CALIBRATION_OFFSET;
	}

	return model_apply_calibration(default_min, default_max);
}

/**
 * @brief Remove a model from persistent storage by name.
 * Warning: if model to remove is the active model
 * active_model will hold another model from persistent storage
 * or will hold a default model with default values if no other model
 * is found
 * 
 * @param model_name Pointer to the name of the model to remove.
 * @return 0 on success, error code on failure.
 */
int model_remove(char *model_name)
{
    int rc = 0;

    /* check if model name corresponds to active_model.name */
    if (strcmp(model_name, active_model.name) == 0) {
        /* check if another model exists by listing models */
        char model_list[MODEL_MAX_COUNT][MODEL_NAME_MAX] = {0};
        size_t model_count = 0;

        /* First of all, remove the model from persistent storage */
        rc = model_fs_delete_model(model_name);
        /* Refresh the device list */
        set_var_device_list("");

        if (rc != 0) {
            LOG_ERR("Failed to remove model '%s' from storage: %d", model_name, rc);
            return rc;
        }
        rc = model_fs_list_models(model_list, &model_count);
        if (rc != 0) {
            LOG_ERR("Failed to list models: %d", rc);
            return rc;
        }
        if (model_count > 0) {
            /* load the first model that is not the one being removed */
            for (size_t i = 0; i < model_count; i++) {
                if (strcmp(model_list[i], model_name) != 0) {
                    rc = model_load(model_list[i]);
                    if (rc != 0) {
                        LOG_ERR("Failed to load model '%s': %d", model_list[i], rc);
                        return rc;
                    }
                    break;
                }
            }
        } else {
            /* No other models exist, create a default model */
            LOG_INF("No other models found, creating default model");
            model_create("Default Model");
            set_var_ch1_min(model_min_to_percentage(new_model.ch[ROULIS].min_end_point));
            set_var_ch2_min(model_min_to_percentage(new_model.ch[TANGAGE].min_end_point));
            set_var_ch3_min(model_min_to_percentage(new_model.ch[GAZ].min_end_point));
            set_var_ch4_min(model_min_to_percentage(new_model.ch[LACET].min_end_point));
            set_var_ch1_max(model_max_to_percentage(new_model.ch[ROULIS].max_end_point));
            set_var_ch2_max(model_max_to_percentage(new_model.ch[TANGAGE].max_end_point));
            set_var_ch3_max(model_max_to_percentage(new_model.ch[GAZ].max_end_point));
            set_var_ch4_max(model_max_to_percentage(new_model.ch[LACET].max_end_point));
            rc = model_store(new_model.ch);
            set_var_device_list("");
            if (rc != 0) {
                LOG_ERR("Failed to save default model: %d", rc);
                return rc;
            }
        }
    } else {
        /* If the model being removed is not the active model, do nothing */
        LOG_INF("Removing model '%s' which is not the active model", model_name);
        rc = model_fs_delete_model(model_name);
        set_var_device_list("");
        if (rc != 0) {
            LOG_ERR("Failed to remove model '%s' from storage: %d", model_name, rc);
            return rc;
        }
    }
    return rc; // Success
}

int model_recover_during_init(void) {

    int rc = 0;

    LOG_INF("Entering recovery process: Erasing all models and reinitializing model storage");
    rc = model_fs_erase_all();

    if (rc != 0) {
        LOG_ERR("model_fs_erase_all failed (%d)", rc);
        return rc;
    }
    LOG_INF("Reinitializing model storage after erase");
    return rc;
}
