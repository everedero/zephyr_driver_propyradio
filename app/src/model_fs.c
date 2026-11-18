/*
 * model_fs.c
 *
 * LittleFS-backed persistence helpers for storing RC model settings
 * in a dedicated flash partition on the STM32F469I-DISCO board.
 */

#include "model_fs.h"

#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(model_fs, CONFIG_LOG_DEFAULT_LEVEL);

#define MODEL_FS_MAGIC 0x52434D44u /* "RCMD" */
#define MODEL_FS_VERSION 3u
#define MODEL_FS_LEGACY_VERSION 1u

#define MODEL_FS_MOUNT_POINT "/lfs"
#define MODEL_FS_MODELS_DIR MODEL_FS_MOUNT_POINT "/models"

#define MODEL_FS_MODEL_NAME_LEN (MODEL_NAME_MAX)

struct model_channel_blob {
	uint16_t min;
	uint16_t max;
	uint16_t center;
	uint16_t input;
	uint8_t min_end_point;
	uint8_t max_end_point;
	uint8_t is_reversed;
};

struct model_blob {
	uint32_t magic;
	uint32_t version;
	char name[MODEL_FS_MODEL_NAME_LEN];
	struct model_channel_blob ch[MAX_CHANNELS];
};

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage_lfs);

// FS_LITTLEFS_DECLARE_CUSTOM_CONFIG(storage_lfs, 4, 1, 16, 256, 32);

static struct fs_mount_t model_fs_mount = {
	.type = FS_LITTLEFS,
	.fs_data = &storage_lfs,
	.storage_dev = (void *)PARTITION_ID(storage_partition),
	.mnt_point = MODEL_FS_MOUNT_POINT,
};

static void model_to_blob(const model_t *model, struct model_blob *blob)
{
	memset(blob, 0, sizeof(*blob));
	blob->magic = MODEL_FS_MAGIC;
	blob->version = MODEL_FS_VERSION;

	strncpy(blob->name, model->name, sizeof(blob->name) - 1);
	blob->name[sizeof(blob->name) - 1] = '\0';

	for (int i = 0; i < MAX_CHANNELS; i++) {
		blob->ch[i].min = model->ch[i].min;
		blob->ch[i].max = model->ch[i].max;
		blob->ch[i].center = model->ch[i].center;
		blob->ch[i].input = model->ch[i].input;
		blob->ch[i].min_end_point = model->ch[i].min_end_point;
		blob->ch[i].max_end_point = model->ch[i].max_end_point;
		blob->ch[i].is_reversed = model->ch[i].is_reversed ? 1u : 0u;
	}
}

static void blob_to_model(const struct model_blob *blob, model_t *model)
{
	memset(model, 0, sizeof(*model));
	model->used = true;

	strncpy(model->name, blob->name, sizeof(model->name) - 1);
	model->name[sizeof(model->name) - 1] = '\0';

	for (int i = 0; i < MAX_CHANNELS; i++) {
		model->ch[i].min = blob->ch[i].min;
		model->ch[i].max = blob->ch[i].max;
		model->ch[i].center = blob->ch[i].center;
		model->ch[i].input = blob->ch[i].input;
		model->ch[i].min_end_point = blob->ch[i].min_end_point;
		model->ch[i].max_end_point = blob->ch[i].max_end_point;
		model->ch[i].is_reversed = blob->ch[i].is_reversed ? true : false;
		model->ch[i].map = def_map;
	}
}

static int validate_model_name(const char *name)
{
	size_t i;

	if (name == NULL) {
		return -EINVAL;
	}

	for (i = 0; i < MODEL_NAME_MAX; i++) {
		unsigned char ch = (unsigned char)name[i];

		if (ch == '\0') {
			return (i > 0) ? 0 : -EINVAL;
		}
		if (i == MODEL_NAME_MAX - 1U) {
			return -ENAMETOOLONG;
		}
		if (ch == '/' || ch == '\\') {
			return -EINVAL;
		}
	}

	return -ENAMETOOLONG;
}

static int build_model_path(const char *name, char *out, size_t out_len)
{
	int rc = validate_model_name(name);
	int written;

	if (rc != 0) {
		return rc;
	}

	written = snprintf(out, out_len, "%s/%s.bin", MODEL_FS_MODELS_DIR, name);
	if (written < 0 || (size_t)written >= out_len) {
		return -ENAMETOOLONG;
	}

	return 0;
}

static int ensure_models_dir(void)
{
	struct fs_dirent entry;
	int rc = fs_stat(MODEL_FS_MODELS_DIR, &entry);

	if (rc == 0) {
		return entry.type == FS_DIR_ENTRY_DIR ? 0 : -ENOTDIR;
	}
	if (rc != -ENOENT) {
		return rc;
	}

	rc = fs_mkdir(MODEL_FS_MODELS_DIR);
	if (rc == -EEXIST) {
		rc = fs_stat(MODEL_FS_MODELS_DIR, &entry);
		if (rc == 0 && entry.type == FS_DIR_ENTRY_DIR) {
			return 0;
		}
		return rc != 0 ? rc : -ENOTDIR;
	}

	return rc;
}

static int modelfs_mount(void)
{
	int rc = fs_mount(&model_fs_mount);

	if (rc != 0 ) {
		LOG_ERR("Failed to mount LittleFS: %d", rc);
		return rc;
	}

	LOG_INF("LittleFS mounted at %s", MODEL_FS_MOUNT_POINT);

	/* Ensure the models directory exists without erasing existing storage. */
	rc = ensure_models_dir();

	if (rc != 0) {
		LOG_ERR("Failed to prepare %s: %d", MODEL_FS_MODELS_DIR, rc);
		return rc;
	}

	LOG_INF("LittleFS %s ready for use", MODEL_FS_MOUNT_POINT);
	return rc;
}

static int modelfs_write_blob(const char *path, const struct model_blob *blob)
{
	struct fs_file_t file;
	int rc;

	fs_file_t_init(&file);

	rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (rc != 0) {
		LOG_ERR("fs_open(%s) failed: %d", path, rc);
		return rc;
	}

	rc = fs_write(&file, blob, sizeof(*blob));
	if (rc < 0) {
		LOG_ERR("fs_write(%s) failed: %d", path, rc);
		fs_close(&file);
		return rc;
	}

	if ((size_t)rc != sizeof(*blob)) {
		LOG_ERR("Short write for %s: got %d bytes", path, rc);
		fs_close(&file);
		return -EIO;
	}

	rc = fs_close(&file);
	if (rc != 0) {
		LOG_ERR("fs_close(%s) failed: %d", path, rc);
		return rc;
	}

	return 0;
}

static int modelfs_read_blob(const char *path, struct model_blob *blob)
{
	struct fs_file_t file;
	int rc;

	fs_file_t_init(&file);

	rc = fs_open(&file, path, FS_O_READ);
	if (rc != 0) {
		LOG_ERR("fs_open(%s) failed: %d", path, rc);
		return rc;
	}

	rc = fs_read(&file, blob, sizeof(*blob));
	fs_close(&file);
	if (rc < 0) {
		LOG_ERR("fs_read(%s) failed: %d", path, rc);
		return rc;
	}

	if ((size_t)rc != sizeof(*blob)) {
		LOG_ERR("Short read for %s: got %d bytes", path, rc);
		return -EIO;
	}

	return 0;
}

int model_fs_init(void)
{
	return modelfs_mount();
}

int model_fs_save_model(const char *name, const model_t *model)
{
	char path[128];
	struct model_blob blob;
	int rc;

	if (name == NULL || model == NULL) {
		return -EINVAL;
	}
	if (validate_model_name(model->name) != 0 || strcmp(name, model->name) != 0) {
		return -EINVAL;
	}

	rc = build_model_path(name, path, sizeof(path));
	if (rc != 0) {
		return rc;
	}

	model_to_blob(model, &blob);
	return modelfs_write_blob(path, &blob);
}

int model_fs_load_model(const char *name, model_t *model)
{
	char path[128];
	struct model_blob blob;
	int rc;

	if (name == NULL || model == NULL) {
		return -EINVAL;
	}

	rc = build_model_path(name, path, sizeof(path));
	if (rc != 0) {
		return rc;
	}

	rc = modelfs_read_blob(path, &blob);
	if (rc != 0) {
		return rc;
	}

	if (blob.magic != MODEL_FS_MAGIC ||
	    (blob.version != MODEL_FS_VERSION &&
	     blob.version != MODEL_FS_LEGACY_VERSION) ||
	    validate_model_name(blob.name) != 0 || strcmp(blob.name, name) != 0) {
		LOG_ERR("Invalid model blob in %s", path);
		return -EIO;
	}

	blob_to_model(&blob, model);
	return 0;
}

int model_fs_delete_model(const char *name)
{
	char path[128];
	int rc;

	if (name == NULL) {
		return -EINVAL;
	}

	rc = build_model_path(name, path, sizeof(path));
	if (rc != 0) {
		return rc;
	}

	rc = fs_unlink(path);
	if (rc == -ENOENT) {
		return 0;
	}

	return rc;
}

int model_fs_list_models(char models[][MODEL_NAME_MAX], size_t *model_count)
{
	struct fs_dir_t dir;
	struct fs_dirent entry;
	static const char suffix[] = ".bin";
	int rc;

	if (models == NULL || model_count == NULL) {
		return -EINVAL;
	}

	*model_count = 0U;
	fs_dir_t_init(&dir);

	rc = fs_opendir(&dir, MODEL_FS_MODELS_DIR);
	if (rc == -ENOENT) {
		return 0;
	}
	if (rc != 0) {
		LOG_ERR("fs_opendir(%s) failed: %d", MODEL_FS_MODELS_DIR, rc);
		return rc;
	}

	while (1) {
		rc = fs_readdir(&dir, &entry);
		if (rc < 0) {
			break;
		}

		if (entry.name[0] == '\0') {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_FILE && *model_count < MODEL_MAX_COUNT) {
			size_t file_name_len = strlen(entry.name);
			size_t model_name_len;

			if (file_name_len <= sizeof(suffix) - 1U ||
			    memcmp(entry.name + file_name_len - (sizeof(suffix) - 1U),
				   suffix, sizeof(suffix) - 1U) != 0) {
				continue;
			}

			model_name_len = file_name_len - (sizeof(suffix) - 1U);
			if (model_name_len >= MODEL_NAME_MAX) {
				continue;
			}

			memcpy(models[*model_count], entry.name, model_name_len);
			models[*model_count][model_name_len] = '\0';
			if (validate_model_name(models[*model_count]) == 0) {
				(*model_count)++;
			}
		}
	}

	fs_closedir(&dir);
	return rc < 0 ? rc : 0;
}

int model_fs_erase_all(void)
{
	const struct flash_area *area;
	int rc;

	rc = flash_area_open(PARTITION_ID(storage_partition), &area);
	if (rc != 0) {
		LOG_ERR("flash_area_open failed: %d", rc);
		return rc;
	}

	rc = flash_area_flatten(area, 0, area->fa_size);
	flash_area_close(area);
	if (rc != 0) {
		LOG_ERR("flash_area_flatten failed: %d", rc);
		return rc;
	}

	rc = fs_unmount(&model_fs_mount);
	if (rc != 0 && rc != -EINVAL) {
		LOG_ERR("fs_unmount failed: %d", rc);
		return rc;
	}

	rc = fs_mount(&model_fs_mount);
	if (rc != 0) {
		LOG_ERR("fs_mount after erase failed: %d", rc);
		return rc;
	}

	LOG_INF("LittleFS storage partition erased and remounted");
	return 0;
}
