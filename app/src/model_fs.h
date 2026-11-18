#ifndef RC_REMOTE_CONTROLLER_MODEL_FS_H
#define RC_REMOTE_CONTROLLER_MODEL_FS_H

#include <stddef.h>
#include <stdint.h>

#include "model.h"

#ifdef __cplusplus
extern "C" {
#endif

int model_fs_init(void);
int model_fs_save_model(const char *name, const model_t *model);
int model_fs_load_model(const char *name, model_t *model);
int model_fs_delete_model(const char *name);
int model_fs_list_models(char models[][MODEL_NAME_MAX], size_t *model_count);
int model_fs_erase_all(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_REMOTE_CONTROLLER_MODEL_FS_H */
