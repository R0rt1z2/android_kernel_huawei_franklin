/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
 * Description: Huawei fbex ca communicate with TA
 * Author : security-ap
 * Create : 2020/07/16
 */

#include "fbex_driver.h"

#include "teek_client_api.h"
#include <teek_client_constants.h>
#include <linux/types.h>
#include <linux/random.h>

#define FBEX_UID       2012

/* uuid to TA: 29727d8c-f197-4339-9a0e-049fe708823a*/
#define UUID_TEEOS_UFS_INLINE_CRYPTO                                    \
	{                                                               \
		0x29727d8c, 0xf197, 0x4339,                             \
		{                                                       \
			0x9a, 0x0e, 0x04, 0x9f, 0xe7, 0x08, 0x82, 0x3a  \
		}                                                       \
	}

struct user_info {
	u32 user_id;
	u8 status;
};

static DEFINE_MUTEX(g_fbex_ta_mutex);

static u32 fbe_ca_invoke_command(u32 cmd, TEEC_Operation *op)
{
	TEEC_Context context;
	TEEC_Session session;
	TEEC_Result result;
	u32 origin;
	TEEC_UUID uuid_id = UUID_TEEOS_UFS_INLINE_CRYPTO;
	TEEC_Operation operation = { 0 };
	u32 root_id = FBEX_UID;
	const char *package_name = "ufs_key_restore";

	mutex_lock(&g_fbex_ta_mutex);
	result = TEEK_InitializeContext(NULL, &context);
	if (result != TEEC_SUCCESS) {
		pr_err("%s, TEEC init failed.\n", __func__);
		goto exit1;
	}
	/* pass TA's FULLPATH to TEE, then OpenSession */
	/* MUST use TEEC_LOGIN_IDENTIFY method */
	operation.started = 1;
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE,
						TEEC_MEMREF_TEMP_INPUT,
						TEEC_MEMREF_TEMP_INPUT);
	operation.params[2].tmpref.buffer = (void *)(&root_id);
	operation.params[2].tmpref.size = sizeof(root_id);
	operation.params[3].tmpref.buffer = (void *)package_name;
	operation.params[3].tmpref.size = (size_t)(strlen(package_name) + 1);

	result = TEEK_OpenSession(&context, &session, &uuid_id,
				  TEEC_LOGIN_IDENTIFY, NULL, &operation,
				  &origin);
	if (result != TEEC_SUCCESS) {
		pr_err("%s: open session failed, 0x%x\n", __func__, result);
		goto exit2;
	}

	result = TEEK_InvokeCommand(&session, cmd, op, &origin);
	if (result != TEEC_SUCCESS) {
		pr_err("%s: invoke failed, ret 0x%x origin 0x%x\n", __func__,
		       result, origin);
	}
	TEEK_CloseSession(&session);
exit2:
	TEEK_FinalizeContext(&context);
exit1:
	mutex_unlock(&g_fbex_ta_mutex);
	return (int)result;
}

void file_encry_record_error(u32 cmd, u32 user, u32 file, u32 error)
{
	pr_err("FBE: cmd 0x%x fail, user 0x%x, file 0x%x, error ID: 0x%x\n",
	       cmd, user, file, error);
}

u32 file_encry_undefined(u32 user _unused, u32 file _unused, u8 *iv _unused,
			 u32 iv_len _unused)
{
	pr_err("%s, into\n", __func__);
	return FBE2_ERROR_CMD_UNDEFINED;
}

u32 file_encry_unsupported(u32 user _unused, u32 file _unused, u8 *iv _unused,
			   u32 iv_len _unused)
{
	pr_err("%s, into\n", __func__);
	return FBE2_ERROR_CMD_UNSUPPORT;
}

/*
 * Function: file_encry_add_iv
 * Parameters:
 *    user: input, user id
 *    file: input, file type(DE/CE/ECE/SECE)
 *    iv: input/output, iv buffer
 *    iv_len: input, iv length
 * Description:
 *    This is called when user create. iv can be create one at a time.
 *    Function should be called 4 times for one user(DE/CE/ECE/SECE)
 */
u32 file_encry_add_iv(u32 user, u32 file, u8 *iv, u32 iv_len)
{
	u32 ret;

	TEEC_Operation operation = { 0 };

	if (!iv || iv_len != KEY_LEN) {
		pr_err("%s, input iv buffer is error 0x%x\n", __func__, iv_len);
		return FBE2_ERROR_IV_BUFFER;
	}
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INOUT,
						TEEC_VALUE_INPUT,
						TEEC_NONE,
						TEEC_NONE);
	operation.params[0].tmpref.buffer = (void *)iv;
	operation.params[0].tmpref.size = iv_len;
	operation.params[1].value.a = user;
	operation.params[1].value.b = file;

	operation.started = 1;
	ret = fbe_ca_invoke_command(SEC_FILE_ENCRY_CMD_ID_VOLD_ADD_IV,
				    &operation);
	return ret;
}

/*
 * Function: file_encry_delete_iv
 * Parameters:
 *    user: input, user id
 *    file: input, file type(DE/CE/ECE/SECE)
 *    iv: input, iv buffer(used for verify)
 *    iv_len: input, iv length
 * Description:
 *    This is called when user delete. iv can be delete one at a time.
 *    Function should be called 4 times for one user(DE/CE/ECE/SECE)
 */
u32 file_encry_delete_iv(u32 user, u32 file, u8 *iv, u32 iv_len)
{
	u32 ret;
	TEEC_Operation operation = { 0 };

	if (!iv || iv_len != KEY_LEN) {
		pr_err("%s, input iv buffer is error 0x%x\n", __func__, iv_len);
		return FBE2_ERROR_IV_BUFFER;
	}
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
						TEEC_VALUE_INPUT,
						TEEC_NONE, TEEC_NONE);
	operation.params[0].tmpref.buffer = (void *)iv;
	operation.params[0].tmpref.size = iv_len;
	operation.params[1].value.a = user;
	operation.params[1].value.b = file;

	operation.started = 1;
	ret = fbe_ca_invoke_command(SEC_FILE_ENCRY_CMD_ID_VOLD_DELETE_IV,
				    &operation);
	return ret;
}

/*
 * Function: file_encry_logout_iv
 * Parameters:
 *    user: input, user id
 *    file: input, file type(DE/CE/ECE/SECE)
 *    iv: input/output, iv buffer
 *    iv_len: input, iv length
 * Description:
 *    This is called when user logout. iv can be logout one at a time.
 *    Function should be called 3 times for one user(CE/ECE/SECE)
 */
u32 file_encry_logout_iv(u32 user, u32 file, u8 *iv, u32 iv_len)
{
	u32 ret;
	TEEC_Operation operation = { 0 };

	if (!iv || iv_len != KEY_LEN) {
		pr_err("%s, input iv buffer is error 0x%x\n", __func__, iv_len);
		return FBE2_ERROR_IV_BUFFER;
	}
	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT,
						TEEC_VALUE_INPUT,
						TEEC_NONE, TEEC_NONE);
	operation.params[0].tmpref.buffer = (void *)iv;
	operation.params[0].tmpref.size = iv_len;
	operation.params[1].value.a = user;
	operation.params[1].value.b = file;

	operation.started = 1;
	ret = fbe_ca_invoke_command(SEC_FILE_ENCRY_CMD_ID_USER_LOGOUT,
				    &operation);
	return ret;
}

u32 huawei_fbex_restore_key(void)
{
	TEEC_Operation operation = { 0 };

	operation.paramTypes = TEEC_PARAM_TYPES(TEEC_NONE, TEEC_NONE,
						TEEC_NONE, TEEC_NONE);
	operation.started = 1;
	return fbe_ca_invoke_command(SEC_FILE_ENCRY_CMD_ID_KEY_RESTORE,
				     &operation);
}

