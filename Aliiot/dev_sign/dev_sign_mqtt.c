#include <stdio.h>
#include <string.h>
#include "infra_defs.h"
#include "infra_config.h"
#include "infra_sha256.h"
#ifdef MQTT_PRE_AUTH
    #include "infra_preauth.h"
#endif

#include "dev_sign_api.h"
#include "dev_sign_wrapper.h"


#define SIGN_MQTT_PORT (1883)

/* all secure mode define */
#define MODE_TLS_GUIDER             "-1"
#define MODE_TLS_DIRECT             "2"
#define MODE_TCP_DIRECT_PLAIN       "3"

#ifdef MQTT_PRE_AUTH
    #define SECURE_MODE                 MODE_TLS_GUIDER
#else
    #ifdef SUPPORT_TLS
        #define SECURE_MODE                 MODE_TLS_DIRECT
    #else
        #define SECURE_MODE                 MODE_TCP_DIRECT_PLAIN
    #endif
#endif

/* use fixed timestamp */
#define TIMESTAMP_VALUE                 "2524608000000"

/* clientid key value pair define */
const char *clientid_kv[][2] = {
    {
        "timestamp", TIMESTAMP_VALUE
    },
    {
        "securemode", SECURE_MODE
    },
    {
        "signmethod", "hmacsha256"
    },
#if defined(DEVICE_MODEL_ENABLED) && !defined(DEVICE_MODEL_CLASSIC)
    {
        "v", IOTX_ALINK_VERSION
    },
#else
    {
        "gw", "0"
    },
    {
        "ext", "0"
    },
#endif
    {
        "a", "sdk-c-"IOTX_SDK_VERSION
    },
};

static void _hex2str(uint8_t *input, uint16_t input_len, char *output)
{
    char *zEncode = "0123456789ABCDEF";
    int i = 0, j = 0;

    for (i = 0; i < input_len; i++) {
        output[j++] = zEncode[(input[i] >> 4) & 0xf];
        output[j++] = zEncode[(input[i]) & 0xf];
    }
}

static int _sign_get_clientid(char *clientid_string, char *device_id)
{
    uint8_t i;

    if (clientid_string == NULL || device_id == NULL) {
        return FAIL_RETURN;
    }

    memset(clientid_string, 0, DEV_SIGN_CLIENT_ID_MAXLEN);
    memcpy(clientid_string, device_id, strlen(device_id));
    memcpy(clientid_string + strlen(clientid_string), "|", 1);

    for (i = 0; i < (sizeof(clientid_kv) / (sizeof(clientid_kv[0]))); i++) {
        if ((strlen(clientid_string) + strlen(clientid_kv[i][0]) + strlen(clientid_kv[i][1]) + 2) >=
            DEV_SIGN_CLIENT_ID_MAXLEN) {
            return FAIL_RETURN;
        }

        memcpy(clientid_string + strlen(clientid_string), clientid_kv[i][0], strlen(clientid_kv[i][0]));
        memcpy(clientid_string + strlen(clientid_string), "=", 1);
        memcpy(clientid_string + strlen(clientid_string), clientid_kv[i][1], strlen(clientid_kv[i][1]));
        memcpy(clientid_string + strlen(clientid_string), ",", 1);
    }

    memcpy(clientid_string + strlen(clientid_string) - 1, "|", 1);

    return SUCCESS_RETURN;
}

int32_t IOT_Sign_MQTT(iotx_mqtt_region_types_t region, iotx_dev_meta_info_t *meta, iotx_sign_mqtt_t *signout)
{
    uint16_t length = 0;
    char device_id[IOTX_PRODUCT_KEY_LEN + IOTX_DEVICE_NAME_LEN + 1] = {0};
    char signsource[DEV_SIGN_SOURCE_MAXLEN] = {0};
    uint16_t signsource_len = 0;
    uint8_t sign[32] = {0};
    const char *sign_fmt = "clientId%sdeviceName%sproductKey%stimestamp%s";

    if (region >= IOTX_MQTT_DOMAIN_NUMBER || meta == NULL) {
        return -1;
    }

    memset(signout, 0, sizeof(iotx_sign_mqtt_t));

    memcpy(device_id, meta->product_key, strlen(meta->product_key));
    memcpy(device_id + strlen(device_id), ".", strlen("."));
    memcpy(device_id + strlen(device_id), meta->device_name, strlen(meta->device_name));

    signsource_len = strlen(sign_fmt) + strlen(device_id) + strlen(meta->device_name) + strlen(meta->product_key) + strlen(
                                 TIMESTAMP_VALUE) + 1;
    if (signsource_len >= DEV_SIGN_SOURCE_MAXLEN) {
        return ERROR_DEV_SIGN_SOURCE_TOO_SHORT;
    }
    memset(signsource, 0, DEV_SIGN_SOURCE_MAXLEN);
    memcpy(signsource, "clientId", strlen("clientId"));
    memcpy(signsource + strlen(signsource), device_id, strlen(device_id));
    memcpy(signsource + strlen(signsource), "deviceName", strlen("deviceName"));
    memcpy(signsource + strlen(signsource), meta->device_name, strlen(meta->device_name));
    memcpy(signsource + strlen(signsource), "productKey", strlen("productKey"));
    memcpy(signsource + strlen(signsource), meta->product_key, strlen(meta->product_key));
    memcpy(signsource + strlen(signsource), "timestamp", strlen("timestamp"));
    memcpy(signsource + strlen(signsource), TIMESTAMP_VALUE, strlen(TIMESTAMP_VALUE));

    utils_hmac_sha256((uint8_t *)signsource, strlen(signsource), (uint8_t *)meta->device_secret,
                      strlen(meta->device_secret), sign);

    /* setup clientid */
    if (_sign_get_clientid(signout->clientid, device_id) != SUCCESS_RETURN) {
        return ERROR_DEV_SIGN_CLIENT_ID_TOO_SHORT;
    }

#if ( defined(MQTT_DEFAULT_IMPL) && defined(MQTT_DIRECT) ) || !defined(MQTT_DEFAULT_IMPL)
    /* setup password */
    memset(signout->password, 0, DEV_SIGN_PASSWORD_MAXLEN);
    _hex2str(sign, 32, signout->password);

    /* setup hostname */
    length = strlen(meta->product_key) + strlen(g_infra_mqtt_domain[region]) + 2;
    if (length >= DEV_SIGN_HOSTNAME_MAXLEN) {
        return ERROR_DEV_SIGN_HOST_NAME_TOO_SHORT;
    }
    memset(signout->hostname, 0, DEV_SIGN_HOSTNAME_MAXLEN);
    memcpy(signout->hostname, meta->product_key, strlen(meta->product_key));
    memcpy(signout->hostname + strlen(signout->hostname), ".", strlen("."));
    memcpy(signout->hostname + strlen(signout->hostname), g_infra_mqtt_domain[region],
           strlen(g_infra_mqtt_domain[region]));

    /* setup username */
    length = strlen(meta->device_name) + strlen(meta->product_key) + 2;
    if (length >= DEV_SIGN_USERNAME_MAXLEN) {
        return ERROR_DEV_SIGN_USERNAME_TOO_SHORT;
    }
    memset(signout->username, 0, DEV_SIGN_USERNAME_MAXLEN);
    memcpy(signout->username, meta->device_name, strlen(meta->device_name));
    memcpy(signout->username + strlen(signout->username), "&", strlen("&"));
    memcpy(signout->username + strlen(signout->username), meta->product_key, strlen(meta->product_key));

    /* setup port */
#ifdef SUPPORT_TLS
    signout->port = 443;
#else
    signout->port = 1883;
#endif /* #ifdef SUPPORT_TLS */
    return SUCCESS_RETURN;
#elif defined(MQTT_PRE_AUTH)
    {
        char sign_string[DEV_SIGN_PASSWORD_MAXLEN];
        (void)length;

        _hex2str(sign, 32, sign_string);
        memset(signout->password, 0, DEV_SIGN_PASSWORD_MAXLEN);
        memset(signout->hostname, 0, DEV_SIGN_HOSTNAME_MAXLEN);
        memset(signout->username, 0, DEV_SIGN_USERNAME_MAXLEN);

        return preauth_get_connection_info(region, meta, sign_string, device_id,
                                           signout->hostname, &signout->port, signout->username, signout->password);
    }
#else
#error Invalid Configuration
#endif /* #ifdef MQTT_DIRECT */
}

