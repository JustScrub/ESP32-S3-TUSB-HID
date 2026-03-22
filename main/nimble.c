/*
 * NimBLE BLE interface for HID keyboard control
 * 
 * Provides two characteristics:
 * 1. Toggle characteristic: READ triggers transmit on/off toggle
 * 2. Key characteristic: READ/WRITE for current key (a-z only)
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "nimble";

#define BLE_DEV_NAME "ESP32-HID-Kbd"
#define BLE_CHA_TOGGLE_DESC "Toggle Transmit"
#define BLE_CHA_KEY_DESC "HID Key (a-z)"

// Extern API functions from main.c
extern void hid_set_transmit_enabled(bool enabled);
extern bool hid_get_transmit_enabled(void);
extern void hid_set_key(char key);
extern char hid_get_key(void);

// UUIDs for our custom service and characteristics
// Service UUID: 12345678-1234-1234-1234-123456789abc
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Toggle characteristic UUID: 12345678-1234-1234-1234-123456789abd
static const ble_uuid128_t gatt_svr_chr_toggle_uuid =
    BLE_UUID128_INIT(0xbd, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Key characteristic UUID: 12345678-1234-1234-1234-123456789abe
static const ble_uuid128_t gatt_svr_chr_key_uuid =
    BLE_UUID128_INIT(0xbe, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

// Attribute handles
static uint16_t toggle_handle;
static uint16_t key_handle;

/*
 * Toggle characteristic access callback
 * READ: toggles transmit state and returns new state (1 or 0)
 */
static int gatt_svr_chr_access_toggle(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Toggle the state
        bool current_state = hid_get_transmit_enabled();
        bool new_state = !current_state;
        hid_set_transmit_enabled(new_state);
        
        ESP_LOGI(TAG, "Toggle characteristic read: state changed from %d to %d", current_state, new_state);
        
        // Return the new state as a single byte
        uint8_t state_byte = new_state ? 1 : 0;
        rc = os_mbuf_append(ctxt->om, &state_byte, sizeof(state_byte));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/*
 * Key characteristic access callback
 * READ: returns current key
 * WRITE: sets new key (must be lowercase a-z)
 */
static int gatt_svr_chr_access_key(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        // Return current key
        char current_key = hid_get_key();
        ESP_LOGI(TAG, "Key characteristic read: '%c'", current_key);
        rc = os_mbuf_append(ctxt->om, &current_key, sizeof(current_key));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        // Validate and set new key
        if (OS_MBUF_PKTLEN(ctxt->om) != 1) {
            ESP_LOGW(TAG, "Key characteristic write: invalid length (expected 1 byte)");
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        
        uint8_t new_key;
        rc = ble_hs_mbuf_to_flat(ctxt->om, &new_key, sizeof(new_key), NULL);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        
        // Validate lowercase a-z
        if (new_key >= 'a' && new_key <= 'z') {
            hid_set_key((char)new_key);
            ESP_LOGI(TAG, "Key characteristic write: '%c'", new_key);
            return 0;
        } else {
            ESP_LOGW(TAG, "Key characteristic write: invalid key 0x%02x (only a-z supported)", new_key);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

    default:
        assert(0);
        return BLE_ATT_ERR_UNLIKELY;
    }
}

/* Descriptor access callbacks for user descriptions */
static int gatt_svr_dsc_access_toggle(uint16_t conn_handle, uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const char *desc = BLE_CHA_TOGGLE_DESC;
    int rc = os_mbuf_append(ctxt->om, desc, strlen(desc));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int gatt_svr_dsc_access_key(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const char *desc = BLE_CHA_KEY_DESC;
    int rc = os_mbuf_append(ctxt->om, desc, strlen(desc));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* GATT server definition */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: HID Control */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Toggle transmit (read to toggle) */
                .uuid = &gatt_svr_chr_toggle_uuid.u,
                .access_cb = gatt_svr_chr_access_toggle,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &toggle_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) { {
                    .uuid = BLE_UUID16_DECLARE(0x2901), /* Characteristic User Description */
                    .att_flags = BLE_ATT_F_READ,
                    .access_cb = gatt_svr_dsc_access_toggle,
                }, {
                    0, /* No more descriptors */
                } },
            }, {
                /* Characteristic: Set/Get key (read/write without response) */
                .uuid = &gatt_svr_chr_key_uuid.u,
                .access_cb = gatt_svr_chr_access_key,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &key_handle,
                .descriptors = (struct ble_gatt_dsc_def[]) { {
                    .uuid = BLE_UUID16_DECLARE(0x2901), /* Characteristic User Description */
                    .att_flags = BLE_ATT_F_READ,
                    .access_cb = gatt_svr_dsc_access_key,
                }, {
                    0, /* No more descriptors */
                } },
            }, {
                0, /* No more characteristics in this service */
            } },
    },
    {
        0, /* No more services */
    },
};

/* GAP event handler */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        ESP_LOGI(TAG, "Connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
        }
        
        /* Start advertising again if connection failed */
        if (event->connect.status != 0) {
            extern void nimble_advertise(void);
            nimble_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
        
        /* Start advertising again */
        extern void nimble_advertise(void);
        nimble_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertise complete; reason=%d", event->adv_complete.reason);
        extern void nimble_advertise(void);
        nimble_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscribe event; conn_handle=%d attr_handle=%d",
                 event->subscribe.conn_handle,
                 event->subscribe.attr_handle);
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update event; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle,
                 event->mtu.channel_id,
                 event->mtu.value);
        return 0;
    }

    return 0;
}

/* Start advertising */
void nimble_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    const char *name;
    int rc;

    /* Set the advertisement data (name only) */
    memset(&fields, 0, sizeof fields);

    /* Advertise name */
    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Set scan response data (UUID) */
    memset(&rsp_fields, 0, sizeof rsp_fields);
    rsp_fields.uuids128 = &gatt_svr_svc_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response data; rc=%d", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error enabling advertisement; rc=%d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "Advertising started");
}

/* BLE host task */
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* Reset callback */
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
}

/* Sync callback */
static void on_sync(void)
{
    int rc;

    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin advertising */
    nimble_advertise();
}

/* Initialize NimBLE */
void nimble_init(void)
{
    int rc;

    ESP_LOGI(TAG, "Initializing NimBLE");

    /* Initialize NVS — required for BLE */
    rc = nvs_flash_init();
    if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(rc);

    /* Initialize NimBLE */
    ESP_ERROR_CHECK(nimble_port_init());

    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    /* Set device name */
    rc = ble_svc_gap_device_name_set(BLE_DEV_NAME);
    assert(rc == 0);

    /* Initialize GATT services */
    ble_svc_gap_init();
    ble_svc_gatt_init();

    /* Register custom GATT services */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    /* Start the BLE host task */
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "NimBLE initialized");
    ESP_LOGI(TAG, "Device name: %s", BLE_DEV_NAME);
    ESP_LOGI(TAG, "Service UUID: 12345678-1234-1234-1234-123456789abc");
    ESP_LOGI(TAG, "  Toggle characteristic: ...89abd (READ to toggle)");
    ESP_LOGI(TAG, "  Key characteristic: ...89abe (READ/WRITE for a-z key)");
}
