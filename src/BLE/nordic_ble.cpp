/*
 * nordic_ble.hpp
 *
 * Wrapper for the BLE portions of the Nordic UART Service example.
 *
 * Copyright (c) 2021 TrueTouch
 * Distributed under the MIT license (see LICENSE or https://opensource.org/licenses/MIT)
 */

#include "nordic_ble.hpp"
#include "util.hpp"

#include <app_error.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_nus.h>
#include <bsp.h>
#include <nrf_ble_gatt.h>
#include <nrf_ble_qwr.h>
#include <nrf_log.h>
#include <nrf_sdh.h>

#include <cstddef>

namespace ble {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private data
////////////////////////////////////////////////////////////////////////////////////////////////////
/** BLE NUS service instance. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);

/** GATT module instance. */
NRF_BLE_GATT_DEF(m_gatt);

/** Context for the Queued Write module.*/
NRF_BLE_QWR_DEF(m_qwr);

/** Advertising module instance. */
BLE_ADVERTISING_DEF(m_advertising);

/** Handle of the current connection. */
static std::uint16_t m_conn_handle { BLE_CONN_HANDLE_INVALID };

/** Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static std::uint16_t m_ble_nus_max_data_len { BLE_GATT_ATT_MTU_DEFAULT - 3 };

/** Universally unique service identifier. */
static ble_uuid_t m_adv_uuids[] {
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

/** BLE write event callbacks. */
std::uint32_t m_callback_cnt {};
UartCallback m_callbacks[CALLBACK_MAX] {};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private function prototypes
////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init();

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init();

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init();

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init();

/**@brief Function for initializing the GATT library. */
static void gatt_init();

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init();

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private handler/callback prototypes
////////////////////////////////////////////////////////////////////////////////////////////////////

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(std::uint32_t nrf_error);

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
static void nus_data_handler(ble_nus_evt_t * p_evt);

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(std::uint32_t nrf_error);

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

void init()
{
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
}

void disconnect()
{
    std::uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                   BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE) {
        APP_ERROR_CHECK(err_code);
    }
}

void advertise()
{
    std::uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

void advertise_no_whitelist()
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        std::uint32_t err_code = ble_advertising_restart_without_whitelist(&m_advertising);
        if (err_code != NRF_ERROR_INVALID_STATE) {
            APP_ERROR_CHECK(err_code);
        }
    }
}

std::uint16_t max_data_length()
{
   return m_ble_nus_max_data_len;
}

void send(std::uint8_t *data, std::uint16_t length)
{
    std::uint32_t err_code;
    do {
        err_code = ble_nus_data_send(&m_nus, data, &length, m_conn_handle);
        if ((err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
    } while (err_code == NRF_ERROR_RESOURCES);
}

void register_callback(UartCallback callback)
{
    if (m_callback_cnt < CALLBACK_MAX) {
       m_callbacks[m_callback_cnt] = callback;
       ++m_callback_cnt;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private function implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

static void gap_params_init()
{
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    std::uint32_t err_code = sd_ble_gap_device_name_set(&sec_mode,
                                                        (const uint8_t *) DEVICE_NAME,
                                                        strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_params_t gap_conn_params {};

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void services_init()
{
    // Initialize Queued Write Module.
    nrf_ble_qwr_init_t qwr_init {};
    qwr_init.error_handler = nrf_qwr_error_handler;

    std::uint32_t err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    ble_nus_init_t nus_init {};
    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void conn_params_init()
{
    ble_conn_params_init_t cp_init {};
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    std::uint32_t err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void ble_stack_init()
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    std::uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void gatt_init()
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

static void advertising_init()
{
    ble_advertising_init_t init {};
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = ADV_DURATION;
    init.evt_handler = on_adv_evt;

    std::uint32_t err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, CONN_CFG_TAG);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private handler/callback implementations
////////////////////////////////////////////////////////////////////////////////////////////////////

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

// TODO: implement haptic glove control protocol here
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (std::size_t i = 0; i < m_callback_cnt; ++i) {
            m_callbacks[i](p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        }
    }
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(std::uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            util::sleep_mode_enter();
            break;
        default:
            break;
    }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    std::uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .tx_phys = BLE_GAP_PHY_AUTO,
                .rx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

}  // namespace ble