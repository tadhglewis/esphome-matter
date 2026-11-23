#include "esphome/core/defines.h"
#if defined(USE_MATTER) && defined(USE_WIFI) &&                                \
    defined(USE_MATTER_BLE_COMMISSIONING)

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "matter_component.h"
#include "matter_wifi_driver.h"

#include <app/clusters/network-commissioning/NetworkCommissioningCluster.h>
#include <app/server/Server.h>
#include <data_model_provider/clusters/general_commissioning/integration.h>
#include <data_model_provider/esp_matter_data_model_provider.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <platform/CHIPDeviceLayer.h>
#include <platform/DeviceControlServer.h>
#include <sdkconfig.h>

#include <algorithm>
#include <cstring>

static const char *const TAG = "matter.wifi";

namespace esphome::matter {

namespace {

using chip::DeviceLayer::NetworkCommissioning::WiFiBand;
using chip::DeviceLayer::NetworkCommissioning::WiFiSecurity;
using chip::DeviceLayer::NetworkCommissioning::WirelessSignalType;

constexpr uint32_t CONNECT_TIMEOUT_MS = 25000;
constexpr uint32_t CONNECT_POLL_MS = 250;
constexpr uint32_t SCAN_TIMEOUT_MS = 12000;
constexpr const char *CONNECT_POLL_NAME = "matter_wifi_connect";
constexpr const char *SCAN_TIMEOUT_NAME = "matter_wifi_scan";
constexpr const char *SCAN_DONE_NAME = "matter_wifi_scan_done";
constexpr uint32_t SCAN_DONE_GRACE_MS = 500;

class NetworkIteratorImpl final : public ESPHomeWiFiDriver::NetworkIterator {
public:
  NetworkIteratorImpl(const MatterWiFiCredentials &network, bool connected)
      : ssid_(network.ssid), connected_(connected) {}
  size_t Count() override { return this->ssid_.empty() ? 0 : 1; }
  bool Next(ESPHomeWiFiDriver::Network &item) override {
    if (this->exhausted_ || this->ssid_.empty())
      return false;
    size_t length = std::min(this->ssid_.size(), sizeof(item.networkID));
    memcpy(item.networkID, this->ssid_.data(), length);
    item.networkIDLen = static_cast<uint8_t>(length);
    item.connected = this->connected_;
    this->exhausted_ = true;
    return true;
  }
  void Release() override { delete this; }

protected:
  std::string ssid_;
  bool connected_;
  bool exhausted_{false};
};

class ScanIteratorImpl final
    : public chip::DeviceLayer::NetworkCommissioning::WiFiScanResponseIterator {
public:
  explicit ScanIteratorImpl(
      std::vector<ESPHomeWiFiDriver::WiFiScanResponse> responses)
      : responses_(std::move(responses)) {}
  size_t Count() override { return this->responses_.size(); }
  bool Next(ESPHomeWiFiDriver::WiFiScanResponse &item) override {
    if (this->index_ >= this->responses_.size())
      return false;
    item = this->responses_[this->index_++];
    return true;
  }
  void Release() override { delete this; }

protected:
  std::vector<ESPHomeWiFiDriver::WiFiScanResponse> responses_;
  size_t index_{0};
};

ESPHomeWiFiDriver::WiFiScanResponse
make_scan_response(const char *ssid, size_t ssid_length, const uint8_t *bssid,
                   uint8_t channel, int8_t rssi, bool with_auth) {
  ESPHomeWiFiDriver::WiFiScanResponse response{};
  if (with_auth) {
    response.security.Set(WiFiSecurity::kWpa2Personal);
  } else {
    response.security.Set(WiFiSecurity::kUnencrypted);
  }
  size_t length = std::min(ssid_length, sizeof(response.ssid));
  memcpy(response.ssid, ssid, length);
  response.ssidLen = static_cast<uint8_t>(length);
  memcpy(response.bssid, bssid, sizeof(response.bssid));
  response.channel = channel;
  response.wiFiBand = WiFiBand::k2g4;
  response.signal.type = WirelessSignalType::kdBm;
  response.signal.strength = rssi;
  return response;
}

class BreadcrumbTracker final : public chip::app::Clusters::BreadCrumbTracker {
public:
  void SetBreadCrumb(uint64_t value) override {
    chip::app::Clusters::GeneralCommissioning::Instance()->SetBreadCrumb(value);
  }
};

BreadcrumbTracker breadcrumb_tracker;
chip::app::LazyRegisteredServerCluster<
    chip::app::Clusters::NetworkCommissioningCluster>
    network_commissioning_server;

} // namespace

ESPHomeWiFiDriver &ESPHomeWiFiDriver::instance() {
  static ESPHomeWiFiDriver driver;
  return driver;
}

void ESPHomeWiFiDriver::setup_on_main_loop() {
  auto *wifi = wifi::global_wifi_component;
  if (wifi == nullptr)
    return;
  wifi->add_scan_results_listener(this);
  esp_event_handler_register(
      WIFI_EVENT, WIFI_EVENT_SCAN_DONE,
      [](void *arg, esp_event_base_t, int32_t, void *) {
        auto *driver = static_cast<ESPHomeWiFiDriver *>(arg);
        global_matter_component->set_main_loop_timeout(
            SCAN_DONE_NAME, SCAN_DONE_GRACE_MS,
            [driver]() { driver->on_scan_done_without_results_(); });
      },
      this);
  if (wifi->has_sta()) {
    auto sta = wifi->get_sta();
    this->committed_.ssid = sta.get_ssid().c_str();
    this->committed_.password = sta.get_password().c_str();
  }
  this->staging_ = this->committed_;
}

CHIP_ERROR ESPHomeWiFiDriver::Init(NetworkStatusChangeCallback *callback) {
  return CHIP_NO_ERROR;
}

ESPHomeWiFiDriver::NetworkIterator *ESPHomeWiFiDriver::GetNetworks() {
  bool connected = wifi::global_wifi_component != nullptr &&
                   wifi::global_wifi_component->is_connected();
  return new NetworkIteratorImpl(this->staging_, connected);
}

ESPHomeWiFiDriver::Status ESPHomeWiFiDriver::AddOrUpdateNetwork(
    chip::ByteSpan ssid, chip::ByteSpan credentials,
    chip::MutableCharSpan &out_debug_text, uint8_t &out_network_index) {
  out_debug_text.reduce_size(0);
  out_network_index = 0;
  if (!this->staging_.empty() && !this->staging_.matches(ssid))
    return Status::kBoundsExceeded;
  if (ssid.empty() || ssid.size() > 32 || credentials.size() > 64)
    return Status::kOutOfRange;
  this->staging_.ssid.assign(reinterpret_cast<const char *>(ssid.data()),
                             ssid.size());
  this->staging_.password.assign(
      reinterpret_cast<const char *>(credentials.data()), credentials.size());
  return Status::kSuccess;
}

ESPHomeWiFiDriver::Status
ESPHomeWiFiDriver::RemoveNetwork(chip::ByteSpan network_id,
                                 chip::MutableCharSpan &out_debug_text,
                                 uint8_t &out_network_index) {
  out_debug_text.reduce_size(0);
  out_network_index = 0;
  if (!this->staging_.matches(network_id))
    return Status::kNetworkIDNotFound;
  this->staging_ = {};
  return Status::kSuccess;
}

ESPHomeWiFiDriver::Status
ESPHomeWiFiDriver::ReorderNetwork(chip::ByteSpan network_id, uint8_t index,
                                  chip::MutableCharSpan &out_debug_text) {
  out_debug_text.reduce_size(0);
  if (index != 0)
    return Status::kOutOfRange;
  if (!this->staging_.matches(network_id))
    return Status::kNetworkIDNotFound;
  return Status::kSuccess;
}

CHIP_ERROR ESPHomeWiFiDriver::CommitConfiguration() {
  if (this->staging_.empty() ||
      (this->staging_.ssid == this->committed_.ssid &&
       this->staging_.password == this->committed_.password))
    return CHIP_NO_ERROR;
  this->committed_ = this->staging_;
  MatterWiFiCredentials credentials = this->committed_;
  global_matter_component->defer_to_main_loop([this, credentials]() {
    ESP_LOGI(TAG, "Saving WiFi network '%s'", credentials.ssid.c_str());
    this->network_changed_ = false;
    wifi::global_wifi_component->save_wifi_sta(credentials.ssid,
                                               credentials.password);
  });
  return CHIP_NO_ERROR;
}

CHIP_ERROR ESPHomeWiFiDriver::RevertConfiguration() {
  this->staging_ = this->committed_;
  global_matter_component->defer_to_main_loop(
      [this]() { this->restore_previous_network_(); });
  return CHIP_NO_ERROR;
}

void ESPHomeWiFiDriver::restore_previous_network_() {
  if (!this->network_changed_)
    return;
  this->network_changed_ = false;
  auto *wifi = wifi::global_wifi_component;
  if (!this->had_previous_network_) {
    ESP_LOGW(TAG, "Reverting to no WiFi network");
    wifi->clear_sta();
    esp_wifi_disconnect();
    return;
  }
  ESP_LOGW(TAG, "Reverting to WiFi network '%s'",
           this->previous_network_.ssid.c_str());
  wifi::WiFiAP sta{};
  sta.set_ssid(this->previous_network_.ssid);
  sta.set_password(this->previous_network_.password);
  wifi->set_sta(sta);
  wifi->start_connecting(sta);
}

void ESPHomeWiFiDriver::ConnectNetwork(chip::ByteSpan network_id,
                                       ConnectCallback *callback) {
  if (!this->staging_.matches(network_id) || this->staging_.empty()) {
    if (callback != nullptr)
      callback->OnResult(Status::kNetworkIDNotFound, chip::CharSpan(), 0);
    return;
  }
  if (this->connect_callback_ != nullptr) {
    if (callback != nullptr)
      callback->OnResult(Status::kUnknownError, chip::CharSpan(), 0);
    return;
  }
  this->connect_callback_ = callback;
  MatterWiFiCredentials credentials = this->staging_;
  global_matter_component->defer_to_main_loop(
      [this, credentials]() { this->start_connect_(credentials); });
}

void ESPHomeWiFiDriver::start_connect_(MatterWiFiCredentials credentials) {
  auto *wifi = wifi::global_wifi_component;
  if (wifi == nullptr || wifi->is_disabled()) {
    this->finish_connect_(Status::kUnknownError);
    return;
  }

  char current_ssid[wifi::SSID_BUFFER_SIZE];
  if (wifi->has_sta() && wifi->is_connected() &&
      credentials.ssid == wifi->wifi_ssid_to(current_ssid)) {
    this->finish_connect_(Status::kSuccess);
    return;
  }

  if (!this->network_changed_) {
    this->had_previous_network_ = wifi->has_sta();
    if (this->had_previous_network_) {
      auto previous = wifi->get_sta();
      this->previous_network_.ssid = previous.get_ssid().c_str();
      this->previous_network_.password = previous.get_password().c_str();
      this->had_previous_network_ = !this->previous_network_.empty();
    }
  }
  this->network_changed_ = true;

  ESP_LOGI(TAG, "Connecting to WiFi network '%s'", credentials.ssid.c_str());
  wifi::WiFiAP sta{};
  sta.set_ssid(credentials.ssid);
  sta.set_password(credentials.password);
  wifi->set_sta(sta);
  wifi->start_connecting(sta);

  this->connecting_ssid_ = credentials.ssid;
  this->connect_started_ = millis();
  this->connect_pending_ = true;
  global_matter_component->set_main_loop_interval(
      CONNECT_POLL_NAME, CONNECT_POLL_MS, [this]() { this->poll_connect_(); });
}

void ESPHomeWiFiDriver::poll_connect_() {
  if (!this->connect_pending_)
    return;
  auto *wifi = wifi::global_wifi_component;
  char current_ssid[wifi::SSID_BUFFER_SIZE];
  if (wifi->is_connected() &&
      this->connecting_ssid_ == wifi->wifi_ssid_to(current_ssid)) {
    this->finish_connect_(Status::kSuccess);
    return;
  }
  if (millis() - this->connect_started_ > CONNECT_TIMEOUT_MS) {
    ESP_LOGW(TAG, "Timed out connecting to WiFi network");
    this->restore_previous_network_();
    this->finish_connect_(Status::kOtherConnectionFailure);
  }
}

void ESPHomeWiFiDriver::finish_connect_(Status status) {
  if (this->connect_pending_) {
    this->connect_pending_ = false;
    global_matter_component->cancel_main_loop_interval(CONNECT_POLL_NAME);
  }
  chip::DeviceLayer::SystemLayer().ScheduleLambda([this, status]() {
    ConnectCallback *callback = this->connect_callback_;
    this->connect_callback_ = nullptr;
    if (callback != nullptr)
      callback->OnResult(status, chip::CharSpan(), 0);
  });
}

void ESPHomeWiFiDriver::ScanNetworks(chip::ByteSpan ssid,
                                     ScanCallback *callback) {
  if (callback == nullptr)
    return;
  if (this->scan_callback_ != nullptr) {
    callback->OnFinished(Status::kUnknownError, chip::CharSpan(), nullptr);
    return;
  }
  this->scan_callback_ = callback;
  std::string scan_ssid(reinterpret_cast<const char *>(ssid.data()),
                        ssid.size());
  global_matter_component->defer_to_main_loop(
      [this, scan_ssid]() { this->start_scan_(scan_ssid); });
}

void ESPHomeWiFiDriver::start_scan_(std::string ssid) {
  auto *wifi = wifi::global_wifi_component;
  if (wifi == nullptr || wifi->is_disabled()) {
    this->finish_scan_(Status::kUnknownError, {});
    return;
  }

  if (wifi->has_sta() && wifi->is_connected()) {
    std::vector<WiFiScanResponse> responses;
    char current_ssid[wifi::SSID_BUFFER_SIZE];
    const char *name = wifi->wifi_ssid_to(current_ssid);
    if (ssid.empty() || ssid == name) {
      auto bssid = wifi->wifi_bssid();
      responses.push_back(
          make_scan_response(name, strlen(name), bssid.data(),
                             static_cast<uint8_t>(wifi->get_wifi_channel()),
                             wifi->wifi_rssi(), true));
    }
    this->finish_scan_(Status::kSuccess, std::move(responses));
    return;
  }
  if (wifi->has_sta()) {
    this->finish_scan_(Status::kUnknownError, {});
    return;
  }

  this->scan_ssid_ = std::move(ssid);
  this->scan_pending_ = true;
  wifi->start_scanning();
  global_matter_component->set_main_loop_timeout(
      SCAN_TIMEOUT_NAME, SCAN_TIMEOUT_MS, [this]() {
        if (!this->scan_pending_)
          return;
        ESP_LOGW(TAG, "WiFi scan timed out");
        this->scan_pending_ = false;
        this->finish_scan_(Status::kUnknownError, {});
      });
}

void ESPHomeWiFiDriver::on_scan_done_without_results_() {
  if (!this->scan_pending_)
    return;
  this->scan_pending_ = false;
  global_matter_component->cancel_main_loop_timeout(SCAN_TIMEOUT_NAME);
  this->finish_scan_(Status::kSuccess, {});
}

void ESPHomeWiFiDriver::on_wifi_scan_results(
    const wifi::wifi_scan_vector_t<wifi::WiFiScanResult> &results) {
  if (!this->scan_pending_)
    return;
  this->scan_pending_ = false;
  global_matter_component->cancel_main_loop_timeout(SCAN_TIMEOUT_NAME);
  global_matter_component->cancel_main_loop_timeout(SCAN_DONE_NAME);

  std::vector<WiFiScanResponse> responses;
  for (const auto &result : results) {
    auto ssid = result.get_ssid();
    if (ssid.empty())
      continue;
    if (!this->scan_ssid_.empty() && this->scan_ssid_ != ssid.c_str())
      continue;
    responses.push_back(make_scan_response(
        ssid.c_str(), ssid.size(), result.get_bssid().data(),
        result.get_channel(), result.get_rssi(), result.get_with_auth()));
  }
  ESP_LOGD(TAG, "WiFi scan found %u matching networks",
           static_cast<unsigned>(responses.size()));
  this->finish_scan_(Status::kSuccess, std::move(responses));
}

void ESPHomeWiFiDriver::finish_scan_(Status status,
                                     std::vector<WiFiScanResponse> responses) {
  ScanIteratorImpl *iterator = nullptr;
  if (status == Status::kSuccess)
    iterator = new ScanIteratorImpl(std::move(responses));
  chip::DeviceLayer::SystemLayer().ScheduleLambda([this, status, iterator]() {
    ScanCallback *callback = this->scan_callback_;
    this->scan_callback_ = nullptr;
    if (callback == nullptr) {
      delete iterator;
      return;
    }
    callback->OnFinished(status, chip::CharSpan(), iterator);
  });
}

} // namespace esphome::matter

extern "C" void esphome_matter_real_network_commissioning_init(
    chip::EndpointId endpoint_id) asm("__real__"
                                      "Z54ESPMatterNetworkCommissioningClusterS"
                                      "erverInitCallbackt");
extern "C" void esphome_matter_wrap_network_commissioning_init(
    chip::EndpointId endpoint_id) asm("__wrap__"
                                      "Z54ESPMatterNetworkCommissioningClusterS"
                                      "erverInitCallbackt");

extern "C" void
esphome_matter_wrap_network_commissioning_init(chip::EndpointId endpoint_id) {
  using namespace esphome::matter;
  if (endpoint_id != CONFIG_WIFI_NETWORK_ENDPOINT_ID) {
    esphome_matter_real_network_commissioning_init(endpoint_id);
    return;
  }

  if (!network_commissioning_server.IsConstructed()) {
    network_commissioning_server.Create(
        endpoint_id,
        static_cast<chip::DeviceLayer::NetworkCommissioning::WiFiDriver *>(
            &ESPHomeWiFiDriver::instance()),
        chip::app::Clusters::NetworkCommissioningCluster::Context{
            .breadcrumbTracker = breadcrumb_tracker,
            .failSafeContext = chip::Server::GetInstance().GetFailSafeContext(),
            .platformManager = chip::DeviceLayer::PlatformMgr(),
            .deviceControlServer =
                chip::DeviceLayer::DeviceControlServer::DeviceControlSvr(),
        });
  }
  CHIP_ERROR err = network_commissioning_server.Cluster().Init();
  if (err != CHIP_NO_ERROR)
    ESP_LOGE(TAG, "NetworkCommissioning init failed: %s", err.AsString());
  err = esp_matter::data_model::provider::get_instance().registry().Register(
      network_commissioning_server.Registration());
  if (err != CHIP_NO_ERROR)
    ESP_LOGE(TAG, "NetworkCommissioning register failed: %s", err.AsString());
}

extern "C" void esphome_matter_wrap_network_commissioning_shutdown(
    chip::EndpointId endpoint_id,
    chip::app::ClusterShutdownType
        type) asm("__wrap__"
                  "Z58ESPMatterNetworkCommissioningClusterServerShutdownCallbac"
                  "kt"
                  "N4chip3app19ClusterShutdownTypeE");
extern "C" void esphome_matter_real_network_commissioning_shutdown(
    chip::EndpointId endpoint_id,
    chip::app::ClusterShutdownType
        type) asm("__real__"
                  "Z58ESPMatterNetworkCommissioningClusterServerShutdownCallbac"
                  "kt"
                  "N4chip3app19ClusterShutdownTypeE");

extern "C" void esphome_matter_wrap_network_commissioning_shutdown(
    chip::EndpointId endpoint_id, chip::app::ClusterShutdownType type) {
  using namespace esphome::matter;
  if (endpoint_id != CONFIG_WIFI_NETWORK_ENDPOINT_ID) {
    esphome_matter_real_network_commissioning_shutdown(endpoint_id, type);
    return;
  }
  if (!network_commissioning_server.IsConstructed())
    return;
  CHIP_ERROR err =
      esp_matter::data_model::provider::get_instance().registry().Unregister(
          &network_commissioning_server.Cluster(), type);
  if (err != CHIP_NO_ERROR)
    ESP_LOGE(TAG, "NetworkCommissioning unregister failed: %s", err.AsString());
  network_commissioning_server.Cluster().Deinit();
  if (type == chip::app::ClusterShutdownType::kPermanentRemove)
    network_commissioning_server.Destroy();
}

#endif // USE_MATTER && USE_WIFI && USE_MATTER_BLE_COMMISSIONING
