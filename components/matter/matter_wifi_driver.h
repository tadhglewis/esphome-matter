#pragma once

#include "esphome/core/defines.h"
#if defined(USE_MATTER) && defined(USE_WIFI) &&                                \
    defined(USE_MATTER_BLE_COMMISSIONING)

#include "esphome/components/wifi/wifi_component.h"

#include <platform/NetworkCommissioning.h>

#include <string>
#include <vector>

namespace esphome::matter {

struct MatterWiFiCredentials {
  std::string ssid;
  std::string password;

  bool empty() const { return this->ssid.empty(); }
  bool matches(chip::ByteSpan network_id) const {
    return network_id.size() == this->ssid.size() &&
           memcmp(network_id.data(), this->ssid.data(), network_id.size()) == 0;
  }
};

class ESPHomeWiFiDriver final
    : public chip::DeviceLayer::NetworkCommissioning::WiFiDriver,
      public wifi::WiFiScanResultsListener {
public:
  using Status = chip::DeviceLayer::NetworkCommissioning::Status;
  using Network = chip::DeviceLayer::NetworkCommissioning::Network;
  using NetworkIterator =
      chip::DeviceLayer::NetworkCommissioning::NetworkIterator;
  using WiFiScanResponse =
      chip::DeviceLayer::NetworkCommissioning::WiFiScanResponse;

  static ESPHomeWiFiDriver &instance();

  void setup_on_main_loop();

  CHIP_ERROR Init(NetworkStatusChangeCallback *callback) override;
  void Shutdown() override {}
  uint8_t GetMaxNetworks() override { return 1; }
  NetworkIterator *GetNetworks() override;
  uint8_t GetScanNetworkTimeoutSeconds() override { return 15; }
  uint8_t GetConnectNetworkTimeoutSeconds() override { return 30; }
  CHIP_ERROR CommitConfiguration() override;
  CHIP_ERROR RevertConfiguration() override;
  Status RemoveNetwork(chip::ByteSpan network_id,
                       chip::MutableCharSpan &out_debug_text,
                       uint8_t &out_network_index) override;
  Status ReorderNetwork(chip::ByteSpan network_id, uint8_t index,
                        chip::MutableCharSpan &out_debug_text) override;
  void ConnectNetwork(chip::ByteSpan network_id,
                      ConnectCallback *callback) override;
  Status AddOrUpdateNetwork(chip::ByteSpan ssid, chip::ByteSpan credentials,
                            chip::MutableCharSpan &out_debug_text,
                            uint8_t &out_network_index) override;
  void ScanNetworks(chip::ByteSpan ssid, ScanCallback *callback) override;

  void on_wifi_scan_results(
      const wifi::wifi_scan_vector_t<wifi::WiFiScanResult> &results) override;

protected:
  void start_connect_(MatterWiFiCredentials credentials);
  void poll_connect_();
  void finish_connect_(Status status);
  void restore_previous_network_();
  void start_scan_(std::string ssid);
  void on_scan_done_without_results_();
  void finish_scan_(Status status, std::vector<WiFiScanResponse> responses);

  MatterWiFiCredentials staging_;
  MatterWiFiCredentials committed_;
  ConnectCallback *connect_callback_{nullptr};
  ScanCallback *scan_callback_{nullptr};

  uint32_t connect_started_{0};
  bool connect_pending_{false};
  bool network_changed_{false};
  bool had_previous_network_{false};
  MatterWiFiCredentials previous_network_;
  std::string connecting_ssid_;
  bool scan_pending_{false};
  std::string scan_ssid_;
};

} // namespace esphome::matter

#endif // USE_MATTER && USE_WIFI && USE_MATTER_BLE_COMMISSIONING
