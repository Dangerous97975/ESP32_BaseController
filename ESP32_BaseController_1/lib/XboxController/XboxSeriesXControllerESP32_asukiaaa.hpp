#pragma once

#include <NimBLEDevice.h>
#include "XboxControllerNotificationParser.h"

#include "XboxSeriesXHIDReportBuilder_asukiaaa.hpp"

// #define XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL Serial
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
const unsigned long printInterval = 100UL;
#endif

namespace XboxSeriesXControllerESP32_asukiaaa
{

  // 定義標準 BLE 服務 UUID
  static NimBLEUUID uuidServiceGeneral("1801"); // 一般屬性設定
  static NimBLEUUID uuidServiceBattery("180f"); // 電池服務
  static NimBLEUUID uuidServiceHid("1812");     // HID (人機介面) 服務 - 關鍵數據通道
  
  // 定義特徵值 UUID (Characteristics)
  static NimBLEUUID uuidCharaReport("2a4d");    // HID 報告
  static NimBLEUUID uuidCharaPnp("2a50");       // Plug and Play ID
  static NimBLEUUID uuidCharaHidInformation("2a4a");
  static NimBLEUUID uuidCharaPeripheralAppearance("2a01");
  static NimBLEUUID uuidCharaPeripheralControlParameters("2a04");

  static NimBLEAdvertisedDevice *advDevice;
  static NimBLEClient *pConnectedClient = nullptr;

  // Xbox 手把的識別特徵
  static const uint16_t controllerAppearance = 964; // BLE 外觀代碼 (Gamepath)
  static const String controllerManufacturerDataNormal = "060000"; // 廠商數據
  static const String controllerManufacturerDataSearching = "0600030080";

  // 連線狀態機
  enum class ConnectionState : uint8_t
  {
    Connected = 0,               // 已連線
    WaitingForFirstNotification = 1, // 連線建立，等待第一筆數據
    Found = 2,                   // 掃描到目標，準備連線
    Scanning = 3,                // 掃描中
  };

  // 用戶端回調函式：處理連線與斷線事件
  class ClientCallbacks : public NimBLEClientCallbacks
  {
  public:
    ConnectionState *pConnectionState;
    ClientCallbacks(ConnectionState *pConnectionState)
    {
      this->pConnectionState = pConnectionState;
    }

    // 當連線成功建立時
    void onConnect(NimBLEClient *pClient)
    {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Connected");
#endif
      *pConnectionState = ConnectionState::WaitingForFirstNotification;
      // pClient->updateConnParams(120,120,0,60); // 更新連線參數 (間隔/延遲/超時)
    };

    // 當連線中斷時 (Safety Critical)
    void onDisconnect(NimBLEClient *pClient)
    {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.print(
          pClient->getPeerAddress().toString().c_str());
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(" Disconnected");
#endif
      // 狀態設為掃描，觸發 Core::onLoop 中的重連機制
      *pConnectionState = ConnectionState::Scanning;
      pConnectedClient = nullptr;
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    // 處理配對密鑰請求 (Xbox 通常不需要輸入 PIN，但協議需要處理)
    uint32_t onPassKeyRequest()
    {
      // Serial.println("Client Passkey Request");
      /** return the passkey to send to the server */
      return 0;
    };

    bool onConfirmPIN(uint32_t pass_key)
    {
      // Serial.print("The passkey YES/NO number: ");
      // Serial.println(pass_key);
      /** Return false if passkeys don't match. */
      return true;
    };

    /** Pairing process complete, we can check the results in ble_gap_conn_desc */
    // 配對完成後的驗證
    void onAuthenticationComplete(ble_gap_conn_desc *desc)
    {
      // Serial.println("onAuthenticationComplete");
      if (!desc->sec_state.encrypted)
      {
        // Serial.println("Encrypt connection failed - disconnecting");
        /** Find the client with the connection handle provided in desc */
        // 若未加密連線則主動斷開 (Xbox 要求加密連線)
        NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
        return;
      }
    };
  };

  /** Define a class to handle the callbacks when advertisments are received */
  // 廣告廣播掃描回調：過濾出 Xbox 手把
  class AdvertisedDeviceCallbacks : public NimBLEAdvertisedDeviceCallbacks
  {
  public:
    AdvertisedDeviceCallbacks(String strTargetDeviceAddress,
                              ConnectionState *pConnectionState)
    {
      if (strTargetDeviceAddress != "")
      {
        this->targetDeviceAddress =
            new NimBLEAddress(strTargetDeviceAddress.c_str());
      }
      this->pConnectionState = pConnectionState;
    }

  private:
    NimBLEAddress *targetDeviceAddress = nullptr;
    ConnectionState *pConnectionState;
    
    // 每發現一個藍牙裝置都會呼叫此函式
    void onResult(NimBLEAdvertisedDevice *advertisedDevice)
    {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      // Print debug info...
#endif
      char *pHex = NimBLEUtils::buildHexData(
          nullptr, (uint8_t *)advertisedDevice->getManufacturerData().data(),
          advertisedDevice->getManufacturerData().length());
      
      // 判斷邏輯：
      // 1. 若有指定 MAC 地址，則檢查地址是否相符
      // 2. 若無指定地址，則檢查外觀代碼(Appearance) 與 廠商數據(Manufacturer Data) 
      //    以及是否包含 HID 服務 UUID
      if ((targetDeviceAddress != nullptr &&
           advertisedDevice->getAddress().equals(*targetDeviceAddress)) ||
          (targetDeviceAddress == nullptr &&
           advertisedDevice->getAppearance() == controllerAppearance &&
           (strcmp(pHex, controllerManufacturerDataNormal.c_str()) == 0 ||
            strcmp(pHex, controllerManufacturerDataSearching.c_str()) == 0) &&
           advertisedDevice->getServiceUUID().equals(uuidServiceHid)))
      // if (advertisedDevice->isAdvertisingService(uuidServiceHid))
      {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Found target");
#endif
        /** stop scan before connecting */
        // NimBLEDevice::getScan()->stop();
        /** Save the device reference in a global for the client to use*/
        *pConnectionState = ConnectionState::Found; // 標記為已找到
        advDevice = advertisedDevice;
      }
    };
  };

  // 核心控制類別
  class Core
  {
  public:
    Core(String targetDeviceAddress = "")
    {
      this->advDeviceCBs =
          new AdvertisedDeviceCallbacks(targetDeviceAddress, &connectionState);
      this->clientCBs = new ClientCallbacks(&connectionState);
    }

    AdvertisedDeviceCallbacks *advDeviceCBs;
    ClientCallbacks *clientCBs;
    uint8_t battery = 0; // 電池電量
    static const int deviceAddressLen = 6;
    uint8_t deviceAddressArr[deviceAddressLen];

    // 初始化 BLE 硬體設定
    void begin()
    {
      NimBLEDevice::setScanFilterMode(CONFIG_BTDM_SCAN_DUPL_TYPE_DEVICE);
      // NimBLEDevice::setScanDuplicateCacheSize(200);
      NimBLEDevice::init("");
      NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
      NimBLEDevice::setSecurityAuth(true, false, false); // 設定安全認證需求
      NimBLEDevice::setPower(ESP_PWR_LVL_P9); /* +9db (最大發射功率，增加連線距離) */
    }

    // 發送 HID 報告給手把 (例如控制震動回饋)
    void writeHIDReport(uint8_t *dataArr, size_t dataLen)
    {
      if (pConnectedClient == nullptr)
      {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("no connnected client");
#endif
        return;
      }
      NimBLEClient *pClient = pConnectedClient;
      auto pService = pClient->getService(uuidServiceHid);
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(pService->toString().c_str());
#endif
      // 遍歷所有特徵值，找到可寫入的進行發送
      for (auto pChara : *pService->getCharacteristics())
      {
        if (pChara->canWrite())
        {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          // Debug prints...
          writeWithComment(pChara, dataArr, dataLen);
#else
          pChara->writeValue(dataArr, dataLen, false); // 寫入數據
#endif
        }
      }
    }

    void writeHIDReport(
        const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase &repo)
    {
      writeHIDReport((uint8_t *)repo.arr8t, repo.arr8tLen);
    }

    void writeHIDReport(
        const XboxSeriesXHIDReportBuilder_asukiaaa::ReportBeforeUnion &
            repoBeforeUnion)
    {
      XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
      repo.v = repoBeforeUnion;
      writeHIDReport((uint8_t *)repo.arr8t, repo.arr8tLen);
    }

    // 主循環：處理狀態機 (掃描 -> 連線 -> 維持)
    void onLoop()
    {
      if (!isConnected())
      {
        if (advDevice != nullptr)
        {
          // 若已發現裝置，嘗試連線
          auto connectionResult = connectToServer(advDevice);
          if (!connectionResult || !isConnected())
          {
            // 連線失敗，刪除綁定並重試
            NimBLEDevice::deleteBond(advDevice->getAddress());
            ++countFailedConnection;
            // reset();
            connectionState = ConnectionState::Scanning;
          }
          else
          {
            countFailedConnection = 0;
          }
          advDevice = nullptr;
        }
        else if (!isScanning())
        {
          // 若未發現且未掃描，啟動掃描
          // reset();
          startScan();
        }
      }
    }

    String buildDeviceAddressStr()
    {
      char buffer[18];
      auto addr = deviceAddressArr;
      snprintf(buffer, sizeof(buffer), "%02x:%02x:%02x:%02x:%02x:%02x", addr[5],
               addr[4], addr[3], addr[2], addr[1], addr[0]);
      return String(buffer);
    }

    // 啟動 BLE 掃描
    void startScan()
    {
      connectionState = ConnectionState::Scanning;
      auto pScan = NimBLEDevice::getScan();
      // pScan->clearResults();
      // pScan->clearDuplicateCache();
      pScan->setDuplicateFilter(false);
      pScan->setAdvertisedDeviceCallbacks(advDeviceCBs);
      // pScan->setActiveScan(true);
      pScan->setInterval(97); // 掃描間隔
      pScan->setWindow(97);   // 掃描視窗
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Start scan");
#endif
      // 啟動掃描，scanCompleteCB 為掃描結束回調
      pScan->start(scanTime, &Core::scanCompleteCB, false);
    }

    XboxControllerNotificationParser xboxNotif; // 通知解析器 (解析按鍵/搖桿)

    bool isWaitingForFirstNotification()
    {
      return connectionState == ConnectionState::WaitingForFirstNotification;
    }
    bool isConnected()
    {
      return connectionState == ConnectionState::WaitingForFirstNotification ||
             connectionState == ConnectionState::Connected;
    }
    unsigned long getReceiveNotificationAt() { return receivedNotificationAt; }
    uint8_t getCountFailedConnection() { return countFailedConnection; }

  private:
    ConnectionState connectionState = ConnectionState::Scanning;
    unsigned long receivedNotificationAt = 0;
    uint32_t scanTime = 4; /** 0 = scan forever */
    uint8_t countFailedConnection = 0;
    uint8_t retryCountInOneConnection = 3; // 單次連線重試次數
    unsigned long retryIntervalMs = 100;
    NimBLEClient *pClient = nullptr;

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    static void writeWithComment(NimBLERemoteCharacteristic *pChara,
                                 uint8_t *data, size_t len)
    {
       // Debug write implementation...
    }
#endif

    static void readAndPrint(NimBLERemoteCharacteristic *pChara)
    {
       // Debug read implementation...
    }

    bool isScanning() { return NimBLEDevice::getScan()->isScanning(); }

    // void reset() {
    //   NimBLEDevice::deinit(true);
    //   delay(500);
    //   begin();
    //   delay(500);
    // }

    /** Handles the provisioning of clients and connects / interfaces with the
     * server */
    // 連線至伺服器 (手把)
    bool connectToServer(NimBLEAdvertisedDevice *advDevice)
    {
      NimBLEClient *pClient = nullptr;

      /** Check if we have a client we should reuse first **/
      // 檢查是否可重複使用舊的 Client 物件
      if (NimBLEDevice::getClientListSize())
      {
        pClient = NimBLEDevice::getClientByPeerAddress(advDevice->getAddress());
        if (pClient)
        {
          pClient->connect();
        }
      }

      /** No client to reuse? Create a new one. */
      // 若無可用 Client，創建新的
      if (!pClient)
      {
        if (NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS)
        {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
              "Max clients reached - no more connections available");
#endif
          return false;
        }

        pClient = NimBLEDevice::createClient();

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("New client created");
#endif
        pClient->setClientCallbacks(clientCBs, true);
        pClient->connect(advDevice, true);
      }

      // 重試邏輯
      int retryCount = retryCountInOneConnection;
      while (!pClient->isConnected())
      {
        if (retryCount <= 0)
        {
          return false;
        }
        else
        {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Try connection. left: " +
                                                        String(retryCount));
#endif
        }

        delay(retryIntervalMs);
        pClient->connect(true);
        --retryCount;
      }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      // Print connected status RSSI...
#endif

      // 連線後處理：探索服務與特徵值
      bool result = afterConnect(pClient);
      if (!result)
      {
        return result;
      }

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Done with this device!");
#endif
      pConnectedClient = pClient;
      return true;
    }

    // 連線建立後的初始化動作
    bool afterConnect(NimBLEClient *pClient)
    {
      memcpy(deviceAddressArr, pClient->getPeerAddress().getNative(),
             deviceAddressLen);
      // 遍歷所有服務 (Service)
      for (auto pService : *pClient->getServices(true))
      {
        auto sUuid = pService->getUUID();
        // 只關心 HID 與 Battery 服務
        if (!sUuid.equals(uuidServiceHid) && !sUuid.equals(uuidServiceBattery))
        {
          continue; // skip
        }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            pService->toString().c_str());
#endif
        // 遍歷特徵值並訂閱通知 (Subscribe Notification)
        for (auto pChara : *pService->getCharacteristics(true))
        {
          charaHandle(pChara);
          charaSubscribeNotification(pChara);
        }
      }

      return true;
    }

#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
    void charaPrintId(NimBLERemoteCharacteristic *pChara)
    {
       // Debug...
    }

    static void printValue(std::__cxx11::string str)
    {
       // Debug...
    }
#endif

    void charaHandle(NimBLERemoteCharacteristic *pChara)
    {
      if (pChara->canWrite())
      {
          // ...
      }
      if (pChara->canRead())
      {
        // 讀取特徵值 (為了某些特徵值的訂閱前置作業)
        auto str = pChara->readValue();
        if (str.size() == 0)
        {
          str = pChara->readValue();
        }
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        printValue(str);
#endif
      }
    }

    // 訂閱特徵值通知
    void charaSubscribeNotification(NimBLERemoteCharacteristic *pChara)
    {
      if (pChara->canNotify())
      {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        charaPrintId(pChara);
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(" canNotify ");
#endif
        // 註冊 notifyCB 作為回調函式
        if (pChara->subscribe(
                true,
                std::bind(&Core::notifyCB, this, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3,
                          std::placeholders::_4),
                true))
        {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
              "succeeded in subscribing");
#endif
        }
        else
        {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("failed subscribing");
#endif
        }
      }
    }

    // !! 核心數據接收函式 !!
    // 當手把狀態改變 (按鍵/搖桿) 時，系統會自動呼叫此函式
    void notifyCB(NimBLERemoteCharacteristic *pRemoteCharacteristic,
                  uint8_t *pData, size_t length, bool isNotify)
    {
      auto sUuid = pRemoteCharacteristic->getRemoteService()->getUUID();
      
      // 若是第一次收到通知，更新連線狀態為 Connected
      if (connectionState != ConnectionState::Connected)
      {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println(
            "Received first notification");
#endif
        connectionState = ConnectionState::Connected;
      }
      
      // 處理 HID 數據 (搖桿與按鍵)
      if (sUuid.equals(uuidServiceHid))
      {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        // Debug printing limited by interval...
#endif
        // 解析數據包並更新 xboxNotif 結構
        xboxNotif.update(pData, length);
        receivedNotificationAt = millis();
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
        // XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.print(xboxNotif.toString());
        // ...
#endif
      }
      else
      {
        // 處理電池數據
        if (sUuid.equals(uuidServiceBattery))
        {
          battery = pData[0];
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
          XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("battery notification");
#endif
        }
        else
        {
          // 未處理的其他通知
        }
      }
    }

    static void scanCompleteCB(NimBLEScanResults results)
    {
#ifdef XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL
      XBOX_SERIES_X_CONTROLLER_DEBUG_SERIAL.println("Scan Ended");
#endif
    }
  };

}; // namespace XboxSeriesXControllerESP32_asukiaaa