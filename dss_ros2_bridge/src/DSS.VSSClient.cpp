/*
* VssClient.cpp (fixed)
*/
#include "DSS.VSSClient.h"
#include "dss.pb.h"
#include <thread>
#include <mutex>
#include <unordered_map>
#include <sstream>
#include <iostream>

namespace {
    using FnPtr = DSSVssClient::OnMessage*;
    using FnRawPtr = DSSVssClient::OnRawMessage*;

    std::mutex gSubMx;
    std::unordered_map<natsSubscription*, FnPtr> gClosures;
    std::unordered_map<natsSubscription*, FnRawPtr> gRawClosures;
}
DSSVssClient::DSSVssClient() {
    start();
}
DSSVssClient::~DSSVssClient() {
    stop();
}

DSSVssClient& DSSVssClient::singleton() {
    static DSSVssClient s;
    return s;
}

natsStatus DSSVssClient::start(const std::string& ip,uint16_t drivePort, uint16_t vssPort) {
    if (conn != nullptr)
        return NATS_ERR;

    natsStatus s = natsOptions_Create(&opts);
    if (s != NATS_OK) {
        std::cerr << "[VssClient] natsOptions_Create failed: " << natsStatus_GetText(s) << std::endl;
        return s;
    }

    natsOptions_SetMaxReconnect(opts, 60);
    natsOptions_SetReconnectWait(opts, 500);

    std::ostringstream oss;
    oss << "nats://" << ip << ":" << vssPort;
    const std::string url = oss.str();

    s = natsOptions_SetURL(opts, url.c_str());
    if (s == NATS_OK) {
        s = natsConnection_Connect(&conn, opts);
    }

    if (s != NATS_OK) {
        std::cerr << "[VssClient] connect failed (" << url << "): " << natsStatus_GetText(s) << std::endl;
        natsOptions_Destroy(opts);
        opts = nullptr;
        return s;
    }

    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
        std::cerr << "[VssClient] Failed to create UDP socket\n";
    } else {
        memset(&dss_addr_, 0, sizeof(dss_addr_));
        dss_addr_.sin_family = AF_INET;
        dss_addr_.sin_port   = htons(drivePort);
        inet_pton(AF_INET, ip.c_str(), &dss_addr_.sin_addr);

        std::cout << "[VssClient] UDP Control Ready: "  << ip << ":" << drivePort << std::endl;
    }    

    std::cout << "[VssClient] connected: " << url << std::endl;

    return NATS_OK;
}

void DSSVssClient::stop() {
    // 구독 정리(클로저 포함)
    {
        std::lock_guard<std::mutex> lock(gSubMx);
        for (auto& kv : gClosures) {
            natsSubscription_Destroy(kv.first);
            delete kv.second;
        }
        gClosures.clear();
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        for (auto& kv : gRawClosures) {
            natsSubscription_Destroy(kv.first);
            delete kv.second;
        }
        gRawClosures.clear();
    }

    if (udp_socket_ >= 0) {
        close(udp_socket_);
        udp_socket_ = -1;
    }

    if (conn) {
        natsConnection_Destroy(conn);
        conn = nullptr;
    }
    if (opts) {
        natsOptions_Destroy(opts);
        opts = nullptr;
    }
    nats_Close();
}

// ------------------------------
// 내부 유틸: 비동기 Request/Reply (인자 순서 수정)
// ------------------------------
void DSSVssClient::requestAsync(const std::string& subject, const std::string& payload, OnReply onReply,int64_t timeoutMs)
{
    std::thread([this, subject, payload, onReply, timeoutMs]() {
        if (conn == nullptr) {
            if (onReply) onReply(NATS_CONNECTION_CLOSED, {});
            return;
        }

        natsMsg* reply = nullptr;
        
        natsStatus s = natsConnection_RequestString(
            &reply,             // [1] out
            conn,               // [2]
            subject.c_str(),    // [3]
            payload.c_str(),    // [4]
            timeoutMs           // [5]
        );

        if (s == NATS_OK && reply != nullptr) {
            const char* data = natsMsg_GetData(reply);
            const int   len = natsMsg_GetDataLength(reply);
            std::string out(data, len);
            natsMsg_Destroy(reply);
            if (onReply) onReply(NATS_OK, out);
        }
        else {
            if (onReply) onReply(s, {});
        }
        }).detach();
}


void DSSVssClient::setDriveControl(float throttle, float steer, float brake) 
{
    if (udp_socket_ < 0) {
        std::cerr << "[VssClient] UDP socket not initialized!\n";
        return;
    }

    dss::DssSetControl ctrl;
    ctrl.set_identifier("DSS.ROSBridge.VSSClient");
    ctrl.set_timestamp(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    ctrl.set_throttle(throttle);
    ctrl.set_brake(brake);
    ctrl.set_steer(steer);

    std::string buffer;
    ctrl.SerializeToString(&buffer);

    ssize_t sent = sendto(udp_socket_,buffer.data(),buffer.size(),0,(struct sockaddr*)&dss_addr_,sizeof(dss_addr_));
    if (sent < 0) {
        std::cerr << "[VssClient] UDP send failed\n";
    }    
}

// ------------------------------
// GET / SET / ACTION
// ------------------------------
void DSSVssClient::get(const std::string& vssName,OnReply onReply,int64_t timeoutMs)
{
    char json[256];
    sprintf(json, "{ \"VSSName\": \"%s\" }", vssName.c_str());

    requestAsync("VSS.Get", json, std::move(onReply), timeoutMs);
}

void DSSVssClient::set(const std::string& vssName,const std::string& value,OnResult onResult,int64_t timeoutMs)
{
    char json[256];

    sprintf(json,
        "{ \"VSSName\": \"%s\", \"Value\": \"%s\" }",
        vssName.c_str(),
        value.c_str());

    requestAsync("VSS.Set", json, [onResult](natsStatus st, const std::string&) {
        if (onResult) {
            onResult(st);
        }
    },timeoutMs);
}

void DSSVssClient::action(const std::string& vssName, const std::string& value,OnReply onReply,int64_t timeoutMs)
{
    char json[256];

    sprintf(json,
        "{ \"VSSName\": \"%s\", \"Value\": \"%s\" }",
        vssName.c_str(),
        value.c_str());

    requestAsync("VSS.Act", json, std::move(onReply), timeoutMs);
}

// ------------------------------
// NATS 콜백 → 람다 bridge (불필요한 캐스팅 제거)
// ------------------------------
void DSSVssClient::natsMsgThunk(natsConnection* /*nc*/,
    natsSubscription* /*sub*/,
    natsMsg* msg,
    void* closure)
{
    auto* cb = reinterpret_cast<OnMessage*>(closure); // 우리가 new 한 std::function*
    if (cb) {
        const char* subj = natsMsg_GetSubject(msg);
        const char* data = natsMsg_GetData(msg);
        const int   len = natsMsg_GetDataLength(msg);

        std::string subject = subj ? subj : "";
        std::string payload(data, len);

        (*cb)(subject, payload);
    }
    natsMsg_Destroy(msg);
}


void DSSVssClient::natsRawMsgThunk(natsConnection* /*nc*/,
    natsSubscription* /*sub*/,
    natsMsg* msg,
    void* closure)
{
    if (closure) {
        auto* cb = reinterpret_cast<std::function<void(std::string, std::vector<uint8_t>)>*>(closure);
        if (cb) {
            const char* subj = natsMsg_GetSubject(msg);
            const char* data = natsMsg_GetData(msg);
            const int len = natsMsg_GetDataLength(msg);

            std::string subject = subj ? subj : "";
            std::vector<uint8_t> payload(data, data + len);

            // 안전한 콜백 호출
            (*cb)(subject, payload);
        }
    }

    // 메시지 메모리 해제
    natsMsg_Destroy(msg);
}

// ------------------------------
// Subscribe
// ------------------------------
natsSubscription* DSSVssClient::subscribe(const std::string& subject,OnMessage onMessage,natsStatus* outStatus)
{
    if (outStatus) *outStatus = NATS_OK;
    if (conn == nullptr) {
        if (outStatus) *outStatus = NATS_CONNECTION_CLOSED;
        return nullptr;
    }
    auto* closure = new OnMessage(std::move(onMessage));
    natsSubscription* sub = nullptr;
    natsStatus s = natsConnection_Subscribe(
        &sub,
        conn,
        subject.c_str(),
        &DSSVssClient::natsMsgThunk,
        closure
    );

    if (s != NATS_OK) {
        delete closure;
        if (outStatus) *outStatus = s;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        gClosures[sub] = closure;
    }

    return sub;
}



// ------------------------------
// Subscribe
// ------------------------------
natsSubscription* DSSVssClient::subscribe(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus)
{
    if (outStatus) *outStatus = NATS_OK;
    if (conn == nullptr) {
        if (outStatus) *outStatus = NATS_CONNECTION_CLOSED;
        return nullptr;
    }
    auto* closure = new OnRawMessage(std::move(onMessage));
    natsSubscription* sub = nullptr;
    natsStatus s = natsConnection_Subscribe(
        &sub,
        conn,
        subject.c_str(),
        &DSSVssClient::natsRawMsgThunk,
        closure
    );

    if (s != NATS_OK) {
        delete closure;
        if (outStatus) *outStatus = s;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        gRawClosures[sub] = closure;
    }

    return sub;
}



// ------------------------------
// Subscribe
// ------------------------------
natsSubscription* DSSVssClient::subscribeSensor(const std::string& subject, OnRawMessage onMessage, natsStatus* outStatus)
{
    if (outStatus) *outStatus = NATS_OK;
    if (conn == nullptr) {
        if (outStatus) *outStatus = NATS_CONNECTION_CLOSED;
        return nullptr;
    }
    auto* closure = new OnRawMessage(std::move(onMessage));
    natsSubscription* sub = nullptr;
    natsStatus s = natsConnection_Subscribe(
        &sub,
        conn,
        subject.c_str(),
        &DSSVssClient::natsRawMsgThunk,
        closure
    );

    if (s != NATS_OK) {
        delete closure;
        if (outStatus) *outStatus = s;
        return nullptr;
    }

    {
        std::lock_guard<std::mutex> lock(gSubMx);
        gRawClosures[sub] = closure;
    }

    return sub;
}


