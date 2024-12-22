/*
 * WiFiSocket.h
 *
 *  Created on: 22 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_WIFISOCKET_H_
#define SRC_NETWORKING_WIFISOCKET_H_

#include "RepRapFirmware.h"

#if HAS_WIFI_NETWORKING

#include "Networking/NetworkDefs.h"
#include "Networking/Socket.h"

class WiFiInterface;

class WiFiSocket : public Socket
{
public:
	explicit WiFiSocket(NetworkInterface *_ecv_from iface) noexcept;
	void Init(SocketNumber n) noexcept;
	int State() const noexcept { return (int)state; }				// used only for reporting debug info, hence the 'int' return
	void SetNeedsPolling() noexcept { needsPolling = true; }
	bool NeedsPolling() const noexcept;

	void Poll() noexcept override;
	void Close() noexcept override;
	bool IsClosing() const noexcept { return (state == SocketState::closing); }
	void Terminate() noexcept override;
	void TerminateAndDisable() noexcept override { Terminate(); }
	bool ReadChar(char& c) noexcept override;
	bool ReadBuffer(const uint8_t *_ecv_array &buffer, size_t &len) noexcept override;
	void Taken(size_t len) noexcept override;
	bool CanRead() const noexcept override;
	bool CanSend() const noexcept override;
	size_t Send(const uint8_t *_ecv_array data, size_t length) noexcept override;
	void Send() noexcept override;

private:
	enum class SocketState : uint8_t
	{
		inactive,
		connecting,
		waitingForResponder,
		connected,
		peerDisconnecting,
		closing,
		broken
	};

	WiFiInterface *GetInterface() const noexcept;
	void ReceiveData(uint16_t bytesAvailable) noexcept;
	void DiscardReceivedData() noexcept;

	NetworkBuffer *_ecv_null receivedData;				// List of buffers holding received data
	bool hasMoreDataPending;							// If there is more data left to read when the buffered data has been processed
	uint32_t whenInState;								// General purpose timekeeping value for duration spent in various socket states
	uint16_t txBufferSpace;								// How much free transmit buffer space the WiFi mofule reported
	SocketNumber socketNum;								// The WiFi socket number we are using
	SocketState state;
	bool needsPolling;
};

#endif	// HAS_WIFI_NETWORKING

#endif /* SRC_NETWORKING_WIFISOCKET_H_ */
